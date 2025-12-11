// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

// mapping
#include <Eigen/Dense>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// standard
#include <list>
#include <mutex>
#include <map>

// symbols for factor graph keys
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct ImuData
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
};

struct LidarData
{
    std::vector<Eigen::Vector3d> points;
    std::vector<double> offset_times;
};

struct ScanBuffer
{
    /// @brief Pointcloud of undistorted (and voxelized) scan
    std::shared_ptr<open3d::geometry::PointCloud> pcd;
    /// @brief Pose of the scan w.r.t. the latest keyframe pose
    std::shared_ptr<gtsam::Pose3> kf_T_scan;
};

enum SystemState
{
    initializing,
    tracking,
    recovery,
};

/// @brief Convert LidarData to Open3D PointCloud (no undistortion).
/// @param lidar_data The raw LiDAR scan, potentially undistorted.
/// @return an Open3D PointCloud, with scan timestamp info removed.
open3d::geometry::PointCloud Scan2PCD(const std::shared_ptr<LidarData> &lidar_data, const double minPointDist, const double maxPointDist)
{
    open3d::geometry::PointCloud pcd;
    size_t point_num = lidar_data->points.size();
    pcd.points_.reserve(point_num);
    for (size_t i = 0; i < point_num; ++i)
    {
        const Eigen::Vector3d &pt = lidar_data->points[i];
        const double dist = pt.norm();
        if (dist < minPointDist || dist > maxPointDist)
            continue;
        pcd.points_.push_back(lidar_data->points[i]);
    }
    return pcd;
}

/// @brief Indexing of points in a submap for building point clusters
/// @details Uses `int` for point index because that's what Open3Ds KNN search returns.
using SubmapIdxPointIdx = std::pair<u_int32_t, int>;
/// @brief Type used to identify point clusters
using ClusterId = u_int32_t;
constexpr ClusterId INVALID_CLUSTER_ID = std::numeric_limits<ClusterId>::max();

class PointToPlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{

public:
    PointToPlaneFactor(
        const gtsam::Key key,                                            // key of the NavState variable in the smoothers factor graph
        const gtsam::Pose3 &imu_T_lidar,                                 // fixed extrinsic calibration
        const std::vector<std::shared_ptr<Eigen::Vector3d>> &scanPoints, // plane points in the LiDAR frame
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,             // plane normal in the world frame
        const double planeNormalOffsetD,                                 // plane normal offset d in the world frame s.t. n^T * x - d = 0
        const gtsam::SharedNoiseModel &noiseModel                        // point-to-plane noise model
        ) : NoiseModelFactor1<gtsam::Pose3>(noiseModel, key),
            imu_T_lidar(imu_T_lidar),
            scanPoints(scanPoints),
            planeNormal(planeNormal),
            planeNormalOffsetD(planeNormalOffsetD) {};

    gtsam::Vector evaluateError(
        const gtsam::Pose3 &w_T_imu,                     ///< current estimate of the IMU pose in the world frame
        boost::optional<gtsam::Matrix &> H = boost::none ///< optional Jacobian matrix
    ) const override
    {
        // transform plane points to world frame
        gtsam::Pose3 w_T_lidar = w_T_imu.compose(imu_T_lidar);
        size_t numPoints = scanPoints.size();
        gtsam::Vector errorVec(numPoints);
        for (size_t i = 0; i < numPoints; ++i)
        {
            gtsam::Point3 w_p = w_T_lidar.transformFrom(gtsam::Point3(*scanPoints[i]));
            errorVec(i) = planeNormal->dot(w_p) - planeNormalOffsetD;
        }

        if (H)
        {
            (*H) = gtsam::Matrix(numPoints, 6);
            for (size_t i = 0; i < numPoints; ++i)
            {
                gtsam::Point3 w_p = w_T_lidar.transformFrom(gtsam::Point3(*scanPoints[i]));
                // derivative of point-to-plane distance w.r.t. pose perturbation
                gtsam::Matrix16 D_error_D_pose;
                const Eigen::Matrix<double, 1, 3> nT = planeNormal->transpose();
                D_error_D_pose.row(i).head<3>() = -nT * gtsam::skewSymmetric(w_p);  // rotation
                D_error_D_pose.row(i).tail<3>() = nT * w_T_imu.rotation().matrix(); // translation
                (*H).row(i) = D_error_D_pose;
            }
        }
        return errorVec;
    }

private:
    // input parameters
    const gtsam::Pose3 imu_T_lidar;
    const std::vector<std::shared_ptr<Eigen::Vector3d>> scanPoints;
    const std::shared_ptr<Eigen::Vector3d> planeNormal;
    const double planeNormalOffsetD;
};

// NOTE: this class should not use ROS-specific stuff,
// as it should ultimately be separated from the ROS implementation
class MapperSystem
{
public:
    MapperSystem()
    {
        /**
         * TODO: readout parameters from config file or ROS params and set them here
         */
        auto params{gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU()};
        // TODO: set sensible values for Livox Mid360 IMU here
        constexpr double accelerometerNoise{0.15}, gyroscopeNoise{0.314};
        params->accelerometerCovariance = gtsam::I_3x3 * std::pow(accelerometerNoise, 2);
        params->gyroscopeCovariance = gtsam::I_3x3 * std::pow(gyroscopeNoise, 2);
        params->integrationCovariance = gtsam::I_3x3 * 1e-6;
        gtsam::imuBias::ConstantBias priorImuBias{}; // default used for example
        preintegrator = gtsam::PreintegratedCombinedMeasurements(params, priorImuBias);
    }
    ~MapperSystem() {}

    void feedImu(const std::shared_ptr<ImuData> imu_data, const double &timestamp)
    {
        imuBuffer[timestamp] = imu_data;
    };

    void feedLidar(const std::shared_ptr<LidarData> lidar_data, const double &timestamp)
    {
        lidarBuffer[timestamp] = lidar_data;
    };

    void update()
    {
        std::cout << "SLAM has " << imuBuffer.size() << " IMU messages and "
                  << lidarBuffer.size() << " LIDAR messages buffered." << std::endl;

        switch (systemState)
        {
        case SystemState::initializing:
        {
            const double maxBufferTime = lidarBuffer.rbegin()->first;
            if (maxBufferTime >= initTimeWindow)
            {
                std::cout << "Initialization complete. Switching to tracking state." << std::endl;
                systemState = SystemState::tracking;
            }
            break;
        }
        case SystemState::tracking:
        {
            track();
            break;
        }
        case SystemState::recovery:
        {
            break;
        }
        }
    };

    // functions
private:
    /// @brief Static state (- assumption based) system intialization
    /// @details
    /// - uses all IMU measurements up to the first LIDAR timestamp
    /// - computes mean accelerometer and gyro measurements
    /// - builds initial orientation from mean accelerometer (gravity alignment)
    /// - creates initial keyframe submap from all buffered LIDAR scans
    void initializeSystem()
    {
        // lock the buffers for accessing values
        std::lock_guard<std::mutex> lockImuBuffer(mtxImuBuffer), lockLidarBuffer(mtxLidarBuffer);
        const double tInit = lidarBuffer.rbegin()->first;
        // upper bound key for all imu samples up to and including tInit
        auto imuBufferEndIt{imuBuffer.upper_bound(tInit)};
        auto lidarBufferEndIt{lidarBuffer.upper_bound(tInit)};
        // gravity direction is obtained from mean accelerometer measurement
        const double numImuSamples{static_cast<double>(std::distance(imuBuffer.begin(), imuBufferEndIt))};
        Eigen::Vector3d accMean{Eigen::Vector3d::Zero()};
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
            accMean += it->second->acceleration;
        accMean /= numImuSamples;
        std::cout << "Mean accelerometer measurement during initialization: " << std::endl
                  << accMean.transpose() << std::endl;
        // build gravity-aligned global reference frame (only roll & pitch are observable)
        Eigen::Vector3d
            zAxis{accMean.normalized()},
            // orthogonal projection of global x-axis onto plane normal to zAxis
            // x_orthog = (I - zz^T) * x
            xAxis{((Eigen::Matrix3d::Identity() - zAxis * zAxis.transpose()) * Eigen::Vector3d::UnitX()).normalized()},
            yAxis{zAxis.cross(xAxis)};
        // build initial orientation matrix
        Eigen::Matrix3d w_R_i0;
        w_R_i0.col(0) = xAxis;
        w_R_i0.col(1) = yAxis;
        w_R_i0.col(2) = zAxis;
        w_R_i0.transposeInPlace(); // rotation from initial IMU frame to world frame
        // full initial pose
        gtsam::Pose3 w_T_i0{gtsam::Rot3(w_R_i0), gtsam::Point3(0, 0, 0)};
        // create new submap with all initial lidar scans
        open3d::geometry::PointCloud newSubmap;
        for (auto it = lidarBuffer.begin(); it != lidarBufferEndIt; ++it)
        {
            open3d::geometry::PointCloud pcdScan = Scan2PCD(it->second, minPointDist, maxPointDist);
            pcdScan.VoxelDownSample(0.5);
            newSubmap += pcdScan;
        }
        const u_int32_t idxNewKF = createKeyframeSubmap(w_T_i0.compose(imu_T_lidar), tLastImu, newSubmap);
        // clear lidar buffer
        lidarBuffer.erase(lidarBuffer.begin(), lidarBufferEndIt);
        // acceleration bias is unobservable because we the mean value is used for gravity alignment
        // gyro bias is the mean of all gyro measurements (because no rotation is assumed)
        Eigen::Vector3d gyroBiasMean{Eigen::Vector3d::Zero()};
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
            gyroBiasMean += it->second->angular_velocity;
        gyroBiasMean /= numImuSamples;
        // variances are computed w.r.t. the mean for acceleration and 0 for gyro measurements
        double accVariance{0.0}, gyroVariance{0.0};
        for (auto it = imuBuffer.begin(); it != imuBufferEndIt; ++it)
        {
            accVariance += (it->second->acceleration - accMean).squaredNorm();
            gyroVariance += (it->second->angular_velocity).squaredNorm();
        }
        accVariance /= (numImuSamples - 1.0);
        gyroVariance /= (numImuSamples - 1.0);
        preintegrator.params()->accelerometerCovariance = gtsam::I_3x3 * accVariance;
        preintegrator.params()->gyroscopeCovariance = gtsam::I_3x3 * gyroVariance;
        // set bias (use existing acceleration bias)
        gtsam::imuBias::ConstantBias priorImuBias{preintegrator.biasHat().accelerometer(), gyroBiasMean};
        preintegrator.resetIntegrationAndSetBias(priorImuBias);
        // construct Navigation State prior (mean & covariance)
        resetNewFactors();
        gtsam::Vector6 priorPoseSigma;
        priorPoseSigma << 1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3;
        // pose prior should be certain
        newSmootherFactors.addPrior(X(idxNewKF), w_T_i0, gtsam::noiseModel::Diagonal::Sigmas(priorPoseSigma));
        newSmootherFactors.addPrior(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Isotropic::Sigma(3, 1e-6));
        // bias prior, very noisy
        newSmootherFactors.addPrior(B(idxNewKF), priorImuBias.vector(), gtsam::noiseModel::Isotropic::Sigma(6, 3.0 * std::max(accVariance, gyroVariance)));
        newValues.insert(X(idxNewKF), w_T_i0);
        newValues.insert(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()));
        newValues.insert(B(idxNewKF), priorImuBias);
        currState = gtsam::NavState(w_T_i0, gtsam::Vector3::Zero());
        lastKeyframeState = currState;
        const double kfInit{static_cast<double>(idxNewKF)};
        // set index of all values
        for (auto const &val : newValues)
            newSmootherIndices[val.key] = kfInit;
        // clear imu buffer and store timestamp of last IMU reading
        tLastImu = imuBufferEndIt->first;
        imuBuffer.erase(imuBuffer.begin(), imuBufferEndIt);
    }

    void track()
    {

        // NOTE: it this point, the buffers should contain only unprocessed measurements
        // lock the buffers for accessing values
        std::unique_lock<std::mutex> lockImuBuffer(mtxImuBuffer);
        // delete imu measurements that are older than the latest processed imu timestamp (should not happen)
        imuBuffer.erase(imuBuffer.begin(), imuBuffer.upper_bound(tLastImu));
        std::cout << "::: [WARNING] the system is discarding unprocessed IMU measurements (too old), this should not happen! :::" << std::endl;
        // delete lidar scans that are older than the latest processed lidar timestamp (should not happen, warn if it does)
        lidarBuffer.erase(lidarBuffer.begin(), lidarBuffer.upper_bound(tLastImu));
        std::cout << "::: [ERROR] discarded unprocessed LiDAR scan (too old), this should not happen! :::" << std::endl;
        std::cout << "::: [DEBUG] begin IMU preintegration :::" << std::endl;
        // perform preintegration to decide whether a new keyframe is needed
        for (auto imuIt = imuBuffer.begin(); imuIt != imuBuffer.end(); ++imuIt)
        {
            const auto [timestamp, u] = *imuIt;
            const double dt{imuIt == imuBuffer.begin() ? timestamp - tLastImu : timestamp - std::prev(imuIt)->first};
            // integrate measurement
            preintegrator.integrateMeasurement(u->acceleration, u->angular_velocity, dt);
        }
        std::cout << "::: [DEBUG] IMU preintegration completed :::" << std::endl;
        // save last imu timestamp and clear the buffer
        tLastImu = imuBuffer.rbegin()->first;
        imuBuffer.clear();
        lockImuBuffer.unlock();
        const gtsam::NavState propState{preintegrator.predict(currState, currBias)}; // imu-propagared state
        const double positionDiff{(propState.pose().translation() - currState.pose().translation()).norm()};
        currState = propState; // need to update
        // iterator to the newest lidar scan that can still be processed (older than latest imu time    stamp)
        std::unique_lock<std::mutex> lockLidarBuffer(mtxLidarBuffer);
        auto lidarBufferEndIt{lidarBuffer.upper_bound(tLastImu)};
        if (std::distance(lidarBuffer.begin(), lidarBufferEndIt) == 0)
        {
            std::cout << "::: [WARN] cannot process LiDAR scans, no IMU readings to predict pose! :::" << std::endl;
            return;
        }

        std::cout << "::: [DEBUG] begin undistorting LiDAR scans :::" << std::endl;
        // --- undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation ---
        // delta pose to last submap at preintegrated state / last IMU time (!!)
        gtsam::Pose3 deltaPoseToSubmap{lastKeyframeState.pose().between(currState.pose().compose(imu_T_lidar))};
        const double
            tLastKeyframe{keyframeTimestamps.rbegin()->second}, // timestamp of last keyframe
            dtPropToKeyframe{tLastImu - tLastKeyframe};         // time delta to last keyframe
        // undistort between individual scans
        gtsam::Pose3 deltaPoseLastScanToKeyframe{gtsam::Pose3::Identity()};
        double dtLastScanToKeyframe{0.0};
        for (auto it = lidarBuffer.begin(); it != lidarBufferEndIt; ++it)
        {
            const auto [tScan, scan] = *it;
            const double dtScanToKeyframe{tScan - tLastKeyframe};
            // pose delta of this scan to last keyframe
            // compute pose delta to last submap at current scan time (extrapolate with constant velocity)
            // linear and angular velocity are obtained from pose delta over preintegration time
            const gtsam::Vector3
                angVel{gtsam::Rot3::Logmap(deltaPoseToSubmap.rotation()) / dtPropToKeyframe},
                linVel{deltaPoseToSubmap.translation() / dtPropToKeyframe};
            // apply scan to keyframe delta time to get pose delta of the scan w.r.t. the last keyframe
            gtsam::Rot3 deltaRotScanToKeyframe{gtsam::Rot3::Expmap(angVel * dtScanToKeyframe)};
            gtsam::Point3 deltaTransScanToKeyframe{linVel * dtScanToKeyframe};
            gtsam::Pose3 deltaPoseScanToKeyframe{deltaRotScanToKeyframe, deltaTransScanToKeyframe};
            // pose & time delta from this to last scan
            gtsam::Pose3 deltaPoseScanToScan{deltaPoseLastScanToKeyframe.between(deltaPoseScanToKeyframe)};
            double dtScanToScan{dtScanToKeyframe - dtLastScanToKeyframe};

            // undistort current scan
            for (std::size_t i = 0; i < scan->points.size(); ++i)
            {
                const Eigen::Vector3d pt = scan->points[i];
                const double dt = scan->offset_times[i];
                const gtsam::Vector3 angVel{gtsam::Rot3::Logmap(deltaPoseScanToScan.rotation())};
                // presumed rotation and translation that the point should have undergone during scan time
                gtsam::Rot3 R{gtsam::Rot3::Expmap(angVel / dtScanToScan * dt)};
                gtsam::Point3 t{deltaPoseScanToScan.translation() / dtScanToScan * dt};
                gtsam::Pose3 T{R, t};
                // undistort point
                const Eigen::Vector3d ptUndistorted{T.transformFrom(pt)};
                scan->points[i] = ptUndistorted;
            }
            // convert scan to pointcloud, voxelize, transform to keyframe pose and add to submap
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, minPointDist, maxPointDist);
            pcdScan.VoxelDownSample(voxelSize);
            gtsam::Pose3 scanPoseInWorld{keyframePoses.rbegin()->second->compose(deltaPoseScanToKeyframe)}; // pose in world frame
            pcdScan.Transform(scanPoseInWorld.matrix());
            // buffer undistorted scan
            bufferScan(deltaPoseScanToKeyframe, pcdScan);
        }
        std::cout << "::: [DEBUG] completed undistorting LiDAR scans, proceeding to identify keyframe :::" << std::endl;
        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // erase processed raw scan data and release buffer lock
        lidarBuffer.erase(lidarBuffer.begin(), lidarBufferEndIt);
        lockLidarBuffer.unlock();

        if (positionDiff < threshNewKeyframeDist)
            return;
        std::cout << "::: [INFO] identified keyframe, creating new submap :::" << std::endl;
        // --- create new keyframe ---
        // merge buffered scans together & create new keyframe submap
        open3d::geometry::PointCloud newSubmap;
        for (const ScanBuffer &scan : scanBuffer)
        {
            // move all scans to same origin
            scan.pcd->Transform(scan.kf_T_scan->matrix());
            newSubmap += *(scan.pcd);
        }
        newSubmap.VoxelDownSample(voxelSize);
        const u_int32_t idxKeyframe = createKeyframeSubmap(currState.pose().compose(imu_T_lidar), tLastImu, newSubmap); // NOTE: also clears the scan buffer

        std::cout << "::: [DEBUG] identifying same-plane point clusters among keyframe submaps :::" << std::endl;
        // search for same-plane point clusters among all keyframes
        const std::shared_ptr<open3d::geometry::PointCloud> pcdQuery = keyframeSubmaps[idxKeyframe];
        std::vector<int> knnIndices(knnMaxNeighbors); // will be used for NN search results
        std::vector<double> knnDists(knnMaxNeighbors);
        // TODO: make this an unordered_map, but requires custom hashing and equality OPs for the key type
        std::map<SubmapIdxPointIdx, ClusterId> clusterTracks;
        // find nearest neighbors between the new submap and all other keyframe submaps
        for (auto itOtherSubmap = keyframeSubmaps.begin(); itOtherSubmap != keyframeSubmaps.end(); ++itOtherSubmap)
        {
            // skip self
            const u_int32_t idxOtherSubmap = itOtherSubmap->first;
            if (idxOtherSubmap == idxKeyframe)
                continue;
            const int numQueryPts = pcdQuery->points_.size();
            for (int idxPt = 0; idxPt < numQueryPts; ++idxPt)
            {
                knnIndices.clear();
                knnDists.clear();
                const int knnFound = submapKDTrees[idxOtherSubmap]->SearchHybrid(
                    pcdQuery->points_[idxPt], knnRadius, knnMaxNeighbors, knnIndices, knnDists);
                if (knnFound > 0)
                {
                    ClusterId clusterId = INVALID_CLUSTER_ID, tempNewCluster = clusterIdCounter + 1;
                    // clusters that need to be merged (polled & processed after search)
                    std::vector<ClusterId> duplicateClusters;
                    // identify or assign cluster to every NN returned point
                    for (int i = 0; i < knnFound; ++i)
                    {
                        SubmapIdxPointIdx idxOther{idxOtherSubmap, knnIndices[i]};
                        if (clusterTracks.find(idxOther) != clusterTracks.end())
                        {
                            clusterTracks[idxOther] = tempNewCluster;
                        }
                        else
                        {
                            // point belongs to a single existing cluster
                            if (clusterId == INVALID_CLUSTER_ID)
                            {
                                clusterId = clusterTracks[idxOther];
                                // may have assigned temp cluster, which needs to be replaced
                                duplicateClusters.push_back(tempNewCluster);
                            }
                            else // point belongs to multiple existing clusters, need to merge
                                duplicateClusters.push_back(clusterTracks[idxOther]);
                        }

                        // if a new cluster was identified, increment the counter
                        if (clusterId == INVALID_CLUSTER_ID)
                            clusterIdCounter = clusterId = tempNewCluster;
                        // this point belongs to an existing cluster
                        else // merge all duplicate clusters with the identified one
                            for (const ClusterId &duplicateId : duplicateClusters)
                                for (auto &entry : clusterTracks)
                                    if (entry.second == duplicateId)
                                        entry.second = clusterId;
                        // add the current point to the identified cluster
                        SubmapIdxPointIdx idxCurrent{idxKeyframe, idxPt};
                        clusterTracks[idxCurrent] = clusterId;
                    }
                }
            }
        }
        std::cout << "::: [DEBUG] cluster identification completed, proceeding with outlier rejection :::" << std::endl;
        // join tracks by clusters
        std::unordered_map<ClusterId, std::vector<SubmapIdxPointIdx>> clusters;
        std::unordered_map<ClusterId, double> clusterPlaneThickness;
        std::unordered_map<ClusterId, std::shared_ptr<Eigen::Vector3d>> clusterCenters, clusterNormals;
        for (auto itTrack = clusterTracks.begin(); itTrack != clusterTracks.end(); ++itTrack)
        {
            const auto [idxPoint, clusterId] = *itTrack;
            if (clusters.find(clusterId) == clusters.end())
                clusters[clusterId] = std::vector<SubmapIdxPointIdx>{idxPoint};
            else
                clusters[clusterId].push_back(idxPoint);
        }
        // remove clusters that are too small
        for (auto itCluster = clusters.begin(); itCluster != clusters.end();)
        {
            if (itCluster->second.size() < minClusterSize)
                itCluster = clusters.erase(itCluster);
            else
                ++itCluster;
        }
        /**
         * TODO: cluster validation & outlier rejection
         * - fit planes to each cluster and remove those with high fitting error (no RANSAC, use all points)
         * - compute plane thickness, plane factor covariance and standard deviation which is used for validation
         *  - MSC-LIO Eq. (18-19) and last paragraphs of IV-D
         * - construct structureless point-to-plane factors for all points in remaining valid clusters
         * - add factors to the factor graph
         * - should existing factors from previous updates be removed? how does iSAM2 handle them?
         *  - check Kimera implementation
         */
        // outlier rejection - remove clusters that aren't planes
        for (auto itCluster = clusters.begin(); itCluster != clusters.end();)
        {
            size_t numPoints{0};
            // compute centered point locations for plane fitting
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const SubmapIdxPointIdx &idxPoint : itCluster->second)
            {
                auto const [idxSubmap, pointIdx] = idxPoint;
                if (idxSubmap == idxKeyframe) // plane thickness and normal will not be computed using points from the new keyframe
                    continue;
                centroid += keyframeSubmaps[idxSubmap]->points_[pointIdx];
                numPoints++;
            }
            centroid /= static_cast<double>(numPoints);

            Eigen::MatrixXd A(numPoints, 3);
            size_t i = 0;
            for (const SubmapIdxPointIdx &idxPoint : itCluster->second)
            {
                auto const [idxSubmap, pointIdx] = idxPoint;
                if (idxSubmap == idxKeyframe) // plane thickness and normal will not be computed using points from the new keyframe
                    continue;
                A.row(i++) = keyframeSubmaps[idxSubmap]->points_[pointIdx] - centroid;
            }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const Eigen::Vector3d planeNormal = svd.matrixV().col(2).normalized();
            // check if plane is valid - all points need to be withing threshold distance to plane
            bool isPlaneValid{true};
            double planeThickness{0.0};
            for (size_t idxPt = 0; idxPt < numPoints; idxPt++)
            {
                const double pointToPlaneDist{std::abs(planeNormal.dot(A.row(idxPt)))};
                if (pointToPlaneDist > threshPlaneValid)
                {
                    isPlaneValid = false;
                    break;
                }
                else
                    planeThickness += std::pow(pointToPlaneDist, 2);
            }
            // remove tracked cluster if plane is not valid
            if (!isPlaneValid)
                itCluster = clusters.erase(itCluster);
            else
            {
                planeThickness /= static_cast<double>(numPoints);
                clusterPlaneThickness[itCluster->first] = planeThickness;
                clusterCenters[itCluster->first] = std::make_shared<Eigen::Vector3d>(centroid);
                clusterNormals[itCluster->first] = std::make_shared<Eigen::Vector3d>(planeNormal);
                ++itCluster;
            }
        }
        std::cout << "::: [DEBUG] outlier rejection completed, proceeding to formulate tracking constraints :::" << std::endl;
        // build residuals for active keyframe points from valid clusters
        for (auto itValidCluster = clusters.begin(); itValidCluster != clusters.end(); ++itValidCluster)
        {
            const ClusterId clusterId = itValidCluster->first;
            const std::vector<SubmapIdxPointIdx> &clusterPoints = itValidCluster->second;
            const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters[clusterId];
            const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals[clusterId];
            const double planeThickness = clusterPlaneThickness[clusterId];
            // noise model for point-to-plane factor
            gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(planeThickness));
            std::vector<std::shared_ptr<Eigen::Vector3d>> planePoints; // this clusters points in the lidar frame of the active keyframe
            for (const SubmapIdxPointIdx &idxPoint : clusterPoints)
            {
                auto const [keySubmap, pointIdx] = idxPoint;
                if (keySubmap != idxKeyframe)
                    continue; // only create factors for points in the new keyframe
                // scan point in LiDAR frame
                planePoints.push_back(std::make_shared<Eigen::Vector3d>(
                    keyframeSubmaps[keySubmap]->points_[pointIdx] - keyframePoses[idxKeyframe]->translation()));
            }
            // add point to plane factors for all points in the active keyframe
            if (planePoints.size() > 0)
            {
                newSmootherFactors.add(boost::make_shared<PointToPlaneFactor>(
                    X(idxKeyframe),
                    imu_T_lidar,
                    planePoints,
                    clusterNormal,
                    clusterNormal->dot(*clusterCenter), // d in plane equation s.t. n^T * x - d = 0
                    noiseModel));
                newValues.insert(X(idxKeyframe), currState.pose());
                newSmootherIndices[X(idxKeyframe)] = static_cast<double>(idxKeyframe);
            }
        }
        // predicted state
        newValues.insert(X(idxKeyframe), currState.pose());
        newValues.insert(V(idxKeyframe), currState.v());
        newValues.insert(B(idxKeyframe), currBias);
        // preintegration factor
        newSmootherFactors.add(
            gtsam::CombinedImuFactor(
                X(idxKeyframe - 1), V(idxKeyframe - 1), // previous pose and velocity
                X(idxKeyframe), V(idxKeyframe),         // new pose and velocity
                B(idxKeyframe - 1), B(idxKeyframe),     // previous bias and new bias
                preintegrator));
        // smoother indices used for marginalization
        newSmootherIndices[X(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        newSmootherIndices[V(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        newSmootherIndices[B(idxKeyframe - 1)] = static_cast<double>(idxKeyframe - 1);
        // update smoother, TODO: identify marginalized keyframes and remove their submap data
        smoother.update(newSmootherFactors, newValues, newSmootherIndices);
        resetNewFactors(); // clear factor, index & value buffers
        // extract estimated state
        currState = gtsam::NavState{
                smoother.calculateEstimate<gtsam::Pose3>(X(idxKeyframe)),
                smoother.calculateEstimate<gtsam::Vector3>(V(idxKeyframe))};
        currBias = smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(B(idxKeyframe));
        lastKeyframeState = currState;
        // reset preintegrator
        preintegrator.resetIntegrationAndSetBias(currBias);
    };

    void resetNewFactors()
    {
        newSmootherFactors.resize(0);
        newValues.clear();
        newSmootherIndices.clear();
    };

    /// @brief Store new keyframe submap and initialized pose, increment keyframe counter and clear the scan buffer.
    /// @param keyframePose
    /// @param keyframeTimestamp
    /// @param keyframeSubmap
    /// @return
    u_int32_t createKeyframeSubmap(const gtsam::Pose3 &keyframePose, const double &keyframeTimestamp, open3d::geometry::PointCloud keyframeSubmap)
    {
        // increment keyframe counter
        const u_int32_t idxNewKf = keyframeCounter++;
        // store keyframe data
        auto ptrSubmap{std::make_shared<open3d::geometry::PointCloud>(keyframeSubmap)};
        // transform submap to its estimated pose in the world frame
        ptrSubmap->Transform(keyframePose.matrix());
        keyframeSubmaps[idxNewKf] = ptrSubmap;
        keyframePoses[idxNewKf] = std::make_shared<gtsam::Pose3>(keyframePose);
        keyframeTimestamps[idxNewKf] = keyframeTimestamp;
        // initialize a KD-Tree for point cluster search
        submapKDTrees[idxNewKf] = std::make_shared<open3d::geometry::KDTreeFlann>(*ptrSubmap);
        // clear scan buffer
        scanBuffer.clear();
        return idxNewKf;
    };

    void updateKeyframeSubmapPose(const u_int32_t keyframeIdx, const gtsam::Pose3 &newWorldPose)
    {
        const gtsam::Pose3 deltaPose{keyframePoses[keyframeIdx]->between(newWorldPose)};
        keyframeSubmaps[keyframeIdx]->Transform(deltaPose.matrix());
        keyframePoses[keyframeIdx] = std::make_shared<gtsam::Pose3>(newWorldPose);
    };

    /// @brief Add a new (undistorted) scan pointcloud to the scan buffer
    /// @param scanPoseToLastKeyframe
    /// @param pcdScan
    void bufferScan(const gtsam::Pose3 &scanPoseToLastKeyframe, open3d::geometry::PointCloud pcdScan)
    {
        scanBuffer.push_back(ScanBuffer{std::make_shared<open3d::geometry::PointCloud>(pcdScan),
                                        std::make_shared<gtsam::Pose3>(scanPoseToLastKeyframe)});
    };

private:
    // input data buffers
    std::map<double, std::shared_ptr<ImuData>> imuBuffer;
    std::map<double, std::shared_ptr<LidarData>> lidarBuffer;
    // IMU preintegrator
    gtsam::PreintegratedCombinedMeasurements preintegrator;
    // fixed lag smoother
    gtsam::BatchFixedLagSmoother smoother{static_cast<double>(slidingWindowSize)};
    // factors, nodes and timestamps to add to the graph
    gtsam::NonlinearFactorGraph newSmootherFactors;              // new factors appended to the smoother
    gtsam::Values newValues;                                     // estimated values of the factors above
    gtsam::FixedLagSmoother::KeyTimestampMap newSmootherIndices; // KF-indices of the factors in the smoother
    // lifecycle management & state estimation
    SystemState systemState{SystemState::initializing};
    u_int32_t keyframeCounter{0}; // used to feed nodes to the factor graph
    /// @brief mutex for accessing the system state & buffers
    std::mutex mtxState, mtxLidarBuffer, mtxImuBuffer;
    /// @brief current estimated state (propagated OR updated)
    /// @details **NOTE** that this is **NOT** the single best estimate as it is used for IMU propagation
    gtsam::NavState currState, lastKeyframeState;
    gtsam::imuBias::ConstantBias currBias;
    double tLastImu{0.0};
    // --- mapping ---
    // unmerged scan data
    std::list<ScanBuffer> scanBuffer;
    // keyframe data
    std::map<u_int32_t, std::shared_ptr<open3d::geometry::PointCloud>> keyframeSubmaps;
    std::map<u_int32_t, std::shared_ptr<open3d::geometry::KDTreeFlann>> submapKDTrees;
    std::map<u_int32_t, std::shared_ptr<gtsam::Pose3>> keyframePoses;
    std::map<u_int32_t, double> keyframeTimestamps;
    // --- calibration ---

    /// @brief Extrinsics from IMU to LiDAR frame.
    /// @details See Mid360 User Manual, p. 15, Paragraph "IMU Data"
    const gtsam::Pose3 imu_T_lidar{gtsam::Rot3::Identity(), gtsam::Point3(-0.011, -0.02329, 0.04412)};
    // (same plane) point clusters
    ClusterId clusterIdCounter{0};
    // configuration constants
    static constexpr int slidingWindowSize{7};                     // in keyframes (including preintegration factors)
    static constexpr double initTimeWindow{2.0};                   // time used for (assumed) static state initialization, in seconds
    static constexpr double voxelSize{0.5};                        // voxel size for submap construction
    static constexpr double threshNewKeyframeDist{0.5};            // threshold for new keyframe creation based on position change
    static constexpr double minPointDist{1.5}, maxPointDist{60.0}; // min/max point distance for lidar points to be considered
    static constexpr double knnRadius{0.5}, knnMaxNeighbors{5};    // parameters for KD-Tree search for plane point clusters
    static constexpr size_t minClusterSize{5};                     // minimum number of points in a cluster to start tracking
    static constexpr double threshPlaneValid{0.1};                 // maximum distance from all cluster pts to fitted plane for validity
};

class MapperNode : public rclcpp::Node
{
public:
public:
    MapperNode() : Node("mapper_node")
    {
        RCLCPP_INFO(this->get_logger(), "MapperNode has been initialized.");

        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&MapperNode::imuCallback, this, std::placeholders::_1));

        lidar_subscription = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "livox/lidar", 10, std::bind(&MapperNode::lidarCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received IMU message.");
        if (!has_start_time)
        {
            start_time = rclcpp::Time(msg->header.stamp);
            has_start_time = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - start_time).seconds();
        // build imu data container from msg
        auto imu_data = std::make_shared<ImuData>();
        imu_data->acceleration = Eigen::Vector3d(
                                     msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z) *
                                 livox_imu_scale; // convert from [g] to [m/s^2]
        imu_data->angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        slam.feedImu(imu_data, timestamp);
    }

    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received LIDAR message.");
        if (!has_start_time)
        {
            start_time = rclcpp::Time(msg->header.stamp);
            has_start_time = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - start_time).seconds();
        // build lidar data container from msg
        auto lidar_data = std::make_shared<LidarData>();
        size_t point_num = msg->points.size();
        lidar_data->points.reserve(point_num);
        lidar_data->offset_times.reserve(point_num);
        for (size_t i = 0; i < point_num; ++i)
        {
            const auto &point = msg->points[i];
            Eigen::Vector3d pt(
                static_cast<double>(point.x),
                static_cast<double>(point.y),
                static_cast<double>(point.z));
            lidar_data->points.push_back(pt);
            // point offset time is given in nanoseconds
            rclcpp::Time point_offset_time(static_cast<uint64_t>(point.offset_time));
            lidar_data->offset_times.push_back(point_offset_time.seconds());
        }
        slam.feedLidar(lidar_data, timestamp);
        slam.update();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription;
    rclcpp::Time start_time;
    bool has_start_time = false;
    static constexpr double livox_imu_scale{9.81};

    MapperSystem slam;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}
