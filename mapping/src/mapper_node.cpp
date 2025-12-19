// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

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
#include <memory>
#include <algorithm>
#include <set>

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

class PointToPlaneFactor : public gtsam::NoiseModelFactor
{
public:
    /// @brief Constructor for point-to-plane factor with multiple keyframe poses
    /// @param keys Vector of pose keys that this factor connects (e.g., X(0), X(1), X(3))
    /// @param imu_T_lidar Fixed extrinsic calibration
    /// @param scanPointsPerKey Map from key index (in keys vector) to scan points in that keyframe's LiDAR frame
    /// @param planeNormal Plane normal in the world frame
    /// @param planeNormalOffsetD Plane offset d in world frame (n^T * x - d = 0)
    /// @param noiseModel Point-to-plane noise model
    PointToPlaneFactor(
        const gtsam::KeyVector &keys,
        const gtsam::Pose3 &imu_T_lidar,
        const std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> &scanPointsPerKey,
        const std::shared_ptr<Eigen::Vector3d> &planeNormal,
        const double planeNormalOffsetD,
        const gtsam::SharedNoiseModel &noiseModel)
        : NoiseModelFactor(noiseModel, keys),
          imu_T_lidar(imu_T_lidar),
          scanPointsPerKey(scanPointsPerKey),
          planeNormal(planeNormal),
          planeNormalOffsetD(planeNormalOffsetD)
    {
        // Calculate total number of points for dimensionality
        totalPoints = 0;
        for (const auto &[keyIdx, points] : scanPointsPerKey)
        {
            totalPoints += points.size();
        }
    }

    /// @brief Evaluate error for all poses involved in this factor
    /// @param values Current estimates of all variables in the factor graph
    /// @param H Optional Jacobians w.r.t. each pose
    gtsam::Vector unwhitenedError(
        const gtsam::Values &values,
        boost::optional<std::vector<gtsam::Matrix> &> H = boost::none) const override
    {
        gtsam::Vector errorVec(totalPoints);

        // If Jacobians requested, initialize them
        if (H)
        {
            H->resize(keys().size());
            for (size_t i = 0; i < keys().size(); ++i)
            {
                (*H)[i] = gtsam::Matrix::Zero(totalPoints, 6);
            }
        }

        size_t errorIdx = 0;

        // Iterate over each keyframe that has observations
        for (const auto &[keyIdx, scanPoints] : scanPointsPerKey)
        {
            // Get the pose estimate for this keyframe
            const gtsam::Pose3 w_T_imu = values.at<gtsam::Pose3>(keys()[keyIdx]);
            const gtsam::Pose3 w_T_lidar = w_T_imu.compose(imu_T_lidar);

            // Compute error and Jacobian for each point from this keyframe
            for (size_t ptIdx = 0; ptIdx < scanPoints.size(); ++ptIdx, ++errorIdx)
            {
                const gtsam::Point3 lidar_p(*scanPoints[ptIdx]);
                const gtsam::Point3 w_p = w_T_lidar.transformFrom(lidar_p);

                // Point-to-plane distance error
                errorVec(errorIdx) = planeNormal->dot(w_p) - planeNormalOffsetD;

                // Compute Jacobian if requested
                if (H)
                {
                    // Jacobian w.r.t. the pose of this keyframe
                    const Eigen::Matrix<double, 1, 3> nT = planeNormal->transpose();

                    // Rotation component: -n^T * [w_p]_x (where [.]_x is skew-symmetric)
                    gtsam::Matrix16 D_error_D_pose;
                    D_error_D_pose.head<3>() = -nT * gtsam::skewSymmetric(w_p);

                    // Translation component: n^T * R
                    D_error_D_pose.tail<3>() = nT * w_T_imu.rotation().matrix();

                    // Assign to the appropriate Jacobian matrix
                    (*H)[keyIdx].row(errorIdx) = D_error_D_pose;
                }
            }
        }

        return errorVec;
    }

    /// @brief Clone method required for GTSAM factor copying
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::make_shared<PointToPlaneFactor>(
            keys(), imu_T_lidar, scanPointsPerKey, planeNormal, planeNormalOffsetD, noiseModel());
    }

private:
    const gtsam::Pose3 imu_T_lidar;
    const std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey;
    const std::shared_ptr<Eigen::Vector3d> planeNormal;
    const double planeNormalOffsetD;
    size_t totalPoints;
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
                initializeSystem();
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
            newSubmap += *(pcdScan.VoxelDownSample(voxelSize));
        }
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(voxelSize);
        const u_int32_t idxNewKF = createKeyframeSubmap(w_T_i0.compose(imu_T_lidar), tLastImu, ptrNewSubmapVoxelized);
        // clear lidar buffer
        lidarBuffer.clear();
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
        newSmootherFactors.addPrior(B(idxNewKF), priorImuBias, gtsam::noiseModel::Isotropic::Sigma(6, 3.0 * std::max(accVariance, gyroVariance)));
        newValues.insert(X(idxNewKF), w_T_i0);
        newValues.insert(V(idxNewKF), gtsam::Vector3(gtsam::Vector3::Zero()));
        newValues.insert(B(idxNewKF), priorImuBias);
        w_X_curr = gtsam::NavState(w_T_i0, gtsam::Vector3::Zero());
        const double kfInit{static_cast<double>(idxNewKF)};
        // set index of all values
        for (auto const &val : newValues)
            newSmootherIndices[val.key] = kfInit;
        // clear imu buffer and store timestamp of last IMU reading
        tLastImu = imuBufferEndIt->first;
        imuBuffer.clear();
    }

    void track()
    {

        // NOTE: it this point, the buffers should contain only unprocessed measurements
        // lock the buffers for accessing values
        std::unique_lock<std::mutex> lockImuBuffer(mtxImuBuffer);
        /*
        // delete imu measurements that are older than the latest processed imu timestamp (should not happen)
        imuBuffer.erase(imuBuffer.begin(), imuBuffer.upper_bound(tLastImu));
        std::cout << "::: [WARNING] the system is discarding unprocessed IMU measurements (too old), this should not happen! :::" << std::endl;
        // delete lidar scans that are older than the latest processed lidar timestamp (should not happen, warn if it does)
        lidarBuffer.erase(lidarBuffer.begin(), lidarBuffer.upper_bound(tLastImu));
        std::cout << "::: [ERROR] discarded unprocessed LiDAR scan (too old), this should not happen! :::" << std::endl;
        std::cout << "::: [DEBUG] begin IMU preintegration :::" << std::endl;
         */
        std::cout << "::: [DEBUG] tracking with " << imuBuffer.size() << " IMU measurements and " << lidarBuffer.size() << " LiDAR scans :::" << std::endl;

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
        const gtsam::NavState w_X_propagated{preintegrator.predict(w_X_curr, currBias)}; // imu-propagated state
        const double positionDiff{(w_X_propagated.pose().translation() - lastKeyframePose().translation()).norm()};
        w_X_curr = w_X_propagated; // need to update
        // iterator to the newest lidar scan that can still be processed (older than latest imu timestamp)
        std::unique_lock<std::mutex> lockLidarBuffer(mtxLidarBuffer);
        std::cout << "::: [DEBUG] tLastIMU " << tLastImu << ", tLastLiDAR " << (lidarBuffer.size() ? lidarBuffer.rbegin()->first : 0) << " :::" << std::endl;
        // --- undistort incoming scans w.r.t. the last keyframe pose & buffer them for new keyframe creation ---
        // delta pose to last submap at preintegrated state / last IMU time (!!)
        gtsam::Pose3 kf_T_prop{lastKeyframePose().inverse().compose(w_X_curr.pose().compose(imu_T_lidar))};
        std::cout << "::: [DEBUG] has " << keyframeTimestamps.size() << " keyframes :::" << std::endl;
        const double
            tLastKeyframe{keyframeTimestamps.rbegin()->second}, // timestamp of last keyframe
            dtPropToKeyframe{tLastImu - tLastKeyframe};         // time delta to last keyframe
        std::cout << "::: [DEBUG] begin undistorting " << lidarBuffer.size() << " LiDAR scans :::" << std::endl;
        // undistort between individual scans
        gtsam::Pose3 deltaPoseLastScanToKeyframe{gtsam::Pose3::Identity()};
        for (auto it = lidarBuffer.begin(); it != lidarBuffer.end(); ++it)
        {
            const auto [tScan, scan] = *it;
            const double dtScanToKeyframe{tScan - tLastKeyframe};
            // pose delta of this scan to last keyframe
            // compute pose delta to last submap at current scan time (extrapolate with constant velocity)
            // linear and angular velocity are obtained from pose delta over preintegration time
            const gtsam::Vector3
                angVel{gtsam::Rot3::Logmap(kf_T_prop.rotation()) / dtPropToKeyframe},
                linVel{kf_T_prop.translation() / dtPropToKeyframe};
            // apply scan to keyframe delta time to get pose delta of the scan w.r.t. the last keyframe
            gtsam::Rot3 kf_R_scan{gtsam::Rot3::Expmap(angVel * dtScanToKeyframe)};
            gtsam::Point3 kf_P_scan{linVel * dtScanToKeyframe};
            gtsam::Pose3 kf_T_scan{kf_R_scan, kf_P_scan};
            // pose & time delta from this to last scan
            gtsam::Pose3 lastScan_T_currScan{deltaPoseLastScanToKeyframe.between(kf_T_scan)};

            // undistort current scan
            for (std::size_t i = 0; i < scan->points.size(); ++i)
            {
                const Eigen::Vector3d pt = scan->points[i];
                const double dt = scan->offset_times[i];
                const gtsam::Vector3 angVel{gtsam::Rot3::Logmap(lastScan_T_currScan.rotation())};
                // presumed rotation and translation that the point should have undergone during scan time
                gtsam::Rot3 scan_R_pt{gtsam::Rot3::Expmap(angVel * dt)};
                gtsam::Point3 scan_P_pt{lastScan_T_currScan.translation() * dt};
                gtsam::Pose3 scan_T_pt{scan_R_pt, scan_P_pt};
                // undistort point
                const Eigen::Vector3d ptUndistorted{scan_T_pt.transformFrom(pt)};
                scan->points[i] = ptUndistorted;
            }
            std::cout << "::: [DEBUG] undistorted LiDAR scan with " << scan->points.size() << " points :::" << std::endl;
            // convert scan to pointcloud, voxelize, transform to keyframe pose and add to submap
            open3d::geometry::PointCloud pcdScan = Scan2PCD(scan, minPointDist, maxPointDist);
            std::shared_ptr<open3d::geometry::PointCloud> ptrPcdScanVoxelized = pcdScan.VoxelDownSample(voxelSize);
            gtsam::Pose3 scanPoseInWorld{keyframePoses.rbegin()->second->compose(kf_T_scan)}; // pose in world frame
            ptrPcdScanVoxelized->Transform(scanPoseInWorld.matrix());
            // buffer undistorted scan
            bufferScan(kf_T_scan, ptrPcdScanVoxelized);
            deltaPoseLastScanToKeyframe = kf_T_scan;
        }
        std::cout << "::: [DEBUG] completed undistorting LiDAR scans, proceeding to identify keyframe :::" << std::endl;
        // NOTE: at this point all scans have been undistorted and buffered, so we only need to use the PCD buffer
        // erase processed raw scan data and release buffer lock
        lidarBuffer.clear();
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
        std::shared_ptr<open3d::geometry::PointCloud> ptrNewSubmapVoxelized = newSubmap.VoxelDownSample(voxelSize);
        const u_int32_t idxKeyframe = createKeyframeSubmap(w_X_curr.pose().compose(imu_T_lidar), tLastImu, ptrNewSubmapVoxelized); // NOTE: also clears the scan buffer
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
                if (knnFound > 4)
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
        std::cout << "::: [DEBUG] identified " << clusters.size() << " clusters, proceeding with outlier rejection :::" << std::endl;
        // outlier rejection - remove clusters that aren't planes
        for (auto itCluster = clusters.begin(); itCluster != clusters.end();)
        {
            size_t numHistoricPoints{0}; // no. of points from historic keyframes
            // compute centered point locations for plane fitting
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const SubmapIdxPointIdx &idxPoint : itCluster->second)
            {
                auto const [idxSubmap, pointIdx] = idxPoint;
                if (idxSubmap == idxKeyframe) // plane thickness and normal will not be computed using points from the new keyframe
                    continue;
                centroid += keyframeSubmaps[idxSubmap]->points_[pointIdx];
                numHistoricPoints++;
            }
            if (numHistoricPoints < 4) // need at least 3 points to fit a plane
            {
                itCluster = clusters.erase(itCluster);
                continue;
            }
            const double n{static_cast<double>(numHistoricPoints)};
            centroid /= n;
            Eigen::MatrixXd A(numHistoricPoints, 3);
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
            // check if plane is valid - all points need to be within threshold distance to plane
            bool isPlaneValid{true};
            double planeThickness{0.0};
            for (size_t idxPt = 0; idxPt < numHistoricPoints; idxPt++)
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
                planeThickness /= n;
                clusterPlaneThickness[itCluster->first] = planeThickness;
                clusterCenters[itCluster->first] = std::make_shared<Eigen::Vector3d>(centroid);
                clusterNormals[itCluster->first] = std::make_shared<Eigen::Vector3d>(planeNormal);
                ++itCluster;
            }
        }
        if (clusters.size() == 0)
        {
            // TODO: tracking lost
            std::cout << "::: [WARNING] no valid point clusters found for keyframe " << idxKeyframe << ", skipping constraint creation :::" << std::endl;
            return;
        }
        std::cout << "::: [DEBUG] outlier rejection completed (keeping " << clusters.size() << " clusters), proceeding to formulate tracking constraints :::" << std::endl;
        // build residuals for active keyframe points from valid clusters
        for (auto itValidCluster = clusters.begin(); itValidCluster != clusters.end(); ++itValidCluster)
        {
            const ClusterId clusterId = itValidCluster->first;
            const std::vector<SubmapIdxPointIdx> &clusterPoints = itValidCluster->second;
            const std::shared_ptr<Eigen::Vector3d> clusterCenter = clusterCenters[clusterId];
            const std::shared_ptr<Eigen::Vector3d> clusterNormal = clusterNormals[clusterId];
            const double planeThickness = clusterPlaneThickness[clusterId];

            // Step 1: Collect unique keyframe IDs involved in this cluster
            std::set<u_int32_t> uniqueKeyframeIds;
            for (const SubmapIdxPointIdx &idxPoint : clusterPoints)
            {
                uniqueKeyframeIds.insert(idxPoint.first);
            }

            // Step 2: Build sorted keys vector and mapping from keyframe ID to index in keys vector
            gtsam::KeyVector keys;
            keys.reserve(uniqueKeyframeIds.size());
            std::unordered_map<u_int32_t, size_t> keyframeIdToKeyIdx;
            size_t keyIdx = 0;
            for (const u_int32_t kfId : uniqueKeyframeIds)
            {
                keys.push_back(X(kfId));
                keyframeIdToKeyIdx[kfId] = keyIdx++;
            }

            // Step 3: Build scanPointsPerKey using indices into keys vector (not keyframe IDs)
            std::unordered_map<size_t, std::vector<std::shared_ptr<Eigen::Vector3d>>> scanPointsPerKey;
            size_t totalPoints = 0;
            for (const SubmapIdxPointIdx &idxPoint : clusterPoints)
            {
                auto const [keyframeId, pointIdx] = idxPoint;
                const size_t keyVecIdx = keyframeIdToKeyIdx[keyframeId];
                // Point in LiDAR frame = world point - keyframe translation
                scanPointsPerKey[keyVecIdx].push_back(std::make_shared<Eigen::Vector3d>(
                    keyframeSubmaps[keyframeId]->points_[pointIdx] - keyframePoses[keyframeId]->translation()));
                totalPoints++;
            }

            // noise model for point-to-plane factor (one sigma per point measurement)
            gtsam::SharedNoiseModel noiseModel = gtsam::noiseModel::Isotropic::Sigma(totalPoints, std::sqrt(planeThickness));
            // create a point-to-plane factor for this cluster
            newSmootherFactors.add(boost::make_shared<PointToPlaneFactor>(
                keys,
                imu_T_lidar,
                scanPointsPerKey,
                clusterNormal,
                clusterNormal->dot(*clusterCenter), // d in plane equation s.t. n^T * x - d = 0
                noiseModel));
        }
        // add variables for the active keyframe
        newValues.insert(X(idxKeyframe), w_X_curr.pose());
        newValues.insert(V(idxKeyframe), w_X_curr.v());
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
        smoother.update(newSmootherFactors, newValues, newSmootherIndices);
        smoother.print("Smoother after update:\n");
        resetNewFactors(); // clear factor, index & value buffers
        // extract estimated state
        w_X_curr = gtsam::NavState{
            smoother.calculateEstimate<gtsam::Pose3>(X(idxKeyframe)),
            smoother.calculateEstimate<gtsam::Vector3>(V(idxKeyframe))};
        currBias = smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(B(idxKeyframe));
        w_X_curr.print();
        // reset preintegrator
        preintegrator.resetIntegrationAndSetBias(currBias);
        if (idxKeyframe > slidingWindowSize)
        {
            u_int32_t
                idxLowerBound = keyframeSubmaps.begin()->first,
                idxUpperbound = idxKeyframe - slidingWindowSize;

            std::cout << "::: [DEBUG] marginalizing " << (idxUpperbound - idxLowerBound) << " keyframes :::" << std::endl;
            for (u_int32_t idx = idxLowerBound; idx < idxUpperbound; ++idx)
            {
                if (keyframeSubmaps.find(idx) != keyframeSubmaps.end())
                {
                    keyframeSubmaps.erase(idx);
                    submapKDTrees.erase(idx);
                    keyframePoses.erase(idx);
                    keyframeTimestamps.erase(idx);
                }
            }
        }
        // update the poses of the keyframe submaps
        for (auto const &[idxKf, _] : keyframeSubmaps)
        {
            const gtsam::Pose3 updatedPose = smoother.calculateEstimate<gtsam::Pose3>(X(idxKf));
            updateKeyframeSubmapPose(idxKf, updatedPose);
        }
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
    u_int32_t createKeyframeSubmap(const gtsam::Pose3 &keyframePose, const double &keyframeTimestamp, std::shared_ptr<open3d::geometry::PointCloud> ptrKeyframeSubmap)
    {
        // increment keyframe counter
        const u_int32_t idxNewKf = keyframeCounter++;
        // transform submap to its estimated pose in the world frame
        ptrKeyframeSubmap->Transform(keyframePose.matrix());
        keyframeSubmaps[idxNewKf] = ptrKeyframeSubmap;
        keyframePoses[idxNewKf] = std::make_shared<gtsam::Pose3>(keyframePose);
        keyframeTimestamps[idxNewKf] = keyframeTimestamp;
        std::cout << "::: [DEBUG] created keyframe " << idxNewKf << " (" << ptrKeyframeSubmap->points_.size() << " pts ) at timestamp " << keyframeTimestamp << " :::" << std::endl;
        // initialize a KD-Tree for point cluster search
        submapKDTrees[idxNewKf] = std::make_shared<open3d::geometry::KDTreeFlann>(*ptrKeyframeSubmap);
        // clear scan buffer
        scanBuffer.clear();
        return idxNewKf;
    };

    gtsam::Pose3 lastKeyframePose() const
    {
        return *keyframePoses.rbegin()->second;
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
    void bufferScan(const gtsam::Pose3 &scanPoseToLastKeyframe, std::shared_ptr<open3d::geometry::PointCloud> pcdScan)
    {
        scanBuffer.push_back(ScanBuffer{pcdScan, std::make_shared<gtsam::Pose3>(scanPoseToLastKeyframe)});
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
    gtsam::NavState w_X_curr;
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
    double lidarTimeOffset{0.0}; // temporal calibration such that t_imu = t_lidar + lidarTimeOffset
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

// Standalone bag reader for debugging
void runFromBag(const std::string &bag_path)
{
    MapperSystem slam;

    // Setup bag reader
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);

    rclcpp::Time start_time;
    bool has_start_time = false;
    constexpr double livox_imu_scale{9.81};

    std::cout << "Reading bag file: " << bag_path << std::endl;

    while (reader.has_next())
    {
        auto bag_message = reader.read_next();

        if (bag_message->topic_name == "/livox/imu")
        {
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
            serialization.deserialize_message(&serialized_msg, msg.get());

            if (!has_start_time)
            {
                start_time = rclcpp::Time(msg->header.stamp);
                has_start_time = true;
            }

            double timestamp = (rclcpp::Time(msg->header.stamp) - start_time).seconds();
            auto imu_data = std::make_shared<ImuData>();
            imu_data->acceleration = Eigen::Vector3d(
                                         msg->linear_acceleration.x,
                                         msg->linear_acceleration.y,
                                         msg->linear_acceleration.z) *
                                     livox_imu_scale;
            imu_data->angular_velocity = Eigen::Vector3d(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);

            slam.feedImu(imu_data, timestamp);
        }
        else if (bag_message->topic_name == "/livox/lidar" || bag_message->topic_name == "livox/lidar")
        {
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
            auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
            rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
            serialization.deserialize_message(&serialized_msg, msg.get());

            if (!has_start_time)
            {
                start_time = rclcpp::Time(msg->header.stamp);
                has_start_time = true;
            }

            double timestamp = (rclcpp::Time(msg->header.stamp) - start_time).seconds();
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
                rclcpp::Time point_offset_time(static_cast<uint64_t>(point.offset_time));
                lidar_data->offset_times.push_back(point_offset_time.seconds());
            }

            slam.feedLidar(lidar_data, timestamp);
            slam.update();
        }
    }

    std::cout << "Finished processing bag file." << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Check if running in standalone bag mode
    if (argc >= 3 && std::string(argv[1]) == "--bag")
    {
        std::cout << "Running in standalone bag reader mode" << std::endl;
        runFromBag(argv[2]);
    }
    else
    {
        std::cout << "Running as ROS 2 node" << std::endl;
        rclcpp::spin(std::make_shared<MapperNode>());
    }

    rclcpp::shutdown();
    return 0;
}
