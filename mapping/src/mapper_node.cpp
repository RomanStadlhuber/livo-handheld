// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

// mapping
#include <Eigen/Dense>
#include <open3d/geometry/PointCloud.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// standard
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

enum SystemState
{
    initializing,
    tracking,
    recovery,
};

/// @brief Convert LidarData to Open3D PointCloud (no undistortion).
/// @param lidar_data The raw LiDAR scan, potentially undistorted.
/// @return an Open3D PointCloud, with scan timestamp info removed.
open3d::geometry::PointCloud Scan2PCD(const std::shared_ptr<LidarData> &lidar_data)
{
    open3d::geometry::PointCloud pcd;
    size_t point_num = lidar_data->points.size();
    pcd.points_.reserve(point_num);
    for (size_t i = 0; i < point_num; ++i)
    {
        pcd.points_.push_back(lidar_data->points[i]);
    }
    return pcd;
}

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
        newSmootherFactors.addPrior(X(keyframeCounter), w_T_i0, gtsam::noiseModel::Diagonal::Sigmas(priorPoseSigma));
        newSmootherFactors.addPrior(V(keyframeCounter), gtsam::Vector3(gtsam::Vector3::Zero()), gtsam::noiseModel::Isotropic::Sigma(3, 1e-6));
        // bias prior, very noisy
        newSmootherFactors.addPrior(B(keyframeCounter), priorImuBias.vector(), gtsam::noiseModel::Isotropic::Sigma(6, 3.0 * std::max(accVariance, gyroVariance)));
        newVaules.insert(X(keyframeCounter), w_T_i0);
        newVaules.insert(V(keyframeCounter), gtsam::Vector3(gtsam::Vector3::Zero()));
        newVaules.insert(B(keyframeCounter), priorImuBias);
        currState = gtsam::NavState(w_T_i0, gtsam::Vector3::Zero());
        lastKeyframeState = currState;
        const double kfInit{static_cast<double>(keyframeCounter)};
        // set index of all values
        for (auto const &val : newVaules)
            newSmootherIndices[val.key] = kfInit;
        // clear imu buffer and store timestamp of last IMU reading
        tLastImu = imuBufferEndIt->first;
        imuBuffer.erase(imuBuffer.begin(), imuBufferEndIt);
        // create new submap with all initial lidar scans
        currentSubmap.Clear();
        for (auto it = lidarBuffer.begin(); it != lidarBufferEndIt; ++it)
        {
            open3d::geometry::PointCloud scanPcd = Scan2PCD(it->second);
            scanPcd.VoxelDownSample(0.5);
            currentSubmap += scanPcd;
        }
        lastKeyframePose = w_T_i0;
        tLastKeyframe = lidarBufferEndIt->first;
        // clear lidar buffer
        lidarBuffer.erase(lidarBuffer.begin(), lidarBufferEndIt);
    }

    void track()
    {

        // NOTE: it this point, the buffers should contain only unprocessed measurements
        // lock the buffers for accessing values
        std::lock_guard<std::mutex> lockImuBuffer(mtxImuBuffer), lockLidarBuffer(mtxLidarBuffer);
        // delete imu measurements that are older than the latest processed imu timestamp (should not happen)
        imuBuffer.erase(imuBuffer.begin(), imuBuffer.upper_bound(tLastImu));
        std::cout << "::: [WARNING] the system is discarding unprocessed IMU measurements (too old), this should not happen! :::" << std::endl;
        // delete lidar scans that are older than the latest processed lidar timestamp (should not happen, warn if it does)
        lidarBuffer.erase(lidarBuffer.begin(), lidarBuffer.upper_bound(tLastImu));
        std::cout << "::: [ERROR] discarded unprocessed LiDAR scan (too old), this should not happen! :::" << std::endl;
        // perform preintegration to decide whether a new keyframe is needed
        for (auto imuIt = imuBuffer.begin(); imuIt != imuBuffer.end(); ++imuIt)
        {
            const auto [timestamp, u] = *imuIt;
            const double dt{imuIt == imuBuffer.begin() ? timestamp - tLastImu : timestamp - std::prev(imuIt)->first};
            // integrate measurement
            preintegrator.integrateMeasurement(u->acceleration, u->angular_velocity, dt);
        }
        // save last imu timestamp and clear the buffer
        tLastImu = imuBuffer.rbegin()->first;
        imuBuffer.clear();
        const gtsam::NavState propState{preintegrator.predict(currState, currBias)}; // imu-propagared state
        const double positionDiff{(propState.pose().translation() - currState.pose().translation()).norm()};
        currState = propState; // need to update
        // iterator to the newest lidar scan that can still be processed (older than latest imu timestamp)
        auto lidarBufferEndIt{lidarBuffer.upper_bound(tLastImu)};
        if (std::distance(lidarBuffer.begin(), lidarBufferEndIt) == 0)
        {
            std::cout << "::: [WARN] cannot process LiDAR scans, no IMU readings to predict pose! :::" << std::endl;
            return;
        }

        if (positionDiff >= threshNewKeyframeDist) // create new submap & add preintegration factor to the graph
        {
            /**
             * TODO:
             * - make undistortion modular
             * - this likely means that that I have no move a large portion of the code below to a function
             * - should LiDAR tracking residuals only be added when a new keyframe is created?
             * - w.r.t. which keyframe pose should the new scans be undistorted here? I think it should not matter..
             */
        }
        else // undistort scans and add them to the current keyframe submap
        {
            // delta pose to last submap at preintegrated state / last IMU time (!!)
            gtsam::Pose3 deltaPoseToSubmap{lastKeyframeState.pose().between(currState.pose())};
            // time delta to last keyframe
            const double dtPropToKeyframe{tLastImu - tLastKeyframe};
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
                for(std::size_t i=0; i < scan->points.size(); ++i)
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
                open3d::geometry::PointCloud scanPcd = Scan2PCD(scan);
                scanPcd.VoxelDownSample(voxelSize);
                gtsam::Pose3 scanPoseInWorld{lastKeyframePose.compose(deltaPoseScanToKeyframe)}; // pose in world frame
                scanPcd.Transform(scanPoseInWorld.matrix()); // move to world frame w.r.t. last KF
                currentSubmap += scanPcd; // add to current submap
            }
        }
    }

    void resetNewFactors()
    {
        newSmootherFactors.resize(0);
        newVaules.clear();
        newSmootherIndices.clear();
    };

private:
    // input datat buffers
    std::map<double, std::shared_ptr<ImuData>> imuBuffer;
    std::map<double, std::shared_ptr<LidarData>> lidarBuffer;
    // IMU preintegrator
    gtsam::PreintegratedCombinedMeasurements preintegrator;
    // fixed lag smoother
    gtsam::BatchFixedLagSmoother smoother{static_cast<double>(slidingWindowSize)};
    // factors, nodes and timestamps to add to the graph
    gtsam::NonlinearFactorGraph newSmootherFactors;              // new factors appended to the smoother
    gtsam::Values newVaules;                                     // estimated values of the factors above
    gtsam::FixedLagSmoother::KeyTimestampMap newSmootherIndices; // KF-indices of the factors in the smoother
    // lifecycle management & state estimation
    SystemState systemState{SystemState::initializing};
    u_int32_t keyframeCounter{0}; // used to feed nodes to the factor graph
    /// @brief mutex for accessing the system state & buffers
    std::mutex mtxState, mtxLidarBuffer, mtxImuBuffer;
    /// @brief current estimated state (propagated OR updated)
    /// @details **NOTE** that this is **NOT** the single best estimate as it is used for IMU propagation
    gtsam::NavState currState, lastKeyframeState;
    gtsam::Pose3 lastKeyframePose; // pose of the last processed LIDAR scan, may be from IMU propagation or KF update
    gtsam::imuBias::ConstantBias currBias;
    double tLastImu{0.0}, tLastKeyframe{0.0};
    // mapping
    open3d::geometry::PointCloud currentSubmap;
    std::map<u_int32_t, open3d::geometry::PointCloud> keyframeSubmaps;
    // configuration constants
    static constexpr int slidingWindowSize{7};          // in keyframes (including preintegration factors)
    static constexpr double initTimeWindow{2.0};        // time used for (assumed) static state initialization, in seconds
    static constexpr double voxelSize{0.5};             // voxel size for submap construction
    static constexpr double threshNewKeyframeDist{0.5}; // threshold for new keyframe creation based on position change
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
            msg->linear_acceleration.z);
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

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription;
    rclcpp::Time start_time;
    bool has_start_time = false;

    MapperSystem slam;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0;
}