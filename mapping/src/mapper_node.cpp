/// @file
/// @ingroup system
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

// Mapping
#include <mapping/MappingSystem.hpp>
#ifdef MAPPING_BENCHMARKING
#include <mapping/benchmarking/RunOutput.hpp>
#include <mapping/benchmarking/TelemetryRecorder.hpp>
#include <mapping/benchmarking/TrajectoryRecorder.hpp>
#endif

// publishing the state
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <open3d_conversions/open3d_conversions.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// configuration form config_utilities
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum)
{
    (void)signum;
    g_shutdown_requested = true; // only async-signal-safe work here, the bag reader polls this flag
}

mapping::MappingConfig loadConfig(const std::string &config_path)
{
    std::cout << "Loading config from: " << config_path << std::endl;
    mapping::MappingConfig config = config::fromYamlFile<mapping::MappingConfig>(config_path);
    config::checkValid(config);
    std::cout << "Loaded config:\n" << config::toString(config) << std::endl;
    return config;
}

// --- message conversions, shared by the ROS node and the standalone bag reader ---

/// @brief Convert an IMU message to `mapping::ImuData`.
/// @param scalingAcc Scale applied to the accelerometer readings (Livox reports in `[g]`, i.e. use 9.81).
std::shared_ptr<mapping::ImuData> imuMsgToImuData(const sensor_msgs::msg::Imu &msg, const double scalingAcc)
{
    auto imuData = std::make_shared<mapping::ImuData>();
    imuData->acceleration =
        Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) * scalingAcc;
    imuData->angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    return imuData;
}

/// @brief Convert a Livox `CustomMsg` scan to `mapping::LidarData`.
/// @details Per-point offset times are given in `[ns]` w.r.t. the header stamp and converted to `[s]`.
std::shared_ptr<mapping::LidarData> livoxMsgToLidarData(const livox_ros_driver2::msg::CustomMsg &msg)
{
    auto lidarData = std::make_shared<mapping::LidarData>();
    lidarData->points.reserve(msg.points.size());
    lidarData->offset_times.reserve(msg.points.size());
    for (const auto &point : msg.points)
    {
        lidarData->points.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y),
                                       static_cast<double>(point.z));
        lidarData->offset_times.push_back(static_cast<double>(point.offset_time) / 1e9);
    }
    return lidarData;
}

/// @brief Convert a generic `PointCloud2` scan to `mapping::LidarData`.
/// @details Expects `FLOAT32` fields `x`, `y`, `z`.
/// If a `FLOAT64` field `timestamp` exists (e.g. Hesai PandarXT in the HILTI 2022 bags), it holds the absolute
/// per-point time in `[s]` and is converted to an offset w.r.t. the header stamp (= scan start).
/// Otherwise all offset times are 0, which makes undistortion a no-op for this scan.
/// Non-finite points are dropped (`Scan2PCD` range filtering does not catch NaNs).
/// If any |offset| exceeds 1 [s] (per-point clock does not match the header clock), all offsets of the scan are set to
/// 0.
/// @return `nullptr` if the message layout is not supported (warning is logged), the caller must skip the scan.
std::shared_ptr<mapping::LidarData> pointCloud2ToLidarData(const sensor_msgs::msg::PointCloud2 &msg)
{
    // --- validate the message layout, the iterators below do not check bounds ---
    auto findField = [&msg](const std::string &name) -> const sensor_msgs::msg::PointField *
    {
        for (const auto &field : msg.fields)
            if (field.name == name)
                return &field;
        return nullptr;
    };
    for (const char *name : {"x", "y", "z"})
    {
        const auto *field = findField(name);
        if (!field || field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->count != 1 ||
            static_cast<size_t>(field->offset) + sizeof(float) > msg.point_step)
        {
            std::cerr << "Warning: skipping PointCloud2 scan, field '" << name
                      << "' is missing or not a FLOAT32 within point_step" << std::endl;
            return nullptr;
        }
    }
    if (msg.is_bigendian)
    {
        std::cerr << "Warning: skipping PointCloud2 scan, big-endian data is not supported" << std::endl;
        return nullptr;
    }
    const size_t width = msg.width, height = msg.height, pointStep = msg.point_step, rowStep = msg.row_step;
    // the iterators walk the data linearly in steps of point_step, so rows must not be padded
    if (rowStep < width * pointStep || (height > 1 && rowStep != width * pointStep) ||
        msg.data.size() < height * rowStep)
    {
        std::cerr << "Warning: skipping PointCloud2 scan, inconsistent layout (width " << width << ", height " << height
                  << ", point_step " << pointStep << ", row_step " << rowStep << ", data size " << msg.data.size()
                  << ")" << std::endl;
        return nullptr;
    }

    auto lidarData = std::make_shared<mapping::LidarData>();
    const size_t numPoints = width * height;
    lidarData->points.reserve(numPoints);
    lidarData->offset_times.reserve(numPoints);

    const auto *fieldTime = findField("timestamp");
    const bool hasPointTimestamps = fieldTime && fieldTime->datatype == sensor_msgs::msg::PointField::FLOAT64 &&
                                    static_cast<size_t>(fieldTime->offset) + sizeof(double) <= msg.point_step;
    static std::once_flag warnNoTimestamp, warnClockMismatch;
    if (!hasPointTimestamps)
        std::call_once(warnNoTimestamp,
                       []
                       {
                           std::cerr << "Warning: PointCloud2 has no FLOAT64 'timestamp' field, "
                                        "falling back to 0 per-point offset times (no undistortion)"
                                     << std::endl;
                       });
    std::optional<sensor_msgs::PointCloud2ConstIterator<double>> iterTime;
    if (hasPointTimestamps)
        iterTime.emplace(msg, "timestamp");
    // subtract the integer seconds first to keep sub-microsecond precision of the absolute timestamps
    const double secHeader = static_cast<double>(msg.header.stamp.sec),
                 nsecHeader = static_cast<double>(msg.header.stamp.nanosec) / 1e9;

    sensor_msgs::PointCloud2ConstIterator<float> iterX(msg, "x"), iterY(msg, "y"), iterZ(msg, "z");
    for (size_t i = 0; i < numPoints; ++i, ++iterX, ++iterY, ++iterZ)
    {
        double offsetTime = 0.0;
        if (iterTime)
        {
            offsetTime = (**iterTime - secHeader) - nsecHeader;
            ++(*iterTime);
        }
        const Eigen::Vector3d point(static_cast<double>(*iterX), static_cast<double>(*iterY),
                                    static_cast<double>(*iterZ));
        if (!point.allFinite())
            continue;
        lidarData->points.push_back(point);
        lidarData->offset_times.push_back(offsetTime);
    }
    constexpr double maxPlausibleOffset = 1.0; // [s]
    if (std::any_of(lidarData->offset_times.begin(), lidarData->offset_times.end(),
                    [](const double t) { return std::abs(t) > maxPlausibleOffset; }))
    {
        std::call_once(
            warnClockMismatch,
            []
            {
                std::cerr << "Warning: PointCloud2 per-point times differ from the header stamp by more "
                             "than 1 s (clock mismatch?), setting offset times to 0 for such scans (logged once)"
                          << std::endl;
            });
        std::fill(lidarData->offset_times.begin(), lidarData->offset_times.end(), 0.0);
    }
    return lidarData;
}

class MapperNode : public rclcpp::Node
{
public:
    explicit MapperNode() : Node("mapper_node"), slam_()
    {

        RCLCPP_INFO(this->get_logger(), "MapperNode has been initialized.");
        this->declare_parameter<std::string>("mapper_config", "");
        this->declare_parameter<std::string>("topic_imu", "/livox/imu");
        this->declare_parameter<double>("scaling_imu", 9.81);
        this->declare_parameter<std::string>("topic_lidar", "/livox/lidar");
        this->declare_parameter<bool>("lidar_msg_pc2", false);

        std::string configPath = this->get_parameter("mapper_config").as_string();
        const std::string topicImu = this->get_parameter("topic_imu").as_string();
        scalingImu_ = this->get_parameter("scaling_imu").as_double();
        const std::string topicLidar = this->get_parameter("topic_lidar").as_string();
        const bool lidarMsgPc2 = this->get_parameter("lidar_msg_pc2").as_bool();
        RCLCPP_INFO(this->get_logger(), "Using mapper config file: %s", configPath.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s (scale %.4f)", topicImu.c_str(), scalingImu_);
        RCLCPP_INFO(this->get_logger(), "LiDAR topic: %s (%s)", topicLidar.c_str(),
                    lidarMsgPc2 ? "sensor_msgs/PointCloud2" : "livox_ros_driver2/CustomMsg");
        mapping::MappingConfig config = loadConfig(configPath);
        slam_.setConfig(config);
        RCLCPP_INFO(this->get_logger(), "bundle adjustment: %s",
                    config.bundle_adjustment.enabled ? "enabled" : "disabled");

        // use callback groups so to keep feeding IMU while keyframe updates are running
        callbackGroupImu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupCamera_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imuSubOpt = rclcpp::SubscriptionOptions(),
                                    lidarSubOpt = rclcpp::SubscriptionOptions(),
                                    cameraSubOpt = rclcpp::SubscriptionOptions();
        imuSubOpt.callback_group = callbackGroupImu_;
        lidarSubOpt.callback_group = callbackGroupLidar_;
        cameraSubOpt.callback_group = callbackGroupCamera_;

        // --- subscribers ---
        subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            // 2 seconds assuming 500 [Hz] IMU
            topicImu, 1000, std::bind(&MapperNode::imuCallback, this, std::placeholders::_1), imuSubOpt);
        if (lidarMsgPc2)
        {
            subLidarPc2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                // 2 seconds assuming 10 [Hz] LiDAR
                topicLidar, 20, std::bind(&MapperNode::lidarPc2Callback, this, std::placeholders::_1), lidarSubOpt);
        }
        else
        {
            subLidar_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                // 2 seconds assuming 10 [Hz] LiDAR
                topicLidar, 20, std::bind(&MapperNode::lidarCallback, this, std::placeholders::_1), lidarSubOpt);
        }
        subCamera_ = this->create_subscription<sensor_msgs::msg::Image>(
            // 2 seconds assuming 20 FPS
            "/camera/image_raw", 40, std::bind(&MapperNode::cameraCallback, this, std::placeholders::_1), cameraSubOpt);
        // --- publishers ----
        pubSlidingWindowPath_ = this->create_publisher<nav_msgs::msg::Path>("mapping/window", 10);
        pubHistoricalPosesPath_ = this->create_publisher<nav_msgs::msg::Path>("mapping/trajectory", 10);
        pubKeyframeSubmap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/keyframe_submap", 10);
        pubGlobalMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/map", 10);
        pubClusters_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mapping/clusters", 10);

        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!hasStartTime_)
        {
            startTime_ = rclcpp::Time(msg->header.stamp);
            hasStartTime_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - startTime_).seconds();
        slam_.feedImu(imuMsgToImuData(*msg, scalingImu_), timestamp);
    }

    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if (!hasStartTime_)
        {
            startTime_ = rclcpp::Time(msg->header.stamp);
            hasStartTime_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - startTime_).seconds();
        slam_.feedLidar(livoxMsgToLidarData(*msg), timestamp);
        slam_.update();
        publishStates();
    }

    void lidarPc2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!hasStartTime_)
        {
            startTime_ = rclcpp::Time(msg->header.stamp);
            hasStartTime_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - startTime_).seconds();
        try
        {
            const auto lidarData = pointCloud2ToLidarData(*msg);
            if (!lidarData)
                return;
            slam_.feedLidar(lidarData, timestamp);
            slam_.update();
            publishStates();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "LiDAR PointCloud2 callback failed: %s", e.what());
        }
    }

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!hasStartTime_) /// NOTE: camere is auxiliary (skips until system starts)
            return;

        double timestamp = (rclcpp::Time(msg->header.stamp) - startTime_).seconds();

        cv_bridge::CvImageConstPtr cvPtr;
        try
        {
            cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        auto cam_data = std::make_shared<mapping::CameraData>();
        cam_data->img = cvPtr->image;
        cam_data->colorSpace = mapping::CameraColorSpace::RGB;
        // feed to SLAM system
        slam_.feedCamera(cam_data, timestamp);
    }

private:
    void publishStates()
    {
        auto states = slam_.getStates();
        if (states.empty())
            return;

        // current pose TF
        mapping::NavStateStamped latestState = states.rbegin()->second;
        geometry_msgs::msg::TransformStamped tfStampedMsg;
        const rclcpp::Time stamp{startTime_ + rclcpp::Duration::from_seconds(latestState.timestamp)};
        tfStampedMsg.header.stamp = stamp;
        tfStampedMsg.header.frame_id = "map";
        tfStampedMsg.child_frame_id = "base_link";

        const gtsam::Pose3 &pose = latestState.state.pose();
        tfStampedMsg.transform.translation.x = pose.translation().x();
        tfStampedMsg.transform.translation.y = pose.translation().y();
        tfStampedMsg.transform.translation.z = pose.translation().z();

        const gtsam::Rot3 &rot = pose.rotation();
        gtsam::Quaternion q = rot.toQuaternion();
        tfStampedMsg.transform.rotation.x = q.x();
        tfStampedMsg.transform.rotation.y = q.y();
        tfStampedMsg.transform.rotation.z = q.z();
        tfStampedMsg.transform.rotation.w = q.w();
        tfBroadcaster_->sendTransform(tfStampedMsg);

        // publish the sliding window before the keyframe gate so it tracks at the update() rate
        // and carries the latest predicted pose, not just newly created keyframes
        // TODO: publish full state as nav_msgs/msg/Odometry
        nav_msgs::msg::Path slidingWindowPathMsg;
        slidingWindowPathMsg.header.stamp = this->now();
        slidingWindowPathMsg.header.frame_id = "map";
        for (const auto &[idxKf, navStateStamped] : states)
        {
            geometry_msgs::msg::PoseStamped poseStampedMsg;
            const rclcpp::Time poseStamp{startTime_ + rclcpp::Duration::from_seconds(navStateStamped.timestamp)};
            poseStampedMsg.header.stamp = poseStamp;
            poseStampedMsg.header.frame_id = "map";

            const gtsam::Pose3 &pose = navStateStamped.state.pose();
            poseStampedMsg.pose.position.x = pose.translation().x();
            poseStampedMsg.pose.position.y = pose.translation().y();
            poseStampedMsg.pose.position.z = pose.translation().z();

            const gtsam::Rot3 &rot = pose.rotation();
            gtsam::Quaternion q = rot.toQuaternion();
            poseStampedMsg.pose.orientation.x = q.x();
            poseStampedMsg.pose.orientation.y = q.y();
            poseStampedMsg.pose.orientation.z = q.z();
            poseStampedMsg.pose.orientation.w = q.w();
            slidingWindowPathMsg.poses.push_back(poseStampedMsg);

            historicalPoses[idxKf] = poseStampedMsg;
        }
        pubSlidingWindowPath_->publish(slidingWindowPathMsg);

        // skip expensive visualization if no new keyframe was created
        const uint32_t currentKeyframeCount = slam_.getKeyframeCount();
        if (currentKeyframeCount == lastPublishedKeyframeCount_)
            return;
        lastPublishedKeyframeCount_ = currentKeyframeCount;

        // current keyframe submap
        std::shared_ptr<open3d::geometry::PointCloud> pcdSubmap = slam_.getCurrentSubmap();
        if (pcdSubmap)
        {
            sensor_msgs::msg::PointCloud2 submapMsg;
            // NOTE: keyframe submaps are placed in the global reference frame ("map")
            open3d_conversions::open3dToRos(*pcdSubmap, submapMsg, "map");
            submapMsg.header.stamp = stamp;
            pubKeyframeSubmap_->publish(submapMsg);
        }

        // build global map from the full BA state: frozen (corrected) + active (pending optimization)
        {
            open3d::geometry::PointCloud pcdGlobalMap;

            for (const auto &sub : slam_.getAllFrozenSubmaps())
                if (sub->pcdWorld)
                    pcdGlobalMap += *sub->pcdWorld;

            for (const auto &[_, submap] : slam_.getAllActiveSubmaps())
                if (submap.pcd)
                {
                    open3d::geometry::PointCloud pcdWorld = *(submap.pcd);
                    pcdWorld.Transform(submap.pose.matrix());
                    pcdGlobalMap += pcdWorld;
                }

            if (!pcdGlobalMap.IsEmpty())
            {
                auto pcdDownsampled = pcdGlobalMap.VoxelDownSample(0.05);
                sensor_msgs::msg::PointCloud2 globalMapMsg;
                open3d_conversions::open3dToRos(*pcdDownsampled, globalMapMsg, "map");
                globalMapMsg.header.stamp = stamp;
                pubGlobalMap_->publish(globalMapMsg);
            }
        }

        std::map<mapping::ClusterId, mapping::PointCluster> clusters = slam_.getCurrentClusters();

        // if markers have been published before, check which need to be deleted
        std::vector<visualization_msgs::msg::Marker> previousMarkers = clusterMarkersMsg_.markers;
        clusterMarkersMsg_.markers.clear();
        clusterMarkersMsg_.markers.reserve(clusters.size());
        for (auto &marker : previousMarkers)
        {
            if (clusters.find(marker.id) == clusters.end())
            {
                // cluster no longer exists, delete the marker
                marker.action = visualization_msgs::msg::Marker::DELETE;
                clusterMarkersMsg_.markers.push_back(marker);
            }
        }

        for (const auto &[clusterId, cluster] : clusters)
        {
            visualization_msgs::msg::Marker markerMsg;
            markerMsg.header.stamp = stamp;
            markerMsg.header.frame_id = "map";
            markerMsg.ns = "clusters";
            markerMsg.id = clusterId;
            markerMsg.type = visualization_msgs::msg::Marker::ARROW;
            markerMsg.action = visualization_msgs::msg::Marker::ADD;
            // Arrow points from cluster center along normal direction
            geometry_msgs::msg::Point startPoint, endPoint;
            startPoint.x = cluster.center->x();
            startPoint.y = cluster.center->y();
            startPoint.z = cluster.center->z();
            endPoint.x = startPoint.x + cluster.normal->x() * 1.0; // scale for visualization
            endPoint.y = startPoint.y + cluster.normal->y() * 1.0;
            endPoint.z = startPoint.z + cluster.normal->z() * 1.0;
            // NOTE: direction of the plane normal arrow can be given from markerMsg.points, see also:
            // https://ros2docs.robook.org/rolling/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html#message-parameters
            markerMsg.points.reserve(2);
            markerMsg.points.push_back(startPoint);
            markerMsg.points.push_back(endPoint);
            markerMsg.scale.x = 0.1; // shaft diameter
            markerMsg.scale.y = 0.2; // head diameter
            markerMsg.scale.z = 0.2; // head length
            markerMsg.color.r = 1.0f;
            markerMsg.color.g = 0.0f;
            markerMsg.color.b = 0.0f;
            markerMsg.color.a = 1.0f;
            clusterMarkersMsg_.markers.push_back(markerMsg);
        }
        pubClusters_->publish(clusterMarkersMsg_);

        // global trajectory of all keyframe poses accumulated so far
        nav_msgs::msg::Path historicalPosesPathMsg;
        historicalPosesPathMsg.header.stamp = this->now();
        historicalPosesPathMsg.header.frame_id = "map";
        historicalPosesPathMsg.poses.reserve(historicalPoses.size());
        for (auto const &[_, histPose] : historicalPoses)
        {
            historicalPosesPathMsg.poses.push_back(histPose);
        }
        pubHistoricalPosesPath_->publish(historicalPosesPathMsg);
    }

private:
    /// @brief asynchronous, mutually exclusive, ROS2 callback-groups for the different sensors
    /// this allows to process high-rate incoming data without halting the system.
    /// @note Some of the data might be discarded due to the async nature of the process!
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu_, callbackGroupLidar_, callbackGroupCamera_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLidar_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidarPc2_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamera_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubSlidingWindowPath_, pubHistoricalPosesPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyframeSubmap_, pubGlobalMap_;
    // visualize the tracking factors as markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubClusters_;
    visualization_msgs::msg::MarkerArray clusterMarkersMsg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    std::map<uint32_t, geometry_msgs::msg::PoseStamped> historicalPoses;
    rclcpp::Time startTime_;
    // accessed from both IMU and LiDAR callback threads without a lock
    // TODO: in the future, use a mutex for values like this
    std::atomic<bool> hasStartTime_ = false;
    /// @brief Scaling applied to raw accelerometer readings (Livox reports in `[g]`, so default is 9.81).
    double scalingImu_ = 9.81;

    mapping::MappingSystem slam_;
    uint32_t lastPublishedKeyframeCount_ = 0;
};

/// @brief Options for the standalone bag reader mode.
struct BagOptions
{
    std::string bagPath;
    std::string topicImu = "/livox/imu";
    std::string topicLidar = "/livox/lidar";
    /// @brief Scaling applied to raw accelerometer readings (Livox reports in `[g]`, so default is 9.81).
    double scalingImu = 9.81;
    /// @brief If set, write trajectory, config copy and run metadata into this directory (benchmarking builds only).
    std::string outputDir;
    /// @brief Config path as given on the command line, recorded in the run metadata.
    std::string configPath;
};

/// @brief Strip a leading slash, so that e.g. `livox/lidar` and `/livox/lidar` match the same bag topic.
std::string normalizeTopicName(const std::string &topic)
{
    return (!topic.empty() && topic.front() == '/') ? topic.substr(1) : topic;
}

#ifdef MAPPING_BENCHMARKING
/// @brief Benchmark output collected during a bag run.
struct BagRecorder
{
    mapping::TrajectoryRecorder trajectory;
    mapping::benchmarking::TelemetryRecorder telemetry;
};
#else
struct BagRecorder
{
};
#endif

/// @brief Counters and timing of a bag run, filled by `processBag`.
struct BagStats
{
    size_t numImuMsgs = 0, numLidarMsgs = 0, numSkippedLidarMsgs = 0, numKeyframes = 0;
    double bagStart = 0.0, wallSeconds = 0.0, realtimeFactor = 0.0;
};

// Standalone bag reader for debugging and benchmarking
// recorder is unused without MAPPING_BENCHMARKING, [[maybe_unused]] suppresses the unused-parameter warning
int processBag(const BagOptions &options, const mapping::MappingConfig &config, [[maybe_unused]] BagRecorder &recorder,
               BagStats &stats)
{
    // Setup bag reader
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = options.bagPath;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    std::vector<rosbag2_storage::TopicMetadata> topicsAndTypes;
    try
    {
        reader.open(storage_options, converter_options);
        topicsAndTypes = reader.get_all_topics_and_types();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: failed to open bag '" << options.bagPath << "': " << e.what() << std::endl;
        return 1;
    }

    // resolve the requested topics against the bag metadata, tolerating a missing leading slash
    auto findTopic = [&topicsAndTypes](const std::string &topic) -> std::optional<rosbag2_storage::TopicMetadata>
    {
        for (const auto &meta : topicsAndTypes)
            if (normalizeTopicName(meta.name) == normalizeTopicName(topic))
                return meta;
        return std::nullopt;
    };
    auto printAvailableTopics = [&topicsAndTypes]()
    {
        std::cerr << "Available topics in the bag:" << std::endl;
        for (const auto &meta : topicsAndTypes)
            std::cerr << "  " << meta.name << " [" << meta.type << "]" << std::endl;
    };

    const std::optional<rosbag2_storage::TopicMetadata> metaImu = findTopic(options.topicImu),
                                                        metaLidar = findTopic(options.topicLidar);
    if (!metaImu || !metaLidar)
    {
        if (!metaImu)
            std::cerr << "Error: IMU topic '" << options.topicImu << "' not found in the bag" << std::endl;
        if (!metaLidar)
            std::cerr << "Error: LiDAR topic '" << options.topicLidar << "' not found in the bag" << std::endl;
        printAvailableTopics();
        return 1;
    }
    if (metaImu->type != "sensor_msgs/msg/Imu")
    {
        std::cerr << "Error: IMU topic '" << metaImu->name << "' has type '" << metaImu->type
                  << "', expected 'sensor_msgs/msg/Imu'" << std::endl;
        printAvailableTopics();
        return 1;
    }
    // detect the LiDAR message type from the bag metadata
    bool lidarMsgPc2 = false;
    if (metaLidar->type == "sensor_msgs/msg/PointCloud2")
        lidarMsgPc2 = true;
    else if (metaLidar->type != "livox_ros_driver2/msg/CustomMsg")
    {
        std::cerr << "Error: LiDAR topic '" << metaLidar->name << "' has unsupported type '" << metaLidar->type
                  << "', expected 'livox_ros_driver2/msg/CustomMsg' or 'sensor_msgs/msg/PointCloud2'" << std::endl;
        printAvailableTopics();
        return 1;
    }

    // only read the two topics we need from storage (skips e.g. camera images)
    rosbag2_storage::StorageFilter storageFilter;
    storageFilter.topics = {metaImu->name, metaLidar->name};
    reader.set_filter(storageFilter);

    std::cout << "Reading bag file: " << options.bagPath << std::endl;
    std::cout << "IMU topic: " << metaImu->name << " (scale " << options.scalingImu << ")" << std::endl;
    std::cout << "LiDAR topic: " << metaLidar->name << " (" << metaLidar->type << ")" << std::endl;

    mapping::MappingSystem slam(config);
    std::cout << "bundle adjustment: " << (config.bundle_adjustment.enabled ? "enabled" : "disabled") << std::endl;

    rclcpp::Serialization<sensor_msgs::msg::Imu> serializationImu;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializationLivox;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationPc2;
    sensor_msgs::msg::Imu msgImu;
    livox_ros_driver2::msg::CustomMsg msgLivox;
    sensor_msgs::msg::PointCloud2 msgPc2;

    // timestamps fed to the SLAM system are relative to the first message
    rclcpp::Time start_time;
    bool has_start_time = false;
    auto toRelativeTime = [&start_time, &has_start_time](const builtin_interfaces::msg::Time &stamp)
    {
        if (!has_start_time)
        {
            start_time = rclcpp::Time(stamp);
            has_start_time = true;
        }
        return (rclcpp::Time(stamp) - start_time).seconds();
    };

    // the smoother only optimizes at keyframes, so the trajectory is recorded once per new keyframe
    uint32_t numRecordedKeyframes = 0;
    auto recordTrajectory = [&]()
    {
#ifdef MAPPING_BENCHMARKING
        numRecordedKeyframes = slam.getKeyframeCount();
        recorder.trajectory.update(slam.getStates(), numRecordedKeyframes);
#endif
    };
    // drain the keyframe telemetry queue, so no row is lost (also a lost-tracking keyframe, which has no new pose)
    auto recordTelemetry = [&]()
    {
#ifdef MAPPING_BENCHMARKING
        recorder.telemetry.append(slam.takeTelemetry());
#endif
    };
    const bool recordEnabled = !options.outputDir.empty();

    size_t numImuMsgs = 0, numLidarMsgs = 0, numSkippedLidarMsgs = 0;
    double tLastMsg = 0.0;
    const auto tWallStart = std::chrono::steady_clock::now();
    bool failed = false;

    try
    {
        while (reader.has_next() && !g_shutdown_requested)
        {
            auto bag_message = reader.read_next();
            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

            if (bag_message->topic_name == metaImu->name)
            {
                serializationImu.deserialize_message(&serialized_msg, &msgImu);
                tLastMsg = toRelativeTime(msgImu.header.stamp);
                slam.feedImu(imuMsgToImuData(msgImu, options.scalingImu), tLastMsg);
                ++numImuMsgs;
            }
            else if (bag_message->topic_name == metaLidar->name)
            {
                std::shared_ptr<mapping::LidarData> lidarData;
                if (lidarMsgPc2)
                {
                    serializationPc2.deserialize_message(&serialized_msg, &msgPc2);
                    tLastMsg = toRelativeTime(msgPc2.header.stamp);
                    lidarData = pointCloud2ToLidarData(msgPc2);
                }
                else
                {
                    serializationLivox.deserialize_message(&serialized_msg, &msgLivox);
                    tLastMsg = toRelativeTime(msgLivox.header.stamp);
                    lidarData = livoxMsgToLidarData(msgLivox);
                }
                if (!lidarData)
                {
                    ++numSkippedLidarMsgs;
                    continue;
                }
                slam.feedLidar(lidarData, tLastMsg);
                slam.update();
                ++numLidarMsgs;
                if (recordEnabled)
                    recordTelemetry();
                if (recordEnabled && slam.getKeyframeCount() != numRecordedKeyframes)
                    recordTrajectory();
            }
        }
        if (recordEnabled)
            recordTrajectory();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: failed while processing the bag: " << e.what() << std::endl;
        failed = true;
    }
    // also after an error: the probe of an aborted keyframe still queues its row
    if (recordEnabled)
        recordTelemetry();

    const double wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - tWallStart).count();
    std::cout << (failed                 ? "Aborted"
                  : g_shutdown_requested ? "Interrupted"
                                         : "Finished")
              << " processing bag file: " << numImuMsgs << " IMU msgs, " << numLidarMsgs << " LiDAR msgs ("
              << numSkippedLidarMsgs << " skipped), " << slam.getKeyframeCount() << " keyframes, " << tLastMsg
              << " s of data in " << wallSeconds << " s wall-clock ("
              << (wallSeconds > 0.0 ? tLastMsg / wallSeconds : 0.0) << "x real-time)" << std::endl;
    stats = {numImuMsgs,
             numLidarMsgs,
             numSkippedLidarMsgs,
             slam.getKeyframeCount(),
             has_start_time ? start_time.seconds() : 0.0,
             wallSeconds,
             wallSeconds > 0.0 ? tLastMsg / wallSeconds : 0.0};
    if (failed)
        return 1;
    if (g_shutdown_requested)
        return 130;
    if (numLidarMsgs == 0)
    {
        std::cerr << "Error: no LiDAR messages were processed from topic '" << metaLidar->name << "'" << std::endl;
        return 2;
    }
    return 0;
}

int runFromBag(const BagOptions &options, const mapping::MappingConfig &config)
{
    BagRecorder recorder;
    BagStats stats;
#ifdef MAPPING_BENCHMARKING
    const std::string wallStart = mapping::wallClockNowISO();
#endif
    int exitCode = processBag(options, config, recorder, stats);
#ifdef MAPPING_BENCHMARKING
    // also written after an error or Ctrl+C, the partial trajectory is still useful for evaluation
    if (!options.outputDir.empty())
    {
        mapping::RunInfo info;
        info.bagPath = options.bagPath;
        info.topicImu = options.topicImu;
        info.topicLidar = options.topicLidar;
        info.scalingImu = options.scalingImu;
        info.configPath = options.configPath;
        info.bagStart = stats.bagStart;
        info.wallStart = wallStart;
        info.wallDurationS = stats.wallSeconds;
        info.realtimeFactor = stats.realtimeFactor;
        info.exitCode = exitCode;
        info.msgsImu = stats.numImuMsgs;
        info.msgsLidar = stats.numLidarMsgs;
        info.scansSkipped = stats.numSkippedLidarMsgs;
        info.keyframes = stats.numKeyframes;
        std::string error;
        if (mapping::writeRunOutput(options.outputDir, recorder.trajectory, recorder.telemetry, config, info, error))
            std::cout << "Wrote " << recorder.trajectory.size() << " keyframe poses, " << recorder.telemetry.size()
                      << " telemetry rows and run metadata to " << options.outputDir << std::endl;
        else
        {
            std::cerr << "Error: " << error << std::endl;
            exitCode = 1;
        }
    }
#endif
    return exitCode;
}

void printUsage(const char *argv0)
{
    std::cerr << "Usage:\n"
              << "  ROS 2 node:  " << argv0 << " [--ros-args ...]\n"
              << "  bag reader:  " << argv0 << " --bag <bag_path> --config <config.yaml> [options]\n"
              << "\n"
              << "Bag reader arguments:\n"
              << "  --bag <path>            ROS 2 bag directory (sqlite3)\n"
              << "  --config <path>         mapping config YAML (required with --bag)\n"
              << "  --topic-imu <topic>     sensor_msgs/msg/Imu topic (default: /livox/imu)\n"
              << "  --topic-lidar <topic>   livox_ros_driver2/msg/CustomMsg or sensor_msgs/msg/PointCloud2 topic,\n"
              << "                          the type is detected from the bag (default: /livox/lidar)\n"
              << "  --scaling-imu <factor>  scale applied to accelerometer readings,\n"
              << "                          9.81 for [g] (Livox), 1.0 for [m/s^2] (default: 9.81)\n"
              << "  --output-dir <dir>      write trajectory.tum (IMU frame, absolute bag timestamps, for evo),\n"
              << "                          telemetry.csv (one row per keyframe), config.yaml\n"
              << "                          (loaded config) and run.yaml into <dir>.\n"
              << "                          Fails if <dir> is not empty.\n"
              << "                          Needs a build with -DMAPPING_BENCHMARKING=ON (default: off)\n"
              << "  -h, --help              show this message" << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Parse arguments for standalone bag mode
    std::string config_path;
    BagOptions bagOptions;
    bool use_bag = false, bagOnlyOptionGiven = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            printUsage(argv[0]);
            rclcpp::shutdown();
            return 0;
        }
        const bool takesValue = arg == "--config" || arg == "--bag" || arg == "--topic-imu" || arg == "--topic-lidar" ||
                                arg == "--scaling-imu" || arg == "--output-dir";
        if (!takesValue)
            continue; // e.g. ROS arguments
        if (i + 1 >= argc)
        {
            std::cerr << "Error: missing value for " << arg << std::endl;
            printUsage(argv[0]);
            rclcpp::shutdown();
            return 1;
        }
        const std::string value = argv[++i];
        if (value.rfind("--", 0) == 0)
        {
            std::cerr << "Error: missing value for " << arg << " (got '" << value << "')" << std::endl;
            printUsage(argv[0]);
            rclcpp::shutdown();
            return 1;
        }
        if (arg == "--topic-imu" || arg == "--topic-lidar" || arg == "--scaling-imu" || arg == "--output-dir")
            bagOnlyOptionGiven = true;
        if (arg == "--config")
            config_path = bagOptions.configPath = value;
        else if (arg == "--bag")
        {
            bagOptions.bagPath = value;
            use_bag = true;
        }
        else if (arg == "--topic-imu")
            bagOptions.topicImu = value;
        else if (arg == "--topic-lidar")
            bagOptions.topicLidar = value;
        else if (arg == "--output-dir")
            bagOptions.outputDir = value;
        else if (arg == "--scaling-imu")
        {
            try
            {
                size_t numParsed = 0;
                bagOptions.scalingImu = std::stod(value, &numParsed);
                if (numParsed != value.size() || !std::isfinite(bagOptions.scalingImu) || bagOptions.scalingImu <= 0.0)
                    throw std::invalid_argument("not a finite positive number");
            }
            catch (const std::exception &)
            {
                std::cerr << "Error: invalid value for --scaling-imu: '" << value << "'" << std::endl;
                printUsage(argv[0]);
                rclcpp::shutdown();
                return 1;
            }
        }
    }

    if (use_bag && config_path.empty())
    {
        std::cerr << "Error: --config <path> is required" << std::endl;
        printUsage(argv[0]);
        rclcpp::shutdown();
        return 1;
    }

    if (!use_bag && bagOnlyOptionGiven)
        std::cerr << "Warning: --topic-imu, --topic-lidar, --scaling-imu and --output-dir only apply to --bag "
                     "mode, the ROS node uses the ROS parameters topic_imu, topic_lidar and scaling_imu"
                  << std::endl;

    int exitCode = 0;
    // Check if running in standalone bag mode
    if (use_bag)
    {
        std::cout << "Running in standalone bag reader mode" << std::endl;
        // rclcpp::init installed its own SIGINT handler, replace it so Ctrl+C stops the bag loop gracefully
        // NOTE: in node mode rclcpp's handler is kept, it shuts down the executor on Ctrl+C
        std::signal(SIGINT, signalHandler);
#ifdef MAPPING_BENCHMARKING
        if (!bagOptions.outputDir.empty())
        {
            std::string error;
            if (!mapping::prepareRunDir(bagOptions.outputDir, error))
            {
                std::cerr << "Error: " << error << std::endl;
                rclcpp::shutdown();
                return 1;
            }
        }
#else
        if (!bagOptions.outputDir.empty())
        {
            std::cerr << "Error: --output-dir requires a build with -DMAPPING_BENCHMARKING=ON" << std::endl;
            rclcpp::shutdown();
            return 1;
        }
#endif
        mapping::MappingConfig config = loadConfig(config_path);
        exitCode = runFromBag(bagOptions, config);
    }
    else
    {
        std::cout << "Running as ROS 2 node" << std::endl;
        auto node = std::make_shared<MapperNode>();
        // MultiThreadedExecutor allows parallel processing of the callback groups
        // NOTE: IMU callbacks are just feeding data, those can run in parallel with the LiDAR callback group
        // the LiDAR callback group feeds data AND runs the expepnsive SLAM update
        // using just rclcpp::spin(node) would lead to IMU messages being capped and the preintegration diverging
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }

    rclcpp::shutdown();
    return exitCode;
}
