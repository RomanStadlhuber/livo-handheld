// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

// Mapping
#include <mapping/MappingSystem.hpp>

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

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum)
{
    (void)signum;
    g_shutdown_requested = true;
    rclcpp::shutdown();
}

mapping::MappingConfig loadConfig(const std::string &config_path)
{
    std::cout << "Loading config from: " << config_path << std::endl;
    mapping::MappingConfig config = config::fromYamlFile<mapping::MappingConfig>(config_path);
    config::checkValid(config);
    std::cout << "Loaded config:\n"
              << config::toString(config) << std::endl;
    return config;
}

class MapperNode : public rclcpp::Node
{
public:
    explicit MapperNode()
        : Node("mapper_node"),
          slam_()
    {

        RCLCPP_INFO(this->get_logger(), "MapperNode has been initialized.");
        this->declare_parameter<std::string>("mapper_config", "");
        std::string configPath = this->get_parameter("mapper_config").as_string();
        RCLCPP_INFO(this->get_logger(), "Using mapper config file: %s",
                    configPath.c_str());
        mapping::MappingConfig config = loadConfig(configPath);
        slam_.setConfig(config);
        // enable collecting marginalized submaps to accumulate the global map
        slam_.setCollectMarginalizedSubmaps(true);

        // use callback groups so to keep feeding IMU while keyframe updates are running
        callbackGroupImu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuSubOpt = rclcpp::SubscriptionOptions();
        imuSubOpt.callback_group = callbackGroupImu_;
        auto lidarSubOpt = rclcpp::SubscriptionOptions();
        lidarSubOpt.callback_group = callbackGroupLidar_;

        subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            // 2 seconds assuming 500 [Hz] IMU
            "/livox/imu", 1000, std::bind(&MapperNode::imuCallback, this, std::placeholders::_1), imuSubOpt);
        subLidar_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            // 2 seconds assuming 10 [Hz] LiDAR
            "livox/lidar", 20, std::bind(&MapperNode::lidarCallback, this, std::placeholders::_1), lidarSubOpt);
        pubSlidingWindowPath_ = this->create_publisher<nav_msgs::msg::Path>(
            "mapping/window", 10);
        pubHistoricalPosesPath_ = this->create_publisher<nav_msgs::msg::Path>(
            "mapping/trajectory", 10);
        pubKeyframeSubmap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "mapping/keyframe_submap", 10);
        pubGlobalMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "mapping/map", 10);
        globalMap_ = std::make_shared<open3d::geometry::PointCloud>();
        pubClusters_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "mapping/clusters", 10);

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

        // Build imu data container from msg
        auto imu_data = std::make_shared<mapping::ImuData>();
        imu_data->acceleration = Eigen::Vector3d(
                                     msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z) *
                                 kLivoxImuScale;
        imu_data->angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        slam_.feedImu(imu_data, timestamp);
    }

    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if (!hasStartTime_)
        {
            startTime_ = rclcpp::Time(msg->header.stamp);
            hasStartTime_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - startTime_).seconds();

        // Build lidar data container from msg
        auto lidar_data = std::make_shared<mapping::LidarData>();
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
            // Point offset time is given in nanoseconds
            rclcpp::Time point_offset_time(static_cast<uint64_t>(point.offset_time));
            lidar_data->offset_times.push_back(point_offset_time.seconds());
        }
        slam_.feedLidar(lidar_data, timestamp);
        slam_.update();
        publishStates();
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

        // build global map from marginalized keyframe submaps (most optimized poses)
        auto marginalizedSubmaps = slam_.getMarginalizedSubmaps();
        if (!marginalizedSubmaps.empty())
        {
            for (const auto &submap : marginalizedSubmaps)
                *globalMap_ += *submap;
            globalMap_ = globalMap_->VoxelDownSample(0.05);
            sensor_msgs::msg::PointCloud2 globalMapMsg;
            open3d_conversions::open3dToRos(*globalMap_, globalMapMsg, "map");
            globalMapMsg.header.stamp = stamp;
            pubGlobalMap_->publish(globalMapMsg);
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

        // sliding window and global trajectory
        nav_msgs::msg::Path slidingWindowPathMsg, historicalPosesPathMsg;
        slidingWindowPathMsg.header.stamp = this->now();
        slidingWindowPathMsg.header.frame_id = "map";
        historicalPosesPathMsg.header = slidingWindowPathMsg.header;
        historicalPosesPathMsg.header.frame_id = "map";
        historicalPosesPathMsg.poses.reserve(historicalPoses.size());
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
        for (auto const &[_, histPose] : historicalPoses)
        {
            historicalPosesPathMsg.poses.push_back(histPose);
        }
        pubSlidingWindowPath_->publish(slidingWindowPathMsg);
        pubHistoricalPosesPath_->publish(historicalPosesPathMsg);
    }

private:
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu_;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLidar_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubSlidingWindowPath_, pubHistoricalPosesPath_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyframeSubmap_, pubGlobalMap_;
    // visualize the tracking factors as markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubClusters_;
    visualization_msgs::msg::MarkerArray clusterMarkersMsg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    std::map<uint32_t, geometry_msgs::msg::PoseStamped> historicalPoses;
    /// @brief Naive accumulation of keyframe submaps to form the global map.
    std::shared_ptr<open3d::geometry::PointCloud> globalMap_;
    rclcpp::Time startTime_;
    bool hasStartTime_ = false;
    static constexpr double kLivoxImuScale = 9.81;

    mapping::MappingSystem slam_;
    uint32_t lastPublishedKeyframeCount_ = 0;
};

// Standalone bag reader for debugging
void runFromBag(const std::string &bag_path, const mapping::MappingConfig &config)
{
    mapping::MappingSystem slam(config);

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
    constexpr double livox_imu_scale = 9.81;

    std::cout << "Reading bag file: " << bag_path << std::endl;

    while (reader.has_next() && !g_shutdown_requested)
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
            auto imu_data = std::make_shared<mapping::ImuData>();
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
            auto lidar_data = std::make_shared<mapping::LidarData>();
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

    // Register signal handler after rclcpp::init, which installs its own
    std::signal(SIGINT, signalHandler);

    // Parse arguments for config file
    std::string config_path;
    std::string bag_path;
    bool use_bag = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc)
        {
            config_path = argv[++i];
        }
        else if (arg == "--bag" && i + 1 < argc)
        {
            bag_path = argv[++i];
            use_bag = true;
        }
    }

    if (use_bag && config_path.empty())
    {
        std::cerr << "Error: --config <path> is required" << std::endl;
        std::cerr << "Usage: " << argv[0] << "--bag <bag_path> --config <config.yaml>" << std::endl;
        return 1;
    }

    // Check if running in standalone bag mode
    if (use_bag)
    {
        std::cout << "Running in standalone bag reader mode" << std::endl;
        mapping::MappingConfig config = loadConfig(config_path);
        runFromBag(bag_path, config);
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
    return 0;
}
