// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

// Mapping
#include <mapping/MappingSystem.hpp>

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
}

class MapperNode : public rclcpp::Node
{
public:
    explicit MapperNode(const mapping::MappingConfig &config)
        : Node("mapper_node"),
          slam_(config)
    {
        RCLCPP_INFO(this->get_logger(), "MapperNode has been initialized.");

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&MapperNode::imuCallback, this, std::placeholders::_1));

        lidar_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "livox/lidar", 10, std::bind(&MapperNode::lidarCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!has_start_time_)
        {
            start_time_ = rclcpp::Time(msg->header.stamp);
            has_start_time_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - start_time_).seconds();

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
        if (!has_start_time_)
        {
            start_time_ = rclcpp::Time(msg->header.stamp);
            has_start_time_ = true;
        }
        double timestamp = (rclcpp::Time(msg->header.stamp) - start_time_).seconds();

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
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription_;
    rclcpp::Time start_time_;
    bool has_start_time_ = false;
    static constexpr double kLivoxImuScale = 9.81;

    mapping::MappingSystem slam_;
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

mapping::MappingConfig loadConfig(const std::string &config_path)
{
    std::cout << "Loading config from: " << config_path << std::endl;
    mapping::MappingConfig config = config::fromYamlFile<mapping::MappingConfig>(config_path);
    config::checkValid(config);
    std::cout << "Loaded config:\n"
              << config::toString(config) << std::endl;
    return config;
}

int main(int argc, char **argv)
{
    // Register signal handler for CTRL+C
    std::signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);

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

    if (config_path.empty())
    {
        std::cerr << "Error: --config <path> is required" << std::endl;
        std::cerr << "Usage: " << argv[0] << " --config <config.yaml> [--bag <bag_path>]" << std::endl;
        return 1;
    }

    // Load configuration
    mapping::MappingConfig config = loadConfig(config_path);

    // Check if running in standalone bag mode
    if (use_bag)
    {
        std::cout << "Running in standalone bag reader mode" << std::endl;
        runFromBag(bag_path, config);
    }
    else
    {
        std::cout << "Running as ROS 2 node" << std::endl;
        rclcpp::spin(std::make_shared<MapperNode>(config));
    }

    rclcpp::shutdown();
    return 0;
}
