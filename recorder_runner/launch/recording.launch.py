from ament_index_python.resources import has_resource

from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from pathlib import Path
import os

################### user configure parameters for ros2 start ###################
xfer_format = 1  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0  # 0-lidar, others-Invalid data src
publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = "livox_frame"
lvx_file_path = "/home/livox/livox_test.lvx"
cmdline_bd_code = "livox0000000001"

cur_path = os.path.split(os.path.realpath(__file__))[0] + "/"
cur_path = Path(cur_path)
user_config_path = cur_path / "../launch/MID360_config.json"
user_config_path = str(user_config_path)

################### user configure parameters for ros2 end #####################

def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description with for the camera node and a visualiser.

    Returns
    -------
        LaunchDescription: the launch description

    """
    # parameters
    camera_param_name = "camera"
    camera_param_default = str(0)
    camera_param = LaunchConfiguration(
        camera_param_name,
        default=camera_param_default,
    )
    camera_launch_arg = DeclareLaunchArgument(
        camera_param_name, default_value=camera_param_default, description="camera ID or name"
    )

    format_param_name = "format"
    format_param_default = "RGB888"  # str()
    format_param = LaunchConfiguration(
        format_param_name,
        default=format_param_default,
    )
    format_launch_arg = DeclareLaunchArgument(
        format_param_name, default_value=format_param_default, description="pixel format"
    )
    # wheter to also run image viewer
    run_imview = False
    """imview_launch_arg = DeclareLaunchArgument(
        run_imview,
        default_value=False
    )"""

    # camera node
    composable_nodes = [
        ComposableNode(
            package="camera_ros",
            plugin="camera::CameraNode",
            parameters=[
                {
                    "camera": camera_param,
                    "width": 640,  # 1456,
                    "height": 480,  # 1088,
                    "format": format_param,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]

    # optionally add ImageViewNode to show camera image
    if run_imview and has_resource("packages", "image_view"):
        composable_nodes += [
            ComposableNode(
                package="image_view",
                plugin="image_view::ImageViewNode",
                remappings=[("/image", "/camera/image_raw")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ]

    # composable nodes in single container
    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
    )

    ### LiDAR node ###

    livox_xfer_format = DeclareLaunchArgument(
        "lidar_format",
        default_value="1", # default to LivoxMsg format
        choices=["0", "1"],
        description="Transfer-format (xfer_format) of LiDAR data, 0-PointCloud2, 1-LivoxMsg",
    )

    livox_ros2_params = [
        # make transfer (xfer) format configurable
        {"xfer_format": LaunchConfiguration("lidar_format")},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]



    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=livox_ros2_params,
    )

    return LaunchDescription(
        [
            livox_driver,  # LiDAR
            container,  # camera
            # ros2 launch arguments
            camera_launch_arg,  # camera
            format_launch_arg,  # camera
            livox_xfer_format,  # LiDAR
        ]
    )
