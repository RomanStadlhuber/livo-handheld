## Recording ROS2 bags (directly) to the external hard drive

With ROS1 bags, the approach was to record a bag to `/tmp` on the host and, once recording is finished and the `*.active` suffix is removed, move the bag to the external hard drive in a subprocess.
I think that with ROS2, I can start recording the data directly onto the external hard drive.

- ROS2 bag files can be split based on a max bag size (in bytes) using the `-b <size in bytes>` option
- However, recording to an external hard drive [might drop messages](https://github.com/ros2/rosbag2/issues/1579) and it is advised to use the MCAP format instead of sqlite3 since it is more efficient.
- MCAP needs to be installed first, [see how here](https://mcap.dev/guides/getting-started/ros-2)
- it might also be necessary to [override the QoS settings](https://github.com/ros2/rosbag2/issues/1579#issuecomment-1977273096) for `ros2 bag record`