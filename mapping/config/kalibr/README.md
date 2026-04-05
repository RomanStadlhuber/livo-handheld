# Using Kalibr with ROS2

For a ROS2-compatible version of kalibr, check out
[Toyotas kalibr fork](github.com/ToyotaResearchInstitute/kalibr).

> **Important:** at the time of writing, the toolbox can only handle ROS2
> bags with a single `.db3` or `.mcap` file, meaning split files are not
> supported (+ a `metadata.yaml` file needs to be present)!!
>
> As such, it is probably best to record a separate bag containing only the
> camera calibration topic.

**NOTE:** at the time of writing, kalibr2 does not support camera-IMU calibration!! <br/>
See also: this [GitHub Issue](https://github.com/ToyotaResearchInstitute/kalibr/issues/49).
If you need require extrinsic calibration, use the
[original kalibr toolbox](https://github.com/ethz-asl/kalibr).

## Build the Container

```bash 
git clone github.com/ToyotaResearchInstitute/kalibr && cd kalibr
mkdir data  # copy the rosbag & calibr config here
docker compose build kalibr_ros 
docker compose run kalibr_ros
```

## Build the Toolbox

This will start & attach to the container.
Inside the container, do

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build  # the toolbox needs to be built first
source ./install/setup.bash
```

## Prepare the Configuration

OpenVINS has a
[great guide](https://docs.openvins.com/gs-calibration.html#gs-calib-cam-static)
about getting calibration right.
If anything is unclear, have a look at it!

- measure your Aprilgrid & insert the values
- don't forget to set the image topic (this repository uses `camera/image_raw`)
- remember that the path to sqlite3 `.db3` file needs to be in the docker file system!



```yaml
# /kalibr/data/config.kalibr.yaml
board:
  # NOTE: A0 dimensions
  # the larger the print, the better
  target_type: 'aprilgrid' # grid type
  tagCols: 6               # number of apriltags in the x direction
  tagRows: 6               # number of apriltags in the y direction
  tagSize: 0.088           # size of apriltag, edge to edge [m]
  tagSpacing: 0.302       # ratio of space between tags to tagSize

cameras:
  camera0:
    # for fisheye, use
    # 'pinhole-equi'
    # see also:
    # https://docs.openvins.com/gs-calibration.html#gs-calib-cam-static
    model: 'pinhole-radtan'
    # NOTE: currently streaming 640x480 [px]
    focal_length_fallback: 100
    source:
      rosbag_path: '/kalibr/data/your_bag/your_bag_0.db3'
      topic: '/camera/image_raw'
```

## Run the calibration

You need to create the results folder first `mkdir data/results`.

```bash
ros2 run kalibr2_ros kalibr_calibrate_cameras \
    --config data/config.kalibr.yaml \
    --output-dir data/results
```

Processing will take a while to complete.
It will write the results into the folder.

I have provided an example output file in
[`results.camera0.yaml`](results.camera0.yaml).

## Enter the Results into the System Configuration

For example, in [`mapping_mid360.yaml`](../mapping_mid360.yaml), set

```yaml
# ...
intrinsics:
  camera:
    model: "PinholeRadTan" # or "PinholeEquidistant", but not supported yet
    pinhole_parameters: # 640x480
      fx: 807.406
      fy: 806.768
      cx: 327.545
      cy: 233.247
    distortion_coefficients:
      - -0.501859
      - 0.270368
      - 0.000114465
      - -0.00185591
```


