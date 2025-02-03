## Recording ROS2 bags (directly) to the external hard drive

With ROS1 bags, the approach was to record a bag to `/tmp` on the host and, once recording is finished and the `*.active` suffix is removed, move the bag to the external hard drive in a subprocess.
I think that with ROS2, I can start recording the data directly onto the external hard drive.

- ROS2 bag files can be split based on a max bag size (in bytes) using the `-b <size in bytes>` option
- However, recording to an external hard drive [might drop messages](https://github.com/ros2/rosbag2/issues/1579) and it is advised to use the MCAP format instead of sqlite3 since it is more efficient.
- MCAP needs to be installed first, [see how here](https://mcap.dev/guides/getting-started/ros-2)
- it might also be necessary to [override the QoS settings](https://github.com/ros2/rosbag2/issues/1579#issuecomment-1977273096) for `ros2 bag record`

## Getting the Global Shutter Camera to work from ROS1 docker container

When installing the `ros-noetic-libcamera*` packages from ctu-mrs and running the `camera.launch` file, I am getting an error that libcamera cannot find any devices attached.

### Checkout `raspberrypi/libcamera` to build from source

After some research I have found some old, locked forum post on the Raspberry Pi forums where people asked how to do use libcamera from docker but there was no reproducible approach to be found.
I have then found this repo: [PiCamera2-ROS2-Humble-Docker](https://github.com/nagtsnegge/PiCamera2-ROS2-Humble-Docker).
It worked for me when downgrading to a specific commit that supported the meson version (see this [pull request](https://github.com/nagtsnegge/PiCamera2-ROS2-Humble-Docker/pull/1)) (0.60) of Ubuntu 22.04 but here I still have 20.04 and meson 0.53.

Ubuntu Focal (20.04) which is used for ROS Noetic container has meson v. 0.53.3

to revert raspberrypi/libcamera to a version that allows for a meson version this old, revert to this commit:

[1db1e31: meson: reduce required version to 0.53](https://github.com/raspberrypi/libcamera/commit/1db1e31e664c1f613dc964d8519fe75d67b154b6)

*Note:* I have found this by inspecting the git blames for this line until I found a compatible meson version string.
This is from the same guy that wrote the ROS2 wrapper for libcamera, find the [patch discussion here](https://patchwork.libcamera.org/patch/15208/).

### Install `libgnutls` which is required to build `libcamera`

Inside the container, run

```bash
apt update && apt install -y -q libgnutls*
```


### Running the build

- clone `libcamera` and checkout to the aforementioned commit
- install `libgnutls` and the other dependencies of the aforementioned ROS2 example container
- run `meson build` and `ninja -C build/ install` inside the `libcamera` repo
- the `ninja` build will take a few minutes to complete


### A different possibility: refactor the CTU-MRS build files

I have found that they check out their own pre-built version of libcamera.
If that **truly** is the issue, maybe I can refactor their packages
Their [libcamera ROS wrapper](https://github.com/ctu-mrs/libcamera_ros) checks out their own fork ([see CMakeLists.txt](https://github.com/ctu-mrs/libcamera_ros/blob/master/CMakeLists.txt#L61)) and builds + installs it.
Here I would need to use the original libcamera repo and revert to the commit using the `GIT_TAG<full commit hash>` with `GIT_SHALLOW OFF` and also for some reason the `BUILD_COMMAND`, namely `meson compile -C build` does not work for me either.

So there is a lot that would need to  be done and even then I am unsure if it would even work.
For now I think it might be easier for me to just **install PiCamera2** and stream from a custom python ROS node instead.
I want to get stuff done afer all..
