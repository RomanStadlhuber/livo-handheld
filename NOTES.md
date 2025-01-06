## Getting the Global Shutter Camera to work from ROS1 docker container

When installing the `ros-noetic-libcamera*` packages from ctu-mrs and running the `camera.launch` file, I am getting an error that libcamera cannot find any devices attached.

After some research I have found some old, locked forum post on the Raspberry Pi forums where people asked how to do use libcamera from docker but there was no reproducible approach to be found.
I have then found this repo: [PiCamera2-ROS2-Humble-Docker](https://github.com/nagtsnegge/PiCamera2-ROS2-Humble-Docker).
It worked for me when downgrading to a specific commit that supported the meson version (see this [pull request](https://github.com/nagtsnegge/PiCamera2-ROS2-Humble-Docker/pull/1)) (0.60) of Ubuntu 22.04 but here I still have 20.04 and meson 0.53.

Ubuntu Focal (20.04) which is used for ROS Noetic container has meson v. 0.53.3

to revert raspberrypi/libcamera to a version that allows for a meson version this old, revert to this commit:

[1db1e31: meson: reduce required version to 0.53](https://github.com/raspberrypi/libcamera/commit/1db1e31e664c1f613dc964d8519fe75d67b154b6)

*Note:* I have found this by inspecting the git blames for this line until I found a compatible meson version string.
This is from the same guy that wrote the ROS2 wrapper for libcamera, find the [patch discussion here](https://patchwork.libcamera.org/patch/15208/).
