# Handheld Setup for Recording with Mid360 Lidar and Camera

> **TODO:** add documentation for 64 bit os and ROS2

- wired network priorities for mid360
- hotspot from raspberry pi
- automatically mounting USB on startup
- `nginx` on host
- the build process

## Docker Build Problems

> **TODO:** can `docker build --network=host ...` fix the build process for the `base` stage?

related: [`pip install` during docker build sometimes fails because of network error](https://github.com/docker/for-win/issues/14667) hints that this problem varies per device and user.
This [answer on stackoverflow](https://stackoverflow.com/a/51794019) suggested using `--network=host` during the build process as well.