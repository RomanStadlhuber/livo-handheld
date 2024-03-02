# Handheld Setup for LIVO

With this setup, you can run LIVO, LIO or VIO on a Raspberry Pi 4 using a CSI Camera, an Adafruit NXP 9DOF IMU and a Livox MID-360 LiDAR.
The repository contains a Docker Image for the Raspbery Pi 4 with the "Buster" OS in **32-Bit Flavor**.
Note that the 32-Bit OS is required for [`raspicam_node`](https://github.com/UbiquityRobotics/raspicam_node/tree/noetic-devel) to work,
since the underlying HAL used is only supported on 32-Bit systems.

The installation of the MID-360 LiDAR is based on the setup provided in [`mid360_docker`](https://github.com/RomanStadlhuber/mid360_docker/).

## Quick Start

This section explains everything needed to set-up the devices and get the ROS nodes running.

### Setup a static IP for EtherNet Connections

On Raspberry Pi OS this is best done by [modifying `/etc/dhcpcd.conf`](https://www.tomshardware.com/how-to/static-ip-raspberry-pi).
Follow the instructions in the [`mid360_docker`](https://github.com/RomanStadlhuber/mid360_docker/) README to setup the connection for your device (in the worst case, do some troubleshooting with WireShark, which is also explained there.)

### Build and Run the Container

The `Dockerfile` is found in the root of this repository. Run

```bash
# build the container
docker build -t livo .
# run the container
./scripts/docker/run_image.sh
```
**Note:** the `run_image.sh` script will put this repository into `/catkin_ws/src/livo` and make that folder a shared directory, such that any changes you make during the development process are retained in the repository.
