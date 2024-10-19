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

### Configure WiFi and Ethernet to be used simultaneously

In my case, it sufficed to do the following in `/etc/dhcpcd.conf`:

```
# the ethernet link
interface eth0
# required for lidar node to work
static ip_address=192.168.1.2/24
# required so eth0 can use the lidar network
static routers=192.168.1.1
# acutally optional
noipv6 
metric 300

# the WiFi link
interface wlan0
metric 200
```
> **NOTE:** the [ `metric` is important](https://raspberrypi.stackexchange.com/a/87967) for this to work.

<details> <summary>If this does not work, you might have to take additional measures.</summary>

This stackexchange post is quite a nice representation as far as the setup goes: [Connect WiFi and Ethernet simultaneously](https://raspberrypi.stackexchange.com/questions/117346/connect-wifi-and-ethernet-simultaneously). Basically, it uses

- WiFi to connect to the same network
- Ethernet to access the hardware

</details>

### Configure automatic USB mounting

Automatic mounting is required for the software to be able to detect external drives when run in "headless mode".
Follow the instructions in this [GitHub gist](https://gist.github.com/michalpelka/82d44a21c29f34ee5320c349f8bbf683).

> **NOTE:** It is imperative that you build `usbmount` [from source](https://github.com/rbrito/usbmount).
> 
> If the file `/etc/usbmount/usbmount.conf` does not exist after the installation, populate it from the source as well, e.g. using
> ```bash
> wget https://raw.githubusercontent.com/rbrito/usbmount/refs/heads/master/usbmount.conf -O /etc/usbmount/usbmount.conf
> ```

<details><summary>Handling USB drive insertion</summary>

Before running the commands that restart the `udevd` service, make sure that the USB drive
 - is not plugged in at that time
 - will not be auto-mounted by the filesystem (in case of GUI-OS)

 > **NOTE:** auto-mounting from the GUI-filesystem and `usbmount` will clash with each other, especially with regards to drive permissions on boot, which is also when the docker container will be started and thus not be able to access the media.

Go to `File Manager > Edit > Preferences > Volume Management` and disable
 - Mount mountable volumes automatically on program start-up
 - Mount removable media automatically when they are inserted

After restarting the `udevd` service, you can now insert the drive and it should be auto-mounted to `/media/usb`.

</details>

## Usage

### Setup the Container

Run

```bash
./scripts/docker/prod.sh
```
to perform the deployment build, which will also setup the web-application used to interface and configure the container to start on system boot.

### Record Data

Connect an external storage devie to the raspberry pi and make sure it is automatically mounted in either of `/media` or `/mnt`.
Find the IP of the device and connect to `<ip>:5000`, then follow the instructions on the webapp.

#### A Note on Data Formats

Please read this **important note** on the ROS message formats used.

##### Camera Image Messages

In order to save recording space and allow for larger-scale recordings, `rosbag record` will only subscribe to `/image/compressed`.
When playing back the recorded bags, use the `image_transport` packages `republish` node for decompression (as is done e.g. in LVI-SAM ([example](https://github.com/TixiaoShan/LVI-SAM/blob/master/launch/include/module_sam.launch#L21)).

> **NOTE:** it seems that because of the compressed images, `rosbag-record` will require a few minutes to perform buffering after the recording is over!

##### LiDAR Point-Cloud Format

The Livox ROS-Driver defines a [Custom Message Format](3rd/livox_ros_driver2/msg/CustomMsg.msg) which includes the start-time of the scan as well as time-offsets of individual points.
Although this message type cannot be shown in RViz, it provides the benefit of allowing for time-accurate deskewing of the 3D scan.
Therefore, the [LiDAR launch file](livo_runner/launch/msg_mid360.launch) is configured to use `xfer_format:=1`.

## Development

### Build and Run the Container

The `Dockerfile` is found in the root of this repository. Run

```bash
# build the container
docker build -t livo .
# run the container
./scripts/docker/run_image.sh
```
**Note:** the `run_image.sh` script will put this repository into `/catkin_ws/src/livo` and make that folder a shared directory, such that any changes you make during the development process are retained in the repository.
