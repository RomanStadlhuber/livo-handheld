# Handheld Setup for Recording with Mid360 Lidar and Camera

> **TODO:** add documentation for 64 bit os and ROS2

- wired network priorities for mid360
- automatically mounting USB on startup

## Building the Docker Image

### For Development

Some important peculiarities about the build-process itself:

- [libcamera_ros](https://github.com/christianrauch/camera_ros) is built with `cmake` via `colcon build`
- for this [raspberrypi/libcamera](https://github.com/raspberrypi/libcamera) is checked out at `6ddd79b` to remain compatible with the available versions of `meson` and `ninja` that can be loaded by `ros2/humble` via `apt`
- the (non-ros) livox-SDK is built with `make -j 8` such as to not overload the CPU and RAM during the build process (my Pi4 8GB would otherwise freeze up)

To build the development image & start a container from it, run

```bash
./scripts/docker/build_image.sh
./scripts/docker/run_image.sh
```

#### Webinterface

To work on the webinterface, which launches & stops the ROS2 device nodes, the following steps need to be done inside the development container

- `source /opt/ros/$ROS_DISTRO/setup.bash`
- build the `recorder_runner` package with `/ros2/src/livo/recorder_runner/build.sh`
- go to `/ros2_ws/src/livo/webinterface` and `pip install -r requirements.txt`
- the total camera + LiDAR driver build time may take up to 15 or 20 minutes

from inside that folder, the development server can then be run with

```bash
python3 app.py
```

### For Deployment

To have the image and server start automatically, build the `delpoy` stage with

```bash
./scripts/docker/prod.sh
```

> **NOTE:** in order be able to access the webserver from a remote device (e.g. a smartphone),
> you will need to setup a **WiFi hotspot** and **install and setup nginx** (see more below)

## Creating a WiFi hotspot

I have basically followed [Host a Wi-Fi hotspot with a Raspberry Pi](https://www.raspberrypi.com/tutorials/host-a-hotel-wifi-hotspot/).

And the hotspot that I have used is [TP-Link WN823N](https://www.tp-link.com/de/home-networking/adapter/tl-wn823n/) which is based on the `rtl8xxxu` chipset that is conveniently **supported out of the box on debian based systems**.

<details><summary>Activating the USB-WiFi Hotspot</summary>

Since the raspberry pi 4 comes with a WiFi transmitter, the USB-WiFi module shows up as `wlan1` from `nmcli device`.

I then activated the hotspot with

```bash
wifi hotspot ssid <hotspot name> password <hotspot password> ifname wlan0
```

#### Manually installing the `rtl8192eu` Chipset Driver

Some more info can be found on this [StackOverflow Thread](https://askubuntu.com/a/1212939)

</details>

## NGINX

Save this as the default NGINX config, assuming `pi4` is the devices hostname (replace otherwise).

```nginx
# /etc/nginx/sites-available/default
server {
        listen 80;
        server_name pi4.local;
        location / {
                # the production server (./scripts/docker/prod.sh)
                proxy_pass http://127.0.0.1:8000;
        }
}
```

then

```bash
sudo service nginx restart
```

and when inside the raspberry pi's hotspot, connect to the webpage using this link

> http://pi4.local

## Webinterface Notes

**NOTE**: The flask webserver serves locally saved versions of bootstrap from local sources in `webinterface/static/bootstrap`.
See also: [bootstrap CDN links](https://www.jsdelivr.com/package/npm/bootstrap)

When hosting the service via WiFi hotspot, the client may not be able to connect to the internet to load external resources (like `fonts.googleapis.com`).


Currently, I am using `bootstrap@5.3.7`.
