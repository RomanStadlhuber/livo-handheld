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

## Creating a WiFi hotspot

I have basically followed [Host a Wi-Fi hotspot with a Raspberry Pi](https://www.raspberrypi.com/tutorials/host-a-hotel-wifi-hotspot/).

And the hotspot that I have used is [TP-Link WN823N](https://www.tp-link.com/de/home-networking/adapter/tl-wn823n/) which is based on the `rtl8xxxu` chipset that is conveniently **supported out of the box on debian based systems**.

<details><summary>Activating the USB-WiFi Hotspot</summary>

Since the raspberry pi 4 comes with a WiFi transmitter, the USB-WiFi module shows up as `wlan1` from `nmcli device`.

I then activated the hotspot with

```bash
wifi hotspot ssid <hotspot name> password <hotspot password> ifname wlan0
```

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

> https://pi4.local

## Webinterface Notes

When hosting the service via WiFi hotspot, the client may not be able to connect to the internet to load external resources (like `fonts.googleapis.com`).

Therefore, the bootstrap file are stored in `webinterface/static/bootstrap` which are loaded in advance with `wget` from the [bootstrap CDN links](https://www.jsdelivr.com/package/npm/bootstrap).

Currently, I am using `bootstrap@5.3.7`.
