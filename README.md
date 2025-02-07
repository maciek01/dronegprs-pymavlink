# dronegprs-mavsdk

Port of https://github.com/maciek01/dronegprs-mavsdk for pymavlink. In theory it should work both with PX4 and ArduPilot (Copter and Plane) but it is being tested only on the following platforms:
- actual ArduCopter-running FC (MATEK series) - physical flights
- PX4ardupilot sitl sim


Goal of the project is to establish 2-way communication and control between ground station server and a fleet of UAVs (drones, planes, ...) leveraging 3G and 4G LTE based data networks.

The idea is to enable governmental institutions to autonomously deploy drones in event of minor scale emergencies with ability to monitor and control the deployment of each indivudual UAV.

Scope of POC:

Each UAV will be equipped with MAVlink compatible flight controller, GPS module, HD Video module, Raspbery PI ZERO connected to a 3g/4g LTE modem. Raspberry PI will host the following software: MAVSDK, WebRTC video streaming software, custom-developed status reporting and control service to enable processing of ground station commands and to control the video stream.

"Ground station" is based on an HTTP RESTfull service and a control and monitoring center web site, deployed and available via internet.

Each UAV will report its stats (position, speed, altitude, heading, battery state, etc.) to the ground station via HTTP client. UAVs will be capable of independently sending the WebRTC video stream to the ground station.

Groud station will allow operatios to monitor/visualize each UAV status/location, display the video stream, and command it remotely. Examples of command: land immediately, return to home/abord the mission, deploy payload, alter course/go to waypoint, alter mission, swarm, travel to another UAV, etc..


Ground station prototype console: http://home.kolesnik.org:8000/map.html



SETUP:

1. Run camera test:

On Bullseye and later OSes:

Make sure to disable legacy camera then restart RPi.
Test the camera:
```
libcamera-vid -t 10000 -o test.h264
```

It shoudl capyture 10 secs of jmpeg video

2. Run:
```
bin/update-pi.sh
```
3. Run:
```
bin/install.sh
bin/gst-install.sh
```
4. Run
```
bin/setup.sh
```

5. Disable ModemManager
```
sudo systemctl stop ModemManager.service
sudo systemctl disable ModemManager.service
```

To enable it back if necessary:
```
sudo systemctl enable ModemManager.service
```

6. Follow steps in [uart.readme.md](./uart.readme.md)




Conduct video and streaming tests:

VIDEO STREAM VALIDATION (requiers gst - part of the install.sh script)
```
gst-launch-1.0 -e -v udpsrc port=3333 ! application/x-rtp, encoding-name=JPEG, payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink
```
or
```
libcamera-vid -v 0 -t 0 --nopreview --framerate 15 --codec mjpeg --bitrate 2500000 --profile baseline --rotation 180  --width 640 --height 480 --inline -o -|gst-launch-1.0 fdsrc ! jpegparse ! rtpjpegpay ! udpsink host=<web rtc host - ex: janus> port=3333
```

Also, with bullseye onwards, you can test streaming to a destinaction (say, vlc on your laptop):
```
libcamera-vid -v 0 -t 0 --nopreview --rotation 180  --inline -o udp://<destination ip>:3333
```



