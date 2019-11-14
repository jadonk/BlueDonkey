# The Donkeys BlueDonkey Proof of Concept
Below are instructions from the Eastern Michigan University IA Capstone project for team The Donkeys.

# Setup image

* Program the following image using Etcher.io:
  * ~~https://rcn-ee.net/rootfs/bb.org/testing/2018-12-16/buster-iot/bone-debian-buster-iot-armhf-2018-12-16-4gb.img.xz~~
  * ~~https://rcn-ee.net/rootfs/bb.org/testing/2019-03-24/buster-iot/bone-debian-buster-iot-armhf-2019-03-24-4gb.img.xz~~
  * AI: https://debian.beagleboard.org/images/am57xx-debian-9.9-lxqt-armhf-2019-08-03-4gb.img.xz
  * Black/Blue: https://debian.beagleboard.org/images/bone-debian-9.9-iot-armhf-2019-08-03-4gb.img.xz

* Get your board on the Internet
  * Your board should have an SSID of BeagleBone-XXXX, where XXXX is random. Password is 'BeagleBone'.
  * Connect to http://192.168.8.1 via your web browser to get to the console/IDE.
  * Run 'sudo connmanctl' at the console to connect to the Internet using your own access point.
```
scan wifi
services
agent on
connect <<your access point ID>>
quit
```

* Install BlueDonkey and dependencies
```sh
sudo apt update
#sudo apt install -y python3-pygame
sudo apt install -y python3-opencv python3-libgpiod mjpg-streamer-opencv-python socat
git clone https://github.com/zogheen/bluedonkey/tree/poc
cd bluedonkey
```

# Build car

* https://github.com/Sashulik/Detroit-Autonomous-Vehicle-Group/tree/master/BeagleBone-Blue_DonkeyCar

# Run

```sh
./bluedonkey.py
```

You should be able to monitor now via the car dashboard. If non-standard IP address you may need to modify the below.
http://192.168.8.1:1880/ui

The below image is of the dashboard. You can see the console, which returns any outputs of the program. 
https://drive.google.com/open?id=1TdTvL4fE4kOcIZy-1ycWyaP5opw_zlrD
This can also be redirected to the bluedonkey_listener.sh program. This can be useful as the dashboard console is overwritten every update and will only flicker with responses. 

Switching the Pause/Unpause switch will pause or unpause the car. 
Clicking JSONREADOUT will cause the program to print out all editable of the variables in car_control.py. 
Clicking LOAD will cause the program to import data from the json file to load into the variables. However, due to global vs local variables in python, this does not currently function as intended.



