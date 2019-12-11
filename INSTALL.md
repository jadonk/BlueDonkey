<!-- this is written in markdown so it can be placed in into the git repo -->
## Installation Instructions:

### For Hardware Setup Instructions:
Read [https://github.com/Sashulik/Detroit-Autonomous-Vehicle-Group/tree/master/BeagleBone-Blue_DonkeyCar](https://github.com/Sashulik/Detroit-Autonomous-Vehicle-Group/tree/master/BeagleBone-Blue_DonkeyCar)

### SD Card Prep
#### Verify Board Model
First identify your beagleboard model.

#### Download the Current Board IMG File
You can find the list of firmware [here](https://beagleboard.org/latest-images).

As of the last edit to this section, the current firmwares are:
* AI: [https://debian.beagleboard.org/images/am57xx-debian-9.9-lxqt-armhf-2019-08-03-4gb.img.xz](https://debian.beagleboard.org/images/am57xx-debian-9.9-lxqt-armhf-2019-08-03-4gb.img.xz)
* Black/Blue: [https://debian.beagleboard.org/images/bone-debian-9.9-iot-armhf-2019-08-03-4gb.img.xz](https://debian.beagleboard.org/images/bone-debian-9.9-iot-armhf-2019-08-03-4gb.img.xz)

#### Flashing the IMG File
You may follow the BeagleBone guide located at [https://beagleboard.org/getting-started](https://beagleboard.org/getting-started) for instructions on using `Etcher` to flash the IMG file.

##### Advanced Users
First you will need to uncompress the downloaded file. On linux you may need a package such as `xz-utils`, on mac os x you may need a program like `The Unarchiver`, and on windows you may need a program like `7zip` installed.

Advanced users on linux can write IMG file to the micro sd card with `dd`. Example:  
```sudo dd bs=4M if=am57xx-debian-9.9-lxqt-armhf-2019-08-03-4gb.img of=/dev/sdc```  
With gnu dd, you can add `status=progress` to see the progress.

### Initial BeagleBone Setup
#### Connecting to Board
Once the Micro SD card is imaged, install it into the board and power on the board.
When the board is powered up it will begin hosting a wireless access point with a name such as `BeagleBone-XXXX`, where XXXX is random. Connect to this access point using the phasephrase ‘BeagleBone’.

Once connected, open a web browser and navigate to [http://192.168.8.1](http://192.168.8.1). This will open the cloud9 console/IDE.

**NOTE**: Advanced users may SSH in if desired. User: ‘debian’, pass: ‘temppwd’.  


#### Expand SD card
If you are using a micro SD card as your boot device, you will need to expand the filesystem to utilize the extra space available. Luckily the BeagleBone repo provides a tool to do this simply. Run the following commands:

```
sudo /opt/scripts/tools/grow_partition.sh
sudo reboot
```

**NOTE:** The following reboot may be a bit slow.  
**NOTE:** If you do not expand the SD card, your usable storage space will be extremely limited.  

#### Connecting board to home network
From the Web Console, run the command `sudo connmanctl`. From the connmanctl shell run the following:

```
scan wifi
services
agent on
connect <<your access point ID>>
quit
```

Now that the board is now connected to your home network, you may disconnect from the `BeagleBone-XXXX` and reconnect to your home network with your computer.

**NOTE:** You can use tab complete when specifying the access point to connect to.

#### Package Dependencies
To install the required dependencies from the package manager, first we will need to refresh the package list.

```
sudo apt update
sudo apt install -y python3-opencv python3-libgpiod mjpg-streamer-opencv-python socat

```

#### Git Repo
Clone the git repo to the home directory.

```
cd ~
git clone https://github.com/jadonk/bluedonkey
cd bluedonkey
```

##### Install Script

```
sudo ./install.sh
```

##### Install Node-RED Palettes
Node-RED will require a handful of ‘Palettes’ installed to provide necessary functionality.
In a web browser, navigate to [http://192.168.8.1:1880](http://192.168.8.1:1880).

This will open the Node-Red editor.
In the top right click on the three bar icon and click on *Manage Palette*.
In the pop out draw select the *Install* tab, then search for and install the following palettes.

* node-red-contrib-fs-ops
* node-red-node-ui-table
* node-red-dashboard
* node-red-contrib-hostip


##### Install Node-RED Flow
In the cloud9 IDE workspace sidebar (left), at the top-right of the sidebar is a small gear icon.
Click this gear, and select *Show Home in Favorites*.
Expand *~* and *bluedonkey* in the sidebar, and open the dashboard.json file.
Now select all, and copy the contents to the clipboard (ctrl-c).

Next, in the Node-RED tab:
In the top right click on the three bar icon and select Import -> Clipboard.
In the text box, paste the dashboard.json contents.
You may now double click the *Flow 1* tab, and delete *Flow 1*.
Click the *Deploy* button in the top right save.

**NOTE**: You ***must*** deploy for the dashboard to work.  


##### Reboot
In the cloud 9 console (or via ssh) run: `sudo reboot`.

### Play
Now you can access the dashboard at [http://192.168.8.1:1880/ui](http://192.168.8.1:1880/ui)



