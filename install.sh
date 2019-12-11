#!/bin/sh
# Set installation directory.
DIR=/opt/bluedonkey
# Stop bluedonkey.service if already running.
systemctl stop bluedonkey.service
# Make needed directories.
mkdir -p -m 755 ${DIR}
mkdir -p -m 755 ${DIR}/profiles
# Install necessary files in the installation directory.
install -m 644 bluedonkey.service ${DIR}
install -m 755 bluedonkey.py ${DIR}/
install -m 744 line_follower.py ${DIR}/
install -m 744 car_control.py ${DIR}/
install -m 755 client.py ${DIR}/
install -m 766 profiles/default_values.json ${DIR}/profiles/
# Link files to other locations.
ln -sf ${DIR}/bluedonkey.service /etc/systemd/system/bluedonkey.service
# Enable bluedonkey.service on startup and start it now.
systemctl enable bluedonkey.service
systemctl start bluedonkey.service
