#!/bin/sh
DIR=/opt/bluedonkey
systemctl stop bluedonkey.service
mkdir -p -m 755 ${DIR}
mkdir -p -m 755 ${DIR}/profiles
install -m 644 bluedonkey.service ${DIR}
ln -sf ${DIR}/bluedonkey.service /etc/systemd/system/bluedonkey.service
install -m 755 bluedonkey.py ${DIR}/
install -m 744 line_follower.py ${DIR}/
install -m 744 car_control.py ${DIR}/
install -m 755 client.py ${DIR}/
install -m 766 profiles/default_values.json ${DIR}/profiles/
ln -sf ${DIR}/bluedonkey_listen.sh /usr/local/bin/bluedonkey_listen
systemctl enable bluedonkey.service
systemctl start bluedonkey.service