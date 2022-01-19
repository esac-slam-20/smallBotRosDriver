#!/bin/bash
SCRIPT=`realpath $0`
DIRNAME=`dirname $SCRIPT`

bash $DIRNAME/systemd/ae87LiteDriver.bash
sudo cp -r $DIRNAME/systemd/*.service /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl enable ae87LiteDriver
sudo systemctl start ae87LiteDriver

