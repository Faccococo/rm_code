#!/bin/bash

sudo systemctl stop ArtinxHub.service >/dev/null
sudo kill $(pidof ArtinxHub) 2>/dev/null
sudo rm -rf /opt/artinx-hub
sudo cp -r ./ /opt/artinx-hub
sudo cp -f ./bringup_templates/ArtinxHub.service /lib/systemd/system/ArtinxHub.service
sudo echo $1 > /opt/deploy_target.conf
sudo systemctl enable ArtinxHub.service
