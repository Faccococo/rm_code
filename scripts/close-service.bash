#!/bin/bash
sudo systemctl stop ArtinxHub.service >/dev/null
sudo kill $(pidof ArtinxHub) 2>/dev/null
sudo rm -f /lib/systemd/system/ArtinxHub.service
sudo systemctl daemon-reload

