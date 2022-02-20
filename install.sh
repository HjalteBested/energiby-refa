#!/bin/bash
rsync -av "./energiby.service" "/lib/systemd/system/"
sudo systemctl daemon-reload
sudo systemctl enable energiby.service