#!/bin/bash
sudo -v
( sleep 0.2 && sudo renice -n -20 -p $(pgrep -n python3) ) & sleep 0.5 && python3 main.py "$@"