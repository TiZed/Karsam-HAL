#!/bin/bash

# Program pin
gpio mode 2 out

# Reset pin
gpio mode 3 out

read -p "Disconnect machine axes and press any key... " -n1 -s

# Drop program pin to set program mode
gpio write 2 0
sleep 0.5

# Reset Controller
gpio write 3 1
sleep 0.5
gpio write 3 0
sleep 0.5

# Rise program pin, device should be in program mode
gpio write 2 1

echo "KReal can be programmed now."
