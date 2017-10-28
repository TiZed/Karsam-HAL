#!/bin/bash

gpio mode 2 out
gpio mode 3 out

gpio write 2 1
gpio write 3 1
sleep 0.5
gpio write 3 0

