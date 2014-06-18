#!/bin/sh

for pin in 4 7 8 9 10 11 14 15 17 18 22 23 24 25 27; do

    /usr/local/bin/gpio export $pin out
    chown -R pi:pi /sys/devices/virtual/gpio/gpio$pin
    chmod -R g+w /sys/devices/virtual/gpio/gpio$pin

done