#!/bin/sh

# Clean up
echo "4" > /sys/class/gpio/unexport

# Set up GPIO 4 and set to output
echo "4" > /sys/class/gpio/export
sleep 1
echo "out" > /sys/class/gpio/gpio4/direction

# Write output
echo "0" > /sys/class/gpio/gpio4/value

