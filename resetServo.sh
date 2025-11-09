#!/bin/bash

echo "=== ServoReset Starting at $(date) ===" >> /home/samonac/servoReset_debug.log

# Kill any running pigpiod processes
sudo killall pigpiod 2>> /home/samonac/servoReset_debug.log

# Restart pigpiod
sudo pigpiod >> /home/samonac/servoReset_debug.log 2>&1

echo "=== ServoReset Done at $(date) ===" >> /home/samonac/servoReset_debug.log
