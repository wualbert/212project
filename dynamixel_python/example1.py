#!/usr/bin/env python

"""
   example1.py - Move the servos to random positions and print out their current positions
   www.pirobot.org
"""

import os
import dynamixel
import time
import random

# The number of Dynamixels on our bus.
nServos = 40

# Set your serial port accordingly.
# if os.name == "posix":
portName = "/dev/tty.usbserial-A4012AZK"
# else:
#     portName = "COM6"
    
# Default baud rate of the USB2Dynamixel device.
baudRate = 400000

# Connect to the serial port
print "Connecting to serial port", portName, '...',
serial = dynamixel.serial_stream.SerialStream( port=portName, baudrate=baudRate, timeout=1)
print "Connected!"
net = dynamixel.dynamixel_network.DynamixelNetwork( serial )
net.scan( 1, nServos )

# A list to hold the dynamixels
myActuators = list()
print myActuators

print "Scanning for Dynamixels...",
for dyn in net.get_dynamixels():
    print dyn.id,
    myActuators.append(net[dyn.id])
print "...Done"

# Set the default speed and torque
for actuator in myActuators:
    actuator.moving_speed = 100
    actuator.synchronized = True
    actuator.torque_enable = True
    actuator.torque_control_enable = False
    actuator.goal_torque = 0
    actuator.torque_limit = 800
    actuator.max_torque = 800

# Move the servos randomly and print out their current positions
while True:
    for actuator in myActuators:
        actuator.goal_position = random.randrange(0, 1023)
    net.synchronize()
    for actuator in myActuators:
        actuator.read_all()
        time.sleep(0.1)
    #for actuator in myActuators:
        #print actuator.cache[dynamixel.defs.REGISTER['Id']], actuator.cache[dynamixel.defs.REGISTER['CurrentPosition']]
    time.sleep(4)
