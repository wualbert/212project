#!/usr/bin/env python

"""
   example2.py - Same as example1 but move to the next position as soon as all servos
                 have stopped moving from the previous command
    www.pirobot.org
"""

import os
import dynamixel
import time
import random

# A function to determine if any actuator is moving
def actuators_moving(actuators):
    for actuator in actuators:
        if actuator.cache[dynamixel.defs.REGISTER['Moving']]:
            return True
    return False

# The number of Dynamixels on our bus.
nServos = 40

# Set your serial port accordingly.
if os.name == "posix":
    portName = "/dev/ttyUSB0"
else:
    portName = "COM6"
    
# Default baud rate of the USB2Dynamixel device.
baudRate = 400000

# Connect to the serial port
serial = dynamixel.serial_stream.SerialStream( port=portName, baudrate=baudRate, timeout=1)
net = dynamixel.dynamixel_network.DynamixelNetwork( serial )
net.scan( 1, nServos )

# A list to hold the dynamixels
myActuators = list()

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
    actuator.goal_torque=0
    actuator.torque_limit = 800
    actuator.max_torque = 800

# Move the servos randomly and print out their current positions
# Wait for all servos to stop moving before moving to the next set 
# of positions
while True:
    for actuator in myActuators:
        actuator.goal_position = random.randrange(4096/3, 4096*2/3)
    net.synchronize()
    time.sleep(0.001)
    for actuator in myActuators:
        actuator.read_all()
        time.sleep(0.001)
    while actuators_moving(myActuators):
        for actuator in myActuators:
            actuator.read_all()
            time.sleep(0.001)
        for actuator in myActuators:
            data = [actuator.cache[dynamixel.defs.REGISTER['Id']], actuator.cache[dynamixel.defs.REGISTER['CurrentPosition']]]
        time.sleep(0.05)
