#!/usr/bin/env python

"""
   2.12 Dynamixel Sinusoidal Position Control (Constant Torque Mode) Example
   by Daniel J. Gonzalez - dgonz@mit.edu

   This example is meant to emulate the final state of the 2-DOF Robot Lab, in which
   the angles of the robots were commanded to be sinusoidal.

   Note: If you stop and start a program too fast, the Dynamixels will get confused.
   Should this happen, simply stop the program again (CTRL+C), wait 5 seconds,
   and start it again.

   Note 2: Depending on loop optimization and your operating system, the main loop
   below will not run much faster than 25Hz. 
"""

import os
import dynamixel
import time
import random
import math

# The number of Dynamixels on our bus.
nServos = 1

# Set your serial port accordingly.
if os.name == "posix":
    portName = "/dev/ttyUSB0"
else:
    portName = "COM7"
    
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
    actuator.torque_control_enable = True
    actuator.torque_limit = 500
    actuator.max_torque = 500
    
    #Present Position is address 36 (0-360 mapped to 0-4096, 2048 is straight ahead)
    #Present Speed is address 38 (0-117RPM mapped to 1-1023, 0 is MaxRPM!!!)
    #Present Load is address 40
    #Clockwise Torque (0 to 1023) is mapped to (1024-2047)
    #Counter-Clockwise Torque (0-1023)is mapped to (0-1023)
    #Torque Control Mode is address 70, and should be set to 1
    #Goal Torque is address 71

def mapTorque(torque):
    #Positive torque is CCW, negative torque is CW
    #Input val is +-100% of max torque
    if abs(torque)>100:
        torque = 100 * torque/abs(torque)
    if(torque>=0):
        return int(torque/100.0 * 1023)
    else:
        return int(abs(torque)/100.0 * 1023)+1024

# Main Loop
freq = 25.0 #Hz
dt = 1.0/freq
programStart=time.time()#Used for timing the math.sin() function
while True:
    startTime=time.time()#Used for timing the loop
    
    kP = 0.025
    thetaDesired = 60.0/180.0 * 2048 * math.sin(startTime-programStart)+2048
    
    for actuator in myActuators:
        actuator.read_all()
        theta = actuator.cache[dynamixel.defs.REGISTER['CurrentPosition']]
        command=mapTorque(kP*(thetaDesired-theta))
        actuator.goal_torque=int(command)
    net.synchronize()
    
    #Used for timing
    if(time.time()-startTime>dt):
        print "Operating Too Slow"
    else:
        time.sleep(dt - (time.time()-startTime))
