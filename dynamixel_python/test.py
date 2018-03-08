

import math
import time
#2-DOF Robot Parameters
l1 = .15 #meters
l2 = .15
theta1 = 1.07 #radians
theta2 = 1.07
g=9.8 #m/s^2

#Given: xE,yE
#Return: theta1,theta2
def inverseKinematics(xIn, yIn):
    myTheta2 = 2*math.atan2(math.sqrt(((l1+l2)**2-(xIn**2+yIn**2))),math.sqrt((xIn**2+yIn**2.0)-(l1-l2)**2))
    myTheta1 = math.atan2(yIn,xIn)-math.atan2(l2*math.sin(myTheta2),l1+l2*math.cos(myTheta2))
    return (myTheta1, myTheta2)

freq = 25.0 #Hz
dt = 1.0/freq
programStart=time.time()#Used for timing the math.sin() function
while(True):
    startTime=time.time()#Used for timing the loop
    thetaDesired2 = inverseKinematics(0.05*math.cos(startTime-programStart)+0.15,
                                      0.05*math.sin(startTime-programStart)+0.15)
    thetaDesired2 = [math.degrees(x)+180 for x in thetaDesired2]
    print thetaDesired2
    #Used for timing
    if(time.time()-startTime>dt):
        print "Operating Too Slow"
    else:
        time.sleep(dt - (time.time()-startTime))
