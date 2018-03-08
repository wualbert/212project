#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from human_robot_comm.msg import floatlist

import numpy as np
import os

class human_comm:
	def __init__(self):
		self.gesture_counter = 0
		self.time_between_messages = .05 #seconds between velocity messages
		rospy.Subscriber('/arm/real/gripperPosition', String, self.gripperPos_callback)
		rospy.Subscriber('/arm/real/endPosition',String, self.endPos_callback)
		rospy.Subscriber('/gesture', String, self.gesture_callback) #make the callback
		#rospy.Subscriber('/velocity_track', floatlist, self.velocitytrack_callback)
		self.gripperPosPub = rospy.Publisher('/arm/target/gripperPosition', String, queue_size=1)
		self.endPosition_pub = rospy.Publisher('/arm/target/endPosition', String, queue_size=1)
		self.anglePub = rospy.Publisher('/arm/target/endAngle', String, queue_size=1)
		self.endCurrentPos = [float(), float(), float()]
		self.finished2 = False
		self.gripperCurrentPos = float()
		self.pulled = False

	def gripperPos_callback(self,msg):
		try:
			self.gripperCurrentPos = float(msg.data)

		except:
			print "Exception in gripperPos_callback"

	def endPos_callback(self,msg):
		#stores current position to be used for velocity tracking
		try:
			currentPosList = msg.data.split(',')
			self.endCurrentPos = [float(currentPosList[0]), float(currentPosList[1]), float(currentPosList[2])]

		except:
			print "Exception in endCurrentPos_callback"
	'''
	def velocitytrack_callback(self,msg):
		#process velocity to be waypoint publishing. publish 1 point at a time.
		try:
			if not self.finished2:
				return
			targetGripperVel = msg.data
			targetGripperVelX = targetGripperVel[0]
			targetGripperVelY = -0.02 #2 centimeters/second#targetGripperVel[1]
			targetGripperVelZ = targetGripperVel[2]
			#Xtarget = self.endCurrentPos[0] + targetGripperVelY*self.time_between_messages
			Ytarget = 0#str(self.endCurrentPos[1])
			Phitarget = 0#str(self.endCurrentPos[2])
			xStart = 0.756
			xEnd = 0.55
			steps = 40
			xList = np.linspace(xStart, xEnd, steps)
			pause = 0.3
			if not self.pulled:
				for xWaypoint in xList:
					Target = str(xWaypoint) + ',' + str(Ytarget) + ',' + str(Phitarget)
								
					if(self.gesture_counter == 2):
						#track velocity by publishing way points
						self.endPosition_pub.publish(Target)
					rospy.sleep(pause)
				self.pulled = True
			#alternatively using euler angle.
			#if(targetGripperVel[0] > 45):
			#	targetGripperVel[0] = 45.0;
			#
			#targetGripperEulerX = targetGripperVel[0]*.005 #scales velocity down to a max of .225 m/s
			#Xtarget = str(self.endCurrentPos[0] + targetGripperEulerX*self.time_between_messages)
			#Ytarget = str(self.endCurrentPos[1])
			#Phitarget = str(self.endCurrentPos[2])
			#Target = Xtarget + ',' + Ytarget + ',' + Phitarget
			#if(self.counter == 2):
				##track velocity by publishing way points
				#self.endPosition_pub(Target)

		except:
			print "Exception in velocitytrack_callback"
	'''	
	def pullDrawer(self):
		try:
			Ytarget = 0#str(self.endCurrentPos[1])
			Phitarget = 0#str(self.endCurrentPos[2])
			xStart = 0.756
			xEnd = 0.55
			steps = 40
			xList = np.linspace(xStart, xEnd, steps)
			pause = 0.08
			if not self.pulled:
				for xWaypoint in xList:
					Target = str(xWaypoint) + ',' + str(Ytarget) + ',' + str(Phitarget)
								
					if(self.gesture_counter == 2):
						#track velocity by publishing way points
						self.endPosition_pub.publish(Target)
					rospy.sleep(pause)
				self.pulled = True	
		except:
			pass	


	def towelWrap(self):
		pause = 0.2
		queue = [(0.867,-1.745,0.878),(1.65,-1.74,0.87),(np.pi*2/3,-np.pi*23/36,0),(np.pi*2/3,-np.pi/2,-np.pi/4),(np.pi/2,-np.pi/3,-np.pi/4),(np.pi/4,-np.pi/6,-np.pi/6)]
		steps = 20
		for i, q in enumerate(queue[0:-1]):
				print(i)
				theta1Space = np.linspace(queue[i][0],queue[i+1][0],steps)
				theta2Space = np.linspace(queue[i][1],queue[i+1][1],steps)
				theta3Space = np.linspace(queue[i][2],queue[i+1][2],steps)
				for j in range(steps):								
					self.anglePub.publish(str(theta1Space[j])+","+str(theta2Space[j])+","+str(theta3Space[j]))
					rospy.sleep(pause)
			

	def gesture_callback(self, msg):
		#fill this in
		try:
			if (self.gesture_counter==0):
				thing = msg.data
				if(thing == "G B\r\n"):
					#begin reaching for the drawer (Task A-1)
					self.endPosition_pub.publish("0.759,0,0,0") #not checked
					self.gesture_counter +=1
					print "0"
			if(self.gesture_counter==1):
				if(msg.data == "G F\r\n"):
				#close gripper
					self.gripperPosPub.publish("1")
					self.gesture_counter+=1 #add 1 to the counter
					print "1"
			if (self.gesture_counter==2):
				if(msg.data == "G A\r\n"):
					#use velocity tracking to pull drawer at velocity.
					#velocitytrack_callback takes over
					self.pullDrawer()
					self.gesture_counter+=1
					print "2"
			if (self.gesture_counter==3):
				if(msg.data != "G A\r\n"):
					#add 1 to the counter, the drawer is no longer being pulled.
					self.gripperPosPub.publish("0")
					self.gesture_counter+=1 #proceed to waiting for Task A-2 #human pulls out towel and closes the drawer
					print "3"
			if (self.gesture_counter==4):
				if(msg.data == "G A\r\n"):
					#begin Task A-2
					self.gripperPosPub.publish("1") #close gripper
					self.gesture_counter+=1
					print "4"

			if (self.gesture_counter==5):
				if(msg.data == "G D\r\n"):
					#send first way point. important for clearing the drawer and not getting hit.
					self.gesture_counter+=1
					print "5"
			if (self.gesture_counter==6):
				if(msg.data == "G F\r\n"):
					#begin wrapping procedure
					#send second waypoint #wrist turns to face human081/
					self.towelWrap()
					self.gesture_counter+=1
					print "6"
			if (self.gesture_counter==7):
				if(msg.data != "G F\r\n"):
					#stop wrapping procedure
					self.gripperPosPub.publish("0") #opens gripper
					self.gesture_counter+=1 #completed with all tasks
					print "7"
		except:
			print "thing1"



if __name__ == "__main__":
	try:
		rospy.init_node("comm")
		comm = human_comm()
		rospy.spin()
	except:
		print "anything"
