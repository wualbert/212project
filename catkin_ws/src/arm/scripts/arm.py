#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#from human_robot_comm.msg import floatlist
import dynamixel

import numpy as np
import os
import warnings

# Remove before moving to ROS
# from armVisualizer import ArmVisualizer

def twoPiToPlusMinusPi(angle):
	angle = np.mod(angle,2*(np.pi))
	if angle > np.pi:
		angle = -(2*np.pi-angle)
	return angle

class RobotArm:
	def __init__(self, simulated=False):
		warnings.filterwarnings("error")
		#initialize gesture counter
		self.gesture_counter = 0

		# Motor assignment
		self.wristID = 2
		self.elbowID = 1
		self.gripperID = 3

		self.elbowOffset = 2000
		self.wristOffset = 1726

		# Arm endpoints, in terms of angles
		# TODO: Values TBD
		self.theta1Min = -np.pi *12/ 18
		self.theta1Max = np.pi *12/ 18
		self.theta2Min = -np.pi*23/36
		self.theta2Max = np.pi*23/36
		self.theta3Min = -np.pi *2/3
		self.theta3Max = np.pi *2/3
		self.gripperMin = -np.pi * 5 / 6  # will be read from motor
		self.gripperMax = np.pi * 5 / 6  # will be read from motor

		self.simulated = simulated
		self.baseSpeed = 50
		if not simulated:
			self.dynamixelNet = self.setSerial()
			self.motors = dict()
			self.motorStatuses = dict()
			for dyn in self.dynamixelNet.get_dynamixels():
				self.motors[dyn.id] = self.dynamixelNet[dyn.id]
				self.motorStatuses[dyn.id] = dict()
			rospy.loginfo(str("Dynamixels connected:")+str(self.motors.keys()))
			self.setInitialParameters()
		else:
			print 'Simulation mode'

		# Constant definition
		self.MXprecision = np.pi * 2 / 4096
		self.AXprecision = np.pi * 10.0 / 6.0 / 1024
		# Current arm position
		self.seaPose = None
		self.realThetaPose = None
		self.targetThetaPose = (0,0,0)  # default starting posiftion
		self.targetGripperPose = 0 # float between 0 and 1
		self.realXYPhiPose = None
		self.realGripperPose = None


		# Arm Geometry
		self.l1 = 0.2949
		self.l2 = 0.2921
		self.l3 = 0.1727

		# self.visualizer = ArmVisualizer()
		rospy.loginfo(str("Initializing arm hardware..."))

		if not self.simulated:
			self.sync()
		"""
			else:callback
				self.simulatedSync()
		"""
		# ROS publisher and subscribers
		rospy.Subscriber('/arm/target/endPosition', String, self.endPosCmd_callback)
		self.endPosition_pub = rospy.Publisher('/arm/real/endPosition', String, queue_size=1)
		rospy.Subscriber('/arm/target/gripperPosition', String, self.gripperPosCmd_callback)
		self.gripperPosition_pub = rospy.Publisher('/arm/real/gripperPosition', String, queue_size=1)
		self.sea_pub = rospy.Publisher('/posPub', Float32, queue_size=1)
		rospy.Subscriber('/uPub', Float32, self.seaPos_callback)

		#For debug purposes: control arm angles directly
		self.endAngle_pub = rospy.Publisher('/arm/real/endAngle', String, queue_size=1)
		rospy.Subscriber('/arm/target/endAngle', String, self.endAngleCmd_callback)

		rospy.loginfo(str("Initialization complete!"))

	def setSerial(self):
		# The number of Dynamixels on our bus.
		nServos = 3
		# Dynamixel 2(AX-12A), 1(MX-64), 0(MX-106)

		# Set serial port
		# if os.name == "posix":  # linux
		portName = "/dev/ttyUSB0"
		# else:  # mac
		# portName = "/dev/tty.usbserial-A4012AZK"

		# Set baud rate
		baudRate = 400000

		# Connect to the serial port
		rospy.loginfo(str("Connecting to serial port")+str(portName)+str("..."))
		serial = dynamixel.serial_stream.SerialStream(port=portName, baudrate=baudRate, timeout=1)
		print "Connected!"
		net = dynamixel.dynamixel_network.DynamixelNetwork(serial)
		net.scan(1, nServos)
		return net

	def setInitialParameters(self):
		"""
		Set dynamixel parameters
		:return:
		"""
		# TODO: set specific parameters for individual motors
		for motorID in self.motors.keys():
			self.motors[motorID].moving_speed = self.baseSpeed
			self.motors[motorID].synchronized = True
			self.motors[motorID].torque_enable = True
			self.motors[motorID].torque_control_enable = False
			self.motors[motorID].goal_torque = 0
			self.motors[motorID].torque_limit = 400
			self.motors[motorID].max_torque = 400
		#Read preprogrammed motor attributes
		if motorID ==self.gripperID:
			self.gripperMin = self.motors[motorID].cw_angle_limit
			self.gripperMax = self.motors[motorID].ccw_angle_limit
		#print "gripperMin: " + str(self.gripperMin)
		#print "gripperMax: " + str(self.gripperMax)
		return True

	def _angleToMotor(self, robotPose):
		"""

		:param robotPose: (theta1,theta2,theta3)
		:return: motor position values for (shoulder, elbow, wrist). 0~4095 for MXs, 0~1024 for AXs
		"""
		# TODO: Error handling
		theta1, theta2, theta3 = robotPose
		# TODO: SEA control
		shoulder = 0

		elbowAngle = max(self.theta2Min, min(self.theta2Max, theta2))
		wristAngle = max(self.theta3Min, min(self.theta3Max, theta3))
		# TODO: SEA control
		# TODO: check motor zero position and sea
		return (shoulder, int(elbowAngle / self.MXprecision) + self.elbowOffset,
				int(wristAngle / self.MXprecision) + self.wristOffset)

	def _motorToAngle(self, motorPose):
		"""

		:param motorPose: motor position. 0~4096 for MXs, 0~1024 for AXs
		:return: (theta1, theta2, theta3)
		"""
		return (
		0, (motorPose[1] - self.elbowOffset) * self.MXprecision, (motorPose[2] - self.wristOffset) * self.MXprecision)

	def _motorToGripper(self, gripperPose):
		"""
		:return: gripper position in percentage form
		"""
		return (gripperPose - self.gripperMin) / (self.gripperMax - self.gripperMin)

	def _robotPoseDistance(self, pose1, pose2):
		"""
		compares how far the distance is between two robot
		:param pose1: position 1
		:param pose2: position 2
		:return: scalar distance
		"""
		# TODO: optimize the distance definition
		distance = 0
		for i in range(3):
			distance += abs(pose1[i] - pose2[i])
		return distance

	def _updateCurrentStatuses(self):
		"""
		updates motor information
		:return:
		"""
		try:
			for motorKey in self.motors.keys():
				self.motorStatuses[motorKey]['position'] = self.motors[motorKey].current_position
				# TODO: read more states

			# TODO: read SEA status
			motorPose = (self.seaPose, self.motorStatuses[self.elbowID]['position'], self.motorStatuses[self.wristID]['position'])
			gripperPose = self.motorStatuses[self.gripperID]['position']
			# wait for motion to end
			self.realThetaPose = self._motorToAngle(motorPose)
			self.realXYPhiPose = self.forwardKinematics(self.realThetaPose)
			self.realGripperPose = self._motorToGripper(gripperPose)
			return True
		except:
			return False

	def _basicMove(self):
		#print(self.targetThetaPose)
		motorPose = self._angleToMotor(self.targetThetaPose)
		self.motors[self.elbowID].goal_position = int(motorPose[1])
		self.motors[self.wristID].goal_position = int(motorPose[2])
		#move gripper
		self.motors[self.gripperID].goal_position = int(self.targetGripperPose*(self.gripperMax-self.gripperMin)+self.gripperMin)
		#print self.motors[self.gripperID].goal_position
		#TODO: implement
		#TODO: convert distance to angle
		try:
			self.dynamixelNet.synchronize()
		except:
			print "re-initializing"
			self.setInitialParameters()

	def _lineMove(self):
		"""
		Moves the robot straight to the target position
		:return:
		"""
		# TODO: Implement
		#Set goal position
		motorPose = self._angleToMotor(self.targetThetaPose)
		self.motors[self.elbowID].goal_position = int(motorPose[1])
		self.motors[self.wristID].goal_position = int(motorPose[2])
		try:
			while self.realThetaPose != self.targetThetaPose:
				self._updateCurrentStatuses()
				targetXYPhiPose = self.inverseKinematics(self.targetThetaPose)
				poseDifference = np.asarray(self.targetThetaPose - self.realXYPhiPose)
				unitVelocityVector = poseDifference/np.linalg.norm(poseDifference)
				unitJointVelocity = np.matmul(invJacobian(self.realThetaPose), unitVelocityVector)

				#Set motor speed

				self.motors[self.elbowID].moving_speed = unitJointVelocity[1]*self.baseSpeed
				self.motors[self.wristID].moving_speed = unitJointVelocity[2]*self.baseSpeed

				self.dynamixelNet.synchronize()
				rospy.sleep(0.1)
		except:
			pass

	def inverseKinematics(self, targetPose):
		"""
		Returns the target position in terms of all possible arm configuration.
		:param targetPose: (x,y,phi) in space, w.r.t. the robot base
		:return:(theta1, theta2, theta3) as the three angles
		"""
		x, y, phi = targetPose
		# correction for l3
		try:
			x -= self.l3 * np.cos(phi)
			y -= self.l3 * np.sin(phi)
			r = (x ** 2 + y ** 2) ** 0.5
			theta1_1 = np.arctan2(y, x) + np.arccos((self.l1 ** 2 + r ** 2 - self.l2 ** 2) / (2 * self.l1 * r))
			theta2_1 = np.pi + np.arccos((self.l1 ** 2 + self.l2 ** 2 - r ** 2) / (2 * self.l1 * self.l2))
			theta3_1 = phi - theta1_1 - theta2_1

			theta1_2 = np.arctan2(y, x) - np.arccos((self.l1 ** 2 + r ** 2 - self.l2 ** 2) / (2 * self.l1 * r))
			theta2_2 = np.pi - np.arccos((self.l1 ** 2 + self.l2 ** 2 - r ** 2) / (2 * self.l1 * self.l2))
			theta3_2 = phi - theta1_2 - theta2_2

			theta1_1 = twoPiToPlusMinusPi(theta1_1)
			theta2_1 = twoPiToPlusMinusPi(theta2_1)
			theta3_1 = twoPiToPlusMinusPi(theta3_1)
			theta1_2 = twoPiToPlusMinusPi(theta1_2)
			theta2_2 = twoPiToPlusMinusPi(theta2_2)
			theta3_2 = twoPiToPlusMinusPi(theta3_2)
			return [(theta1_1, theta2_1, theta3_1), (theta1_2, theta2_2, theta3_2)]
		except RuntimeWarning:
			print "No inverse kinematic solution!"

	def forwardKinematics(self, robotPose):
		"""
		Returns the end effector position given the arm configuration.
		:param robotPose: (theta1, theta2, theta3) in space, w.r.t. the robot base
		:return:(x,y,phi) as the three angles
		"""
		theta1, theta2, theta3 = robotPose
		return (
		self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2) + self.l3 * np.cos(theta1 + theta2 + theta3),
		self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2) + self.l3 * np.sin(theta1 + theta2 + theta3),
		theta1 + theta2 + theta3)

	def jacobian(self, robotPose):
		"""
		Returns the Jacobian of the given position.
		:param robotPose: arm position
		:return: numpy matrix invJacobian
		"""
		theta1, theta2, theta3 = robotPose
		jacobianList = [
			-self.l1 * np.sin(theta1) - self.l2 * np.sin(theta1 + theta2) - self.l3 * np.sin(theta1 + theta2 + theta3),
			-self.l2 * np.sin(theta1 + theta2) - self.l3 * np.sin(theta1 + theta2 + theta3),
			- self.l3 * np.sin(theta1 + theta2 + theta3),
			self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2) + self.l3 * np.cos(theta1 + theta2 + theta3),
			self.l2 * np.cos(theta1 + theta2) + self.l3 * np.cos(theta1 + theta2 + theta3),
			self.l3 * np.cos(theta1 + theta2 + theta3),
			1, 1, 1]
		return np.reshape(jacobianList, (3, 3))

	def invJacobian(self, robotPose):
		"""
		Returns the inverse Jacobian of the given position. Returns None if singular.
		:param target_pose: end effector position
		:return: numpy matrix invJacobian. Returns none if singular.
		"""
		# TODO:
		try:
			invJ = np.linalg.inv(self.jacobian(robotPose))
			return invJ
		except np.linalg.LinAlgError:
			return None

	def setTargetPosition(self, targetPose, favorLeft=None):
		"""
		Steers the robot to the target position. Returns true if this is possible, false if not.
		:param targetPose: target position
		:param ccw: None by default. Returns closest(see RobotArm._robotPoseDistance) configuration if none.
					Favors left config (top view) if True, right if False.
		:return: boolean of whether the point is reachable
		"""
		try:
			candidatePose = self.inverseKinematics(targetPose)
			# self.visualizer.plotTarget((targetPose[0:2]))
			#check if angle is in range
			if favorLeft is not None:
				if favorLeft:
					if candidatePose[0][0] <= candidatePose[1][0]:
						return self.setTargetAngle(candidatePose[0])
					else:
						return self.setTargetAngle(candidatePose[1])
				else:
					if candidatePose[0][0] > candidatePose[1][0]:
						return self.setTargetAngle(candidatePose[0])
					else:
						return self.setTargetAngle(candidatePose[1])
						# check which option is quicker to move to
			if self._robotPoseDistance(candidatePose[0], self.realThetaPose) < self._robotPoseDistance(candidatePose[1],
																									   self.realThetaPose):
				return self.setTargetAngle(candidatePose[0])
			else:
				return self.setTargetAngle(candidatePose[1])

		except:
			print "Failed to set target position"
			return False

	def setTargetAngle(self, robotPose):
		"""
		:param robotPose: angles (theta1, theta2, theta3) for the robot to move to
		:return: True
		"""
		# TODO: check if given position is reachable
		temp = []
		for angle in robotPose:
			temp.append(twoPiToPlusMinusPi(angle))
		robotPose = temp
		if robotPose[0] < self.theta1Min or robotPose[0] > self.theta1Max or robotPose[1] < self.theta2Min or robotPose[1] > self.theta2Max or robotPose[2] < self.theta3Min or robotPose[2] > self.theta3Max:
			print "Command out of admissible space"
		else:
			self.targetThetaPose = robotPose
			return True

	def pinch(self, distance):
		"""
		Keep the distance between the gripper ends at distance. Enter 0 for pinching.
		:param distance:
		:return:
		"""
		# TODO: implement for distance
		# Currently by percentage
		gripperPercent = min(max(distance, 0.0), 1.0)
		self.targetGripperPose = gripperPercent
		return True

	def sync(self):
		"""
		Actuates the motors. Call this after setting target positions.
		Publish states
		:return:
		"""
		try:
			if self.simulated:
				return self.simulatedSync()
			# read motor states before moving
			self._updateCurrentStatuses()
			# move motors
			# TODO: Implement advance moving
			self._basicMove()

			#Publish to SEA
			#!!!Important!!! SEA command should be between -180~180 deg
			if self.targetThetaPose[0] >= self.theta1Min and self.targetThetaPose[0] <= self.theta1Max:
					self.sea_pub.publish((self.targetThetaPose[0]*180.0/np.pi))
			else:
				print "Invalid SEA Command!"
				# read motor states after moving
				self._updateCurrentStatuses()
				# Remove before moving to ROS
				# self.visualizer.plotArm(self.realThetaPose)

				return True

		except:
			return False

	def simulatedSync(self):
		"""
		For simulation purposes
		:return: True
		"""
		self.realThetaPose = self.targetThetaPose
		self.realXYPhiPose = self.forwardKinematics(self.realThetaPose)
		self.realGripperPose = self.targetGripperPose
		#self.visualizer.plotArm(self.targetThetaPose, True)
		#self.sea_pub.publish((self.targetThetaPose[0]*180.0/np.pi)%360.0)
		return True

	def endPosCmd_callback(self, msg):
		try:
			targetPoseList = msg.data.split(',')
			targetPoseFloat = [float(x) for x in targetPoseList[0:3]]
			direction = None
			if len(targetPoseList) > 3:
				# Selection for left/right
				direction = bool(int(targetPoseList[3]))
			self.setTargetPosition(targetPoseFloat, direction)
			self.sync()
			while(self.motors[self.elbowID].current_speed != 0 or self.motors[self.wristID].current_speed!=0):
				self.sync()
				realTheta = String()
				realThetaPoseString = ','.join(map(str, self.realThetaPose))
				realTheta.data = realThetaPoseString
				self.endAngle_pub.publish(realTheta)
				realPosition = String()
				realPositionString = ','.join(map(str, self.realXYPhiPose))
				realPosition.data = realPositionString
				self.endPosition_pub.publish(realPosition)
				rospy.sleep(0.1)

		except:
			print "Exception in endPosCmd_callback"
	
	def gripperPosCmd_callback(self, msg):
		try:
			targetGripperPose = float(msg.data)
			self.pinch(targetGripperPose)
			self.sync()
			realGripperPosition = String()
			realGripperPosition.data = str(self.realGripperPose)
			self.gripperPosition_pub.publish(realGripperPosition)

		except:
			print "Exception in gripperPosCmd_callback"
	
	def velocitytrack_callback(self,msg):
		#fill this in
		try:
			targetGripperVel = float(msg.data)
		except:
			print "Exception in velocitytrack_callback"

	def seaPos_callback(self,msg):
		try:
			self.seaPose = float(msg.data)
		except:
			pass

	def endAngleCmd_callback(self, msg):
		try:
			targetAngleList = msg.data.split(',')
			targetPoseFloat = [float(x) for x in targetAngleList[0:3]]
			self.setTargetAngle(targetPoseFloat)
			self.sync()
			realPosition = String()
			realThetaPoseString = ','.join(map(str, self.realThetaPose))
			realPosition.data = realThetaPoseString
			self.endAngle_pub.publish(realPosition)

		except:
			print "Exception in endAngleCmd_callback"



if __name__ == "__main__":
	try:
		rospy.init_node("arm_main")
		arm = RobotArm(False)
		rospy.spin()
	except:
		pass
