#!/usr/bin/env python

import dynamixel
import time
import numpy as np


# Remove before moving to ROS
# from armVisualizer import ArmVisualizer

class RobotArm:
    def __init__(self, simulated=False):
        self.simulated = simulated
        self.baseSpeed = 50
        # Motor related constants
        self.wristID = 24
        self.elbowID = 36
        self.gripperID = 6
        self.MXprecision = np.pi * 2 / 4096
        self.AXprecision = np.pi * 10.0 / 6.0 / 1024
        self.wristOffSet = 2048
        self.elbowOffset = 2048

        if not simulated:
            self.dynamixelNet = self.setSerial()
            self.motors = dict()
            self.motorStatuses = dict()
            for dyn in self.dynamixelNet.get_dynamixels():
                self.motors[dyn.id] = self.dynamixelNet[dyn.id]
                self.motorStatuses[dyn.id] = dict()
            print 'Dynamixels connected:', self.motors.keys()
            self.setInitialParameters()
        else:
            print 'Simulation mode'
        # Current motor position
        self.realThetaPose = None
        self.targetThetaPose = (0, 0, 0)  # default starting position
        self.targetGripperPose = 1
        self.realXYPhiPose = None

        # Arm Geometry
        self.l1 = 0.3556
        self.l2 = 0.2794
        self.l3 = 0.1143

        # Arm endpoints, in terms of angles
        # TODO: Values TBD
        self.theta1Min = -np.pi / 2
        self.theta1Max = np.pi / 2
        self.theta2Min = -np.pi / 2
        self.theta2Max = np.pi / 2
        self.theta3Min = -np.pi / 2
        self.theta3Max = np.pi / 2

        # self.visualizer = ArmVisualizer()
        if not self.simulated:
            self.sync()
        else:
            self.simulatedSync()

        print 'Initialization complete!'

    def setSerial(self):
        # The number of Dynamixels on our bus.
        nServos = 40
        # Dynamixel 6(AX-12A), 24(MX-64), 36(MX-106)

        # Set serial port
        # if os.name == "posix":  # linux
        #     portName = "/dev/ttyUSB0"
        # else:  # mac
        portName = "/dev/tty.usbserial-A4012AZK"

        # Set baud rate
        baudRate = 400000

        # Connect to the serial port
        print "Connecting to serial port", portName, '...',
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
            self.motors[motorID].torque_limit = 200
            self.motors[motorID].max_torque = 300
            if motorID == self.gripperID:
                self.gripperMin = self.motors[motorID].cw_angle_limit
                self.gripperMax = self.motors[motorID].ccw_angle_limit
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
                int(wristAngle / self.MXprecision) + self.wristOffSet)

    def _motorToAngle(self, motorPose):
        """

        :param motorPose: motor position. 0~4096 for MXs, 0~1024 for AXs
        :return: (theta1, theta2, theta3)
        """
        return (
        0, (motorPose[1] - self.elbowOffset) * self.MXprecision, (motorPose[2] - self.wristOffSet) * self.MXprecision)

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
        for motorKey in self.motors.keys():
            self.motorStatuses[motorKey]['position'] = self.motors[motorKey].current_position
            # TODO: read more states

        # TODO: read SEA status
        motorPose = (0, self.motorStatuses[self.elbowID]['position'], self.motorStatuses[self.wristID]['position'])
        # wait for motion to end
        self.realThetaPose = self._motorToAngle(motorPose)
        self.realXYPhiPose = self.forwardKinematics(self.realThetaPose)
        return True

    def _basicMove(self):
        motorPose = self._angleToMotor(self.targetThetaPose)
        self.motors[self.elbowID].goal_position = motorPose[1]
        self.motors[self.wristID].goal_position = motorPose[2]
        # TODO: add support for SEA base

        # move gripper
        self.motors[self.gripperID].goal_position = self.targetGripperPose
        # TODO: implement
        # TODO: convert distance to angle
        # self.motors[self.gripperID].goal_position = self.targetGripperPose

        self.dynamixelNet.synchronize()
        time.sleep(2.5)

    def _lineMove(self):
        """
        Moves the robot straight to the target position
        :return:
        """
        # TODO: Implement
        self._updateCurrentStatuses()
        while self.realThetaPose != self.targetThetaPose:
            targetXYPhiPose = self.inverseKinematics(self.targetThetaPose)

            poseDifference = np.asarray(self.targetThetaPose - self.realXYPhiPose)

    def inverseKinematics(self, targetPose):
        """
        Returns the target position in terms of all possible arm configuration.
        :param targetPose: (x,y,phi) in space, w.r.t. the robot base
        :return:(theta1, theta2, theta3) as the three angles
        """
        # TODO: Change to be compatible with ROS
        x, y, phi = targetPose
        # correction for l3
        x -= self.l3 * np.cos(phi)
        y -= self.l3 * np.sin(phi)
        r = (x ** 2 + y ** 2) ** 0.5
        theta1_1 = np.arctan2(y, x) + np.arccos((self.l1 ** 2 + r ** 2 - self.l2 ** 2) / (2 * self.l1 * r))
        theta2_1 = np.pi + np.arccos((self.l1 ** 2 + self.l2 ** 2 - r ** 2) / (2 * self.l1 * self.l2))
        theta3_1 = phi - theta1_1 - theta2_1

        theta1_2 = np.arctan2(y, x) - np.arccos((self.l1 ** 2 + r ** 2 - self.l2 ** 2) / (2 * self.l1 * r))
        theta2_2 = np.pi - np.arccos((self.l1 ** 2 + self.l2 ** 2 - r ** 2) / (2 * self.l1 * self.l2))
        theta3_2 = phi - theta1_2 - theta2_2
        # TODO: check if angle is in range
        return [(theta1_1, theta2_1, theta3_1), (theta1_2, theta2_2, theta3_2)]

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
        candidatePose = self.inverseKinematics(targetPose)
        # self.visualizer.plotTarget((targetPose[0:2]))
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

    def setTargetAngle(self, robotPose):
        """
        :param robotPose: angles (theta1, theta2, theta3) for the robot to move to
        :return: True
        """
        self.targetThetaPose = robotPose
        # TODO: check if given position is reachable
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
        self.targetGripperPose = gripperPercent * (self.gripperMax - self.gripperMin) + self.gripperMin
        return True

    def sync(self):
        """
        Actuates the motors. Call this after setting target positions.
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
            # read motor states after moving
            self._updateCurrentStatuses()
            # Remove before moving to ROS
            # self.visualizer.plotArm(self.realThetaPose)R
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
        # self.visualizer.plotArm(self.targetThetaPose, True)
        return True


if __name__ == "__main__":
    arm = RobotArm(False)
