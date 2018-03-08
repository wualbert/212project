#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

class ArmVisualizer:
    def __init__(self, l1=0.3556, l2=0.2794, l3=0.1143, origin = (0,0)):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.origin = origin
        self.targetPose = None
        self.armPose = ((self.l1,0),(self.l1+self.l2,0),(self.l1+self.l2+self.l3,0))
    def plotArm(self, pose, clear = True):
        elbowPose = (self.origin[0] + self.l1 * np.cos(pose[0]), self.origin[1] + self.l1 * np.sin(pose[0]))
        wristPose =(self.origin[0] + self.l1 * np.cos(pose[0]) + self.l2 * np.cos(pose[0] + pose[1]),
                    self.origin[1] + self.l1 * np.sin(pose[0]) + self.l2 * np.sin(pose[0] + pose[1]))
        endPose =(self.origin[0] + self.l1 * np.cos(pose[0]) + self.l2 * np.cos(pose[0] + pose[1]) + self.l3 * np.cos(pose[0] + pose[1] + pose[2]),
                  self.origin[1] + self.l1 * np.sin(pose[0]) + self.l2 * np.sin(pose[0] + pose[1]) + self.l3 * np.sin(pose[0] + pose[1] + pose[2]))
        armPose = [self.origin,elbowPose,wristPose,endPose]
        self.armPose = armPose
        self.plot()

    def plot(self, clear = True):
        if clear:
            plt.clf()

        plt.axis([-0.5, 1,-.75,.75])
        plotPoints = [self.origin]+list(self.armPose)
        plt.plot(*zip(*plotPoints), color = 'b', linewidth = 3.5, marker = 'o', ms = 7)
        plt.plot(*zip(self.origin), color = 'r', linewidth = 3, marker = 'o', ms = 10)
        if self.targetPose:
            plt.plot(*zip(self.targetPose), color='g', linewidth=3, marker='o', ms=8)
        plt.show()

    def plotTarget(self, coords):
        self.targetPose = coords
        self.plot(False)

if __name__=="__main__":
    visualizer = ArmVisualizer()
    visualizer.plotArm((0, 0, 0))
