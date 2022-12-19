#!/usr/bin/env python
import rospy
import sys
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np
import time
import matplotlib.pyplot as plt 
class Stabilization:
    def __init__(self, x_d=1.0, y_d=1.0):
        # gzy2 control parameters
        self.kk = 0.2
        self.l = 0.2

        self.x_d = x_d
        self.y_d = y_d

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=1
        )
        self.xList = []
        self.yList = []
        self.eList = []

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def setPosition(self, x, y):
        self.x_d = x
        self.y_d = y



    def gzy_stabilization_2(self, x, y, theta, x_d, y_d):
        e_x = x_d - x
        e_y = y_d - y
        u_x = self.kk * e_x
        u_y = self.kk * e_y
        A = np.array(
            [
                [np.cos(theta), -self.l * np.sin(theta)],
                [np.sin(theta), self.l * np.cos(theta)],
            ]
        )
        U = np.array([[u_x], [u_y]])
        v_w = np.linalg.solve(A, U)
        v = v_w[0]
        w = v_w[1]

        

        return v, w

  
    def timer_cb(self, data):
        self.xList.append(self.x)
        self.yList.append(self.y)
        v, w = self.gzy_stabilization_2(self.x, self.y, self.theta, self.x_d, self.y_d)
        e_x = self.x_d - self.x
        e_y = self.y_d - self.y
        e = math.sqrt(e_x*e_x+e_y*e_y)
        self.eList.append(e)
        if (math.sqrt(e_x*e_x+e_y*e_y)<0.1):
            vel = Twist()
            vel.linear.x = 0
            vel.angular.z = 0
            
            self.vel_pub.publish(vel)
            plt.subplot(1,2,1)
            plt.scatter(x=self.xList,y=self.yList,s=1)
            plt.title("robot path")
            plt.subplot(1,2,2)
            plt.plot(self.eList)
            plt.title("control error")
            plt.show()
            exit()
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        self.vel_pub.publish(vel)

    def odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x = posistion.x
        self.y = posistion.y

        
        _, _, self.theta = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        # info = "(self.x, self.y, theta) = ({}, {}, {})".format(
        #     self.x, self.y, self.theta
        # )
        # rospy.loginfo(info)

    pass


def main(args):
    rospy.init_node("stabilization_demo")

    stabilization = Stabilization(0.0, 0.0)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
