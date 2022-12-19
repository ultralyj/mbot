#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Tracking:
    def __init__(self, duration, controller="lpj"):
        controllers = ["lpj", "gzy"]
        assert controller in controllers, "Controller not defined."
        self.controller = controller

        self.duration = duration
        self.N = duration * 10

        # lpj control parameters
        self.k1 = 0.3
        self.k2 = 0.2
        self.k3 = 0.3

        # gzy control parameters
        self.kk = 0.5
        self.l = 0.2

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom_sub = rospy.Subscriber("/tb3_0/odom", Odometry, self.odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "tb3_0/cmd_vel", Twist, queue_size=1
        )

        self.x_d, self.y_d, self.theta_d, self.v_d, self.w_d = self.genTraj()
        xList = []
        yList = []

        k = 0
        rate = rospy.Rate(10.0)
        while k < self.N:
            if self.controller == "lpj":
                v, w = self.lpj_tracking(
                    self.x,
                    self.y,
                    self.theta,
                    self.x_d[k],
                    self.y_d[k],
                    self.theta_d[k],
                    self.v_d[k],
                    self.w_d[k],
                )
            elif self.controller == "gzy":
                v, w = self.gzy_tracking(
                    self.x, self.y, self.theta, self.x_d[k], self.y_d[k]
                )

            vel = Twist()
            vel.linear.x = v
            vel.angular.z = w
            self.vel_pub.publish(vel)

            xList.append(self.x)
            yList.append(self.y)

            k += 1
            rate.sleep()
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        plt.scatter(self.x_d, self.y_d, c="r")
        plt.scatter(xList, yList, c="b")
        plt.show()

    def genTraj(self):
        t = np.linspace(0, self.duration, num=self.N)
        w = 0.2 * np.ones(t.shape)
        r = 1.0
        v = w * r
        x, y = r * np.cos(w * t), r * np.sin(w * t)
        theta = math.pi / 2 + w * t

        return x, y, theta, v, w

    def lpj_tracking(self, x, y, theta, x_d, y_d, theta_d, v_d, w_d):
        e_x = x_d - x
        e_y = y_d - y

        x_e = e_x * math.cos(theta) + e_y * math.sin(theta)
        y_e = -e_x * math.sin(theta) + e_y * math.cos(theta)

        theta_e = theta_d - theta

        v = v_d * math.cos(theta_e) + self.k2 * x_e
        w = w_d + self.k1 * v_d * y_e + self.k3 * math.sin(theta_e)

        return v, w

    def gzy_tracking(self, x, y, theta, x_d, y_d):
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

    def odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x = posistion.x
        self.y = posistion.y

        _, _, self.theta = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        info = "(self.x, self.y, theta) = ({}, {}, {})".format(
            self.x, self.y, self.theta
        )
        rospy.loginfo(info)

    pass


def main(args):
    rospy.init_node("tracking_demo")

    tracking = Tracking(20, "gzy")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
