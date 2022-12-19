#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def odom_cb(data):
    posistion = data.pose.pose.position
    oriention = data.pose.pose.orientation

    x = posistion.x
    y = posistion.y

    _, _, theta = euler_from_quaternion(
        [oriention.x, oriention.y, oriention.z, oriention.w]
    )

    info = "(x, y, theta) = ({}, {}, {})".format(x, y, theta)
    rospy.loginfo(info)


def main():
    rospy.init_node("odom_listener")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
