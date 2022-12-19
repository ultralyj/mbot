#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("vel_publisher")

    vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    vel = Twist()
    vel.linear.x = 0.15
    vel.angular.z = 0.3

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing...")
        vel_pub.publish(vel)
        rate.sleep()


if __name__ == "__main__":
    main()
