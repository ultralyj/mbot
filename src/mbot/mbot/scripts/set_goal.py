#! /usr/bin/env python2
# # -*- coding: utf-8 -*-
# from numpy.core.numeric import moveaxis
# import rospy
# from actionlib_msgs.msg import *
# from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
# from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
# import copy
# import actionlib

# test1 = Pose(Point(-1.5, -1.5, 0.000), Quaternion(0.000, 0.000, -0.752, 0.659))

# def goto_point(point, pub):
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = "map"
#     goal_pose.pose = point
#     pub.publish(goal_pose)

# def goto_multi_points(center, pub):
#     step2 = 0.15
#     temp = copy.deepcopy(center)
#     temp.position.x-=step2
#     temp.position.y-=step2
#     goal1 = temp

#     temp = copy.deepcopy(center)
#     temp.position.x-=step2
#     temp.position.y+=step2
#     goal2 = temp

#     temp = copy.deepcopy(center)
#     temp.position.x+=step2
#     temp.position.y-=step2
#     goal3 = temp

#     temp = copy.deepcopy(center)
#     temp.position.x+=step2
#     temp.position.y+=step2
#     goal4 = temp

#     rospy.loginfo(goal1)
#     rospy.loginfo(goal2)
#     rospy.loginfo(goal3)
#     rospy.loginfo(goal4)
#     goto_point(goal1,pub[0])
#     goto_point(goal2,pub[1])
#     goto_point(goal3,pub[2])
#     goto_point(goal4,pub[3])

# if __name__ == "__main__":
#     rospy.init_node("multi_goal_publisher")

#     # move_base的初始化
#     move_base0 = actionlib.SimpleActionClient("tb3_0/move_base", MoveBaseAction)
#     move_base1 = actionlib.SimpleActionClient("tb3_1/move_base", MoveBaseAction)
#     move_base2 = actionlib.SimpleActionClient("tb3_2/move_base", MoveBaseAction)
#     move_base3 = actionlib.SimpleActionClient("tb3_3/move_base", MoveBaseAction)

#     # 发布/move_base_simple/goal，给小车指定下一个坐标点。
#     goal_pub0 = rospy.Publisher('tb3_0/move_base_simple/goal', PoseStamped, queue_size=1)
#     goal_pub1 = rospy.Publisher('tb3_1/move_base_simple/goal', PoseStamped, queue_size=1)
#     goal_pub2 = rospy.Publisher('tb3_2/move_base_simple/goal', PoseStamped, queue_size=1)
#     goal_pub3 = rospy.Publisher('tb3_3/move_base_simple/goal', PoseStamped, queue_size=1)

    
#     pubs = [goal_pub0, goal_pub1, goal_pub2, goal_pub3]
#     # publish the goals
#     goto_multi_points(test1, pubs)
#     rospy.spin()
    
#! /usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('multi_goal_pub')
    turtle_vel_pub1 = rospy.Publisher('tb3_0/move_base_simple/goal', PoseStamped, queue_size=1)
    turtle_vel_pub2 = rospy.Publisher('tb3_1/move_base_simple/goal', PoseStamped, queue_size=1)
    turtle_vel_pub3 = rospy.Publisher('tb3_2/move_base_simple/goal', PoseStamped, queue_size=1)
    turtle_vel_pub4 = rospy.Publisher('tb3_3/move_base_simple/goal', PoseStamped, queue_size=1)
    mypose=PoseStamped()
    turtle_vel_pub1.publish(mypose) #先发送一个空位置，试探一下，否则第一个包容易丢
    turtle_vel_pub2.publish(mypose)
    turtle_vel_pub3.publish(mypose)
    turtle_vel_pub4.publish(mypose)

    time.sleep(1)
    
    mypose=PoseStamped()
    mypose.header.frame_id='map' #设置自己的目标
    mypose.pose.position.x=-1.4
    mypose.pose.position.y=-1.4
    mypose.pose.position.z=0
    mypose.pose.orientation.x=0
    mypose.pose.orientation.y=0
    mypose.pose.orientation.z=1
    mypose.pose.orientation.w=0
    
    turtle_vel_pub1.publish(mypose) #发送自己设置的目标点
    mypose.pose.position.y=1.4
    turtle_vel_pub2.publish(mypose) #发送自己设置的目标点
    mypose.pose.position.x=1.4
    turtle_vel_pub4.publish(mypose) #发送自己设置的目标点
    mypose.pose.position.x= 1.4
    mypose.pose.position.y= -1.4
    turtle_vel_pub3.publish(mypose) #发送自己设置的目标点
    time.sleep(5)
    
    