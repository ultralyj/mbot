#!/usr/bin/env python
#coding=utf-8

import rospy

import math
import tf
import geometry_msgs.msg
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist


def gzy_stabilization_2(x, y, theta, x_d, y_d):
    kk = 0.2
    l = 0.2
    e_x = x_d - x
    e_y = y_d - y
    u_x = kk * e_x
    u_y = kk * e_y
    A = np.array(
        [
            [np.cos(theta), -l * np.sin(theta)],
            [np.sin(theta), l * np.cos(theta)],
        ]
    )
    U = np.array([[u_x], [u_y]])
    v_w = np.linalg.solve(A, U)
    v = v_w[0]
    w = v_w[1]

    return v, w

if __name__ == '__main__':
    rospy.init_node('item')

    listener = tf.TransformListener() #TransformListener创建后就开始接受tf广播信息，最多可以缓存10s

    '''
    #设置robot2的初始坐标
    robot2_start = rospy.Publisher('robot2/odom', nav_msgs/Odometry, queue_size=1)
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.z = 0
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = 0
    msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 0
    robot2_start.publish(msg) #将请求的参数传入  robot2的初始位置
    '''
    robot1name = rospy.get_param('~robot1')    #取参数服务器robot的值
    robot2name = rospy.get_param('~robot2')    #取参数服务器robot的值
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    turtle_vel = rospy.Publisher('/%s/cmd_vel' % robot2name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0) #循环执行，更新频率是10hz
    while not rospy.is_shutdown():
        try:
            #得到以robot2为坐标原点的robot1的姿态信息(平移和旋转)
            (trans, rot) = listener.lookupTransform('/%s/odom' % robot2name, '/%s/odom' % robot1name, rospy.Time()) #查看相对的tf,返回平移和旋转  turtle2跟着turtle1变换
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        x_d = trans[0]
        y_d = trans[1]
        v, w = gzy_stabilization_2(0,0,0,x_d,y_d)

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        
        distance = np.sqrt(x_d*x_d +y_d*y_d)
        if(distance>0.2):
            vel.linear.x = v
            vel.angular.z = w
        else:
            vel.linear.x = 0
            vel.angular.z = 0
        turtle_vel.publish(vel) #向/robot2/cmd_vel话题发布新坐标  (即robot2根据/robot2/cmd_vel的数据来控制robot2移动)
        rate.sleep() #以固定频率执行
        