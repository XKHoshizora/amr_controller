#!/usr/bin/env python3
#coding=utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class AMRController:
    def __init__(self):
        rospy.init_node('amr_controller_node')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(20)  # 20Hz
        self.twist = Twist()
        self.has_input = False  # 添加标志位来追踪是否有输入

    def joy_callback(self, joy_msg):
        # 检查是否有摇杆输入
        if abs(joy_msg.axes[0]) > 0.1 or abs(joy_msg.axes[1]) > 0.1 or abs(joy_msg.axes[3]) > 0.1:
            # 左摇杆控制线速度
            self.twist.linear.x = joy_msg.axes[1] * 2.0  # 向前/后
            self.twist.linear.y = joy_msg.axes[0] * 2.0  # 左/右

            # 右摇杆控制角速度
            self.twist.angular.z = joy_msg.axes[3] * 6.283186  # 旋转

            self.has_input = True
        else:
            # 没有输入时，将所有速度置为0
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0
            self.has_input = False

    def run(self):
        while not rospy.is_shutdown():
            # 只在有输入时发布消息
            if self.has_input:
                self.pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AMRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass