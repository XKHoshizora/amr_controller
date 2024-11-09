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
        self.need_publish = False

        # 初始化上一次的轴状态
        self.last_axes = [0.0] * 3  # 存储axes[0], axes[1], axes[3]的上一次状态

    def joy_callback(self, joy_msg):
        current_input = False

        # 检查是否有摇杆输入
        if abs(joy_msg.axes[0]) > 0.1 or abs(joy_msg.axes[1]) > 0.1 or abs(joy_msg.axes[3]) > 0.1:
            # 左摇杆控制线速度
            self.twist.linear.x = joy_msg.axes[1] * 2.0  # 向前/后
            self.twist.linear.y = joy_msg.axes[0] * 2.0  # 左/右

            # 右摇杆控制角速度
            self.twist.angular.z = joy_msg.axes[3] * 6.283186  # 旋转

            current_input = True
        else:
            # 没有输入时，将所有速度置为0
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0

            # 检查是否是从有输入状态变为无输入状态
            if (abs(self.last_axes[0]) > 0.1 or
                abs(self.last_axes[1]) > 0.1 or
                abs(self.last_axes[2]) > 0.1):
                # 摇杆刚回正，需要发送一次零速度
                self.need_publish = True

        # 更新状态
        self.need_publish = self.need_publish or current_input

        # 保存当前轴状态用于下次比较
        self.last_axes[0] = joy_msg.axes[0]
        self.last_axes[1] = joy_msg.axes[1]
        self.last_axes[2] = joy_msg.axes[3]  # 注意这里是axes[3]

    def run(self):
        while not rospy.is_shutdown():
            # 当需要发布时才发送消息
            if self.need_publish:
                self.pub.publish(self.twist)
                # 如果速度都为0，发送后就不需要继续发送了
                if (self.twist.linear.x == 0.0 and
                    self.twist.linear.y == 0.0 and
                    self.twist.angular.z == 0.0):
                    self.need_publish = False

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AMRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass