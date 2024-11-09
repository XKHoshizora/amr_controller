#!/usr/bin/env python3
#coding=utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

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

        # 控制参数
        self.deadzone = 0.02  # 降低死区阈值
        self.max_linear_vel = 2.0  # 最大线速度 (m/s)
        self.max_angular_vel = 6.283186  # 最大角速度 (rad/s)

        # 指数映射的参数
        self.expo_factor = 2.0  # 指数曲线的幂次，用于调整灵敏度

    def apply_deadzone(self, value, deadzone):
        """应用死区，并将死区外的值重新映射到完整范围"""
        if abs(value) < deadzone:
            return 0.0
        # 将死区外的值重新映射到 [-1, 1] 范围
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    def apply_expo(self, value, expo):
        """应用指数曲线来实现更精确的低速控制
        在中间区域提供更高的分辨率，而在极限位置保持全速"""
        return math.copysign(abs(value) ** expo, value)

    def map_speed(self, value, max_speed):
        """将输入值映射到速度，应用死区和指数映射"""
        # 应用死区
        value = self.apply_deadzone(value, self.deadzone)
        # 应用指数映射获得更好的低速控制
        value = self.apply_expo(value, self.expo_factor)
        # 映射到最终速度
        return value * max_speed

    def joy_callback(self, joy_msg):
        current_input = False

        # 获取摇杆输入
        x_input = joy_msg.axes[1]  # 前后
        y_input = joy_msg.axes[0]  # 左右
        rot_input = joy_msg.axes[3]  # 旋转

        # 检查是否有摇杆输入（使用较小的死区）
        if (abs(x_input) > self.deadzone or
            abs(y_input) > self.deadzone or
            abs(rot_input) > self.deadzone):

            # 映射到速度命令
            self.twist.linear.x = self.map_speed(x_input, self.max_linear_vel)
            self.twist.linear.y = self.map_speed(y_input, self.max_linear_vel)
            self.twist.angular.z = self.map_speed(rot_input, self.max_angular_vel)

            current_input = True
        else:
            # 没有输入时，将所有速度置为0
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.angular.z = 0.0

            # 检查是否是从有输入状态变为无输入状态
            if (abs(self.last_axes[0]) > self.deadzone or
                abs(self.last_axes[1]) > self.deadzone or
                abs(self.last_axes[2]) > self.deadzone):
                self.need_publish = True

        # 更新状态
        self.need_publish = self.need_publish or current_input

        # 保存当前轴状态用于下次比较
        self.last_axes[0] = y_input
        self.last_axes[1] = x_input
        self.last_axes[2] = rot_input

    def run(self):
        while not rospy.is_shutdown():
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