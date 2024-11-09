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

        # 添加诊断信息的发布器
        self.debug_pub = rospy.Publisher('/joy_debug', Joy, queue_size=10)

        self.rate = rospy.Rate(20)  # 20Hz
        self.twist = Twist()
        self.need_publish = False

        # 初始化上一次的轴状态
        self.last_axes = [0.0] * 3

        # 控制参数
        self.deadzone = 0.005  # 使用极小的死区
        self.max_linear_vel = 2.0
        self.max_angular_vel = 6.283186
        self.expo_factor = 2.0

        # 添加噪声统计
        self.noise_stats = {'count': 0, 'sum': [0.0] * 3, 'max': [0.0] * 3}

    def apply_deadzone(self, value, deadzone):
        """应用死区，并将死区外的值重新映射到完整范围"""
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    def apply_expo(self, value, expo):
        """应用指数曲线来实现更精确的低速控制"""
        return math.copysign(abs(value) ** expo, value)

    def map_speed(self, value, max_speed):
        """将输入值映射到速度，应用死区和指数映射"""
        value = self.apply_deadzone(value, self.deadzone)
        value = self.apply_expo(value, self.expo_factor)
        return value * max_speed

    def update_noise_stats(self, axes):
        """更新噪声统计信息"""
        for i in range(3):
            if abs(axes[i]) < 0.1:  # 只统计小信号
                self.noise_stats['count'] += 1
                self.noise_stats['sum'][i] += abs(axes[i])
                self.noise_stats['max'][i] = max(self.noise_stats['max'][i], abs(axes[i]))

        # 每1000个样本打印一次统计信息
        if self.noise_stats['count'] >= 1000:
            for i in range(3):
                avg = self.noise_stats['sum'][i] / self.noise_stats['count']
                rospy.loginfo(f"Axis {i} noise - Avg: {avg:.6f}, Max: {self.noise_stats['max'][i]:.6f}")
            self.noise_stats = {'count': 0, 'sum': [0.0] * 3, 'max': [0.0] * 3}

    def joy_callback(self, joy_msg):
        current_input = False

        # 获取摇杆输入
        x_input = joy_msg.axes[1]  # 前后
        y_input = joy_msg.axes[0]  # 左右
        rot_input = joy_msg.axes[3]  # 旋转

        # 更新噪声统计
        self.update_noise_stats([y_input, x_input, rot_input])

        # 检查是否有摇杆输入（使用极小的死区）
        if (abs(x_input) > self.deadzone or
            abs(y_input) > self.deadzone or
            abs(rot_input) > self.deadzone):

            # 映射到速度命令
            self.twist.linear.x = self.map_speed(x_input, self.max_linear_vel)
            self.twist.linear.y = self.map_speed(y_input, self.max_linear_vel)
            self.twist.angular.z = self.map_speed(rot_input, self.max_angular_vel)

            current_input = True

            # 打印原始输入和映射后的速度（仅当有明显输入时）
            if max(abs(x_input), abs(y_input), abs(rot_input)) > 0.05:
                rospy.loginfo(f"Input: {x_input:.4f}, {y_input:.4f}, {rot_input:.4f}")
                rospy.loginfo(f"Speed: {self.twist.linear.x:.4f}, {self.twist.linear.y:.4f}, {self.twist.angular.z:.4f}")

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

        # 发布调试信息
        debug_msg = Joy()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.axes = [x_input, y_input, rot_input]
        self.debug_pub.publish(debug_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.need_publish:
                self.pub.publish(self.twist)
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