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
        
        self.rate = rospy.Rate(10)  # 10Hz
        self.twist = Twist()

    def joy_callback(self, joy_msg):
        # 左摇杆控制线速度
        self.twist.linear.x = joy_msg.axes[1] * 2.0  # 向前/后
        self.twist.linear.y = joy_msg.axes[0] * 2.0  # 左/右
        
        # 右摇杆控制角速度
        self.twist.angular.z = joy_msg.axes[3] * 6.2  # 旋转

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AMRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
