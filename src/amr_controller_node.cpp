#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class AMRController {
public:
    AMRController() : rate(10) {  // 10Hz
        vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &AMRController::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        // 左摇杆控制线速度
        vel_msg.linear.x = joy_msg->axes[1] * 2.0;  // 向前/后
        vel_msg.linear.y = joy_msg->axes[0] * 2.0;  // 左/右
        
        // 右摇杆控制角速度
        vel_msg.angular.z = joy_msg->axes[3] * 6.283186;  // 旋转
    }

    void run() {
        while (ros::ok()) {
            vel_pub.publish(vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Rate rate;
    geometry_msgs::Twist vel_msg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_controller_node");
    
    try {
        AMRController controller;
        controller.run();
    }
    catch (ros::Exception& e) {
        ROS_ERROR("Caught exception: %s", e.what());
        return 1;
    }

    return 0;
}