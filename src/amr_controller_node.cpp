#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class AMRController {
public:
    AMRController(ros::NodeHandle& nh) : nh_(nh) {
        // 初始化发布器和订阅器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        joy_sub_ = nh_.subscribe("/joy", 10, &AMRController::joyCallback, this);

        has_input_ = false;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        // 检查是否有摇杆输入
        if (std::abs(joy_msg->axes[0]) > 0.1 ||
            std::abs(joy_msg->axes[1]) > 0.1 ||
            std::abs(joy_msg->axes[3]) > 0.1) {

            geometry_msgs::Twist twist;
            // 左摇杆控制线速度
            twist.linear.x = joy_msg->axes[1] * 2.0;  // 向前/后
            twist.linear.y = joy_msg->axes[0] * 2.0;  // 左/右

            // 右摇杆控制角速度
            twist.angular.z = joy_msg->axes[3] * 6.283186;  // 旋转

            twist_ = twist;
            has_input_ = true;
        } else {
            // 没有输入时，将所有速度置为0
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;

            twist_ = twist;
            has_input_ = false;
        }
    }

    void run() {
        ros::Rate rate(20);  // 20Hz

        while (ros::ok()) {
            if (has_input_) {
                cmd_vel_pub_.publish(twist_);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber joy_sub_;

    geometry_msgs::Twist twist_;
    bool has_input_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_controller_node");
    ros::NodeHandle nh;

    AMRController controller(nh);
    controller.run();

    return 0;
}