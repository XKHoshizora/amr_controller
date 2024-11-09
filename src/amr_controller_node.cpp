#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class AMRController {
public:
    AMRController(ros::NodeHandle& nh) : nh_(nh) {
        // 初始化发布器和订阅器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        joy_sub_ = nh_.subscribe("/joy", 10, &AMRController::joyCallback, this);

        // 初始化上一次的轴状态
        last_axes_[0] = 0.0;
        last_axes_[1] = 0.0;
        last_axes_[3] = 0.0;
        need_publish_ = false;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        geometry_msgs::Twist twist;
        bool current_input = false;

        // 检查是否有摇杆输入
        if (std::abs(joy_msg->axes[0]) > 0.1 ||
            std::abs(joy_msg->axes[1]) > 0.1 ||
            std::abs(joy_msg->axes[3]) > 0.1) {

            // 有效输入，设置速度
            twist.linear.x = joy_msg->axes[1] * 2.0;  // 向前/后
            twist.linear.y = joy_msg->axes[0] * 2.0;  // 左/右
            twist.angular.z = joy_msg->axes[3] * 6.283186;  // 旋转
            current_input = true;

        } else {
            // 无输入，速度置零
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;

            // 检查是否是从有输入状态变为无输入状态
            if (std::abs(last_axes_[0]) > 0.1 ||
                std::abs(last_axes_[1]) > 0.1 ||
                std::abs(last_axes_[3]) > 0.1) {
                // 摇杆刚回正，需要发送一次零速度
                need_publish_ = true;
            }
        }

        // 更新状态
        twist_ = twist;
        need_publish_ = need_publish_ || current_input;

        // 保存当前轴状态用于下次比较
        last_axes_[0] = joy_msg->axes[0];
        last_axes_[1] = joy_msg->axes[1];
        last_axes_[3] = joy_msg->axes[3];
    }

    void run() {
        ros::Rate rate(20);  // 20Hz

        while (ros::ok()) {
            if (need_publish_) {
                cmd_vel_pub_.publish(twist_);
                // 如果速度都为0，发送后就不需要继续发送了
                if (twist_.linear.x == 0.0 &&
                    twist_.linear.y == 0.0 &&
                    twist_.angular.z == 0.0) {
                    need_publish_ = false;
                }
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
    bool need_publish_;
    double last_axes_[3];  // 存储上一次的轴状态
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_controller_node");
    ros::NodeHandle nh;

    AMRController controller(nh);
    controller.run();

    return 0;
}