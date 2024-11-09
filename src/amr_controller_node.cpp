#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class AMRController {
public:
    AMRController(ros::NodeHandle& nh) : nh_(nh) {
        // 获取调试模式参数
        nh_.param<bool>("debug_mode", debug_mode_, false);

        // 初始化发布器和订阅器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        joy_sub_ = nh_.subscribe("/joy", 10, &AMRController::joyCallback, this);

        // 仅在调试模式下创建调试发布器
        if (debug_mode_) {
            debug_pub_ = nh_.advertise<sensor_msgs::Joy>("/joy_debug", 10);
            ROS_INFO("AMR Controller running in debug mode");
        } else {
            ROS_INFO("AMR Controller running in normal mode");
        }

        // 初始化上一次的轴状态
        last_axes_[0] = 0.0;
        last_axes_[1] = 0.0;
        last_axes_[2] = 0.0;
        need_publish_ = false;

        // 控制参数
        deadzone_ = 0.005;
        max_linear_vel_ = 2.0;
        max_angular_vel_ = 6.283186;
        expo_factor_ = 2.0;

        // 仅在调试模式下初始化噪声统计
        if (debug_mode_) {
            noise_count_ = 0;
            for (int i = 0; i < 3; ++i) {
                noise_sum_[i] = 0.0;
                noise_max_[i] = 0.0;
            }
        }
    }

    double applyDeadzone(double value, double deadzone) {
        if (std::abs(value) < deadzone) {
            return 0.0;
        }
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
    }

    double applyExpo(double value, double expo) {
        return std::copysign(std::pow(std::abs(value), expo), value);
    }

    double mapSpeed(double value, double max_speed) {
        value = applyDeadzone(value, deadzone_);
        value = applyExpo(value, expo_factor_);
        return value * max_speed;
    }

    void updateNoiseStats(const double axes[3]) {
        if (!debug_mode_) return;

        for (int i = 0; i < 3; ++i) {
            if (std::abs(axes[i]) < 0.1) {
                noise_count_++;
                noise_sum_[i] += std::abs(axes[i]);
                noise_max_[i] = std::max(noise_max_[i], std::abs(axes[i]));
            }
        }

        if (noise_count_ >= 1000) {
            for (int i = 0; i < 3; ++i) {
                double avg = noise_sum_[i] / noise_count_;
                ROS_INFO_STREAM("Axis " << i << " noise - Avg: " << avg
                              << ", Max: " << noise_max_[i]);
            }
            noise_count_ = 0;
            for (int i = 0; i < 3; ++i) {
                noise_sum_[i] = 0.0;
                noise_max_[i] = 0.0;
            }
        }
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        geometry_msgs::Twist twist;
        bool current_input = false;

        double x_input = joy_msg->axes[1];
        double y_input = joy_msg->axes[0];
        double rot_input = joy_msg->axes[3];

        // 仅在调试模式下更新噪声统计
        if (debug_mode_) {
            double current_axes[3] = {y_input, x_input, rot_input};
            updateNoiseStats(current_axes);
        }

        if (std::abs(x_input) > deadzone_ ||
            std::abs(y_input) > deadzone_ ||
            std::abs(rot_input) > deadzone_) {

            twist.linear.x = mapSpeed(x_input, max_linear_vel_);
            twist.linear.y = mapSpeed(y_input, max_linear_vel_);
            twist.angular.z = mapSpeed(rot_input, max_angular_vel_);
            current_input = true;

            // 仅在调试模式下打印日志
            if (debug_mode_ && std::max({std::abs(x_input), std::abs(y_input), std::abs(rot_input)}) > 0.05) {
                ROS_INFO_STREAM("Input: " << x_input << ", " << y_input << ", " << rot_input);
                ROS_INFO_STREAM("Speed: " << twist.linear.x << ", " << twist.linear.y
                            << ", " << twist.angular.z);
            }
        } else {
            // 摇杆回中，立即发布停止命令
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;
            cmd_vel_pub_.publish(twist);  // 直接发布停止命令
            need_publish_ = false;        // 设置标志位为false
        }

        twist_ = twist;
        need_publish_ = need_publish_ || current_input;

        last_axes_[0] = y_input;
        last_axes_[1] = x_input;
        last_axes_[2] = rot_input;

        // 仅在调试模式下发布调试信息
        if (debug_mode_ && debug_pub_) {
            sensor_msgs::Joy debug_msg;
            debug_msg.header.stamp = ros::Time::now();
            debug_msg.axes = {static_cast<float>(x_input), static_cast<float>(y_input), static_cast<float>(rot_input)};
            debug_pub_.publish(debug_msg);
        }
    }

    void run() {
        ros::Rate rate(20);

        while (ros::ok()) {
            if (need_publish_) {
                cmd_vel_pub_.publish(twist_);
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
    ros::Publisher debug_pub_;
    ros::Subscriber joy_sub_;

    bool debug_mode_;
    geometry_msgs::Twist twist_;
    bool need_publish_;
    double last_axes_[3];

    // 控制参数
    double deadzone_;
    double max_linear_vel_;
    double max_angular_vel_;
    double expo_factor_;

    // 仅在调试模式下使用的成员变量
    int noise_count_;
    double noise_sum_[3];
    double noise_max_[3];
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_controller_node");
    ros::NodeHandle nh("~");  // 使用私有命名空间以访问私有参数

    AMRController controller(nh);
    controller.run();

    return 0;
}