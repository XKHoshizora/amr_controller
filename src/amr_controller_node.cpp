#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

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

        // 控制参数
        deadzone_ = 0.02;  // 降低死区阈值
        max_linear_vel_ = 2.0;  // 最大线速度 (m/s)
        max_angular_vel_ = 6.283186;  // 最大角速度 (rad/s)
        expo_factor_ = 2.0;  // 指数曲线的幂次，用于调整灵敏度
    }

    // 应用死区，并将死区外的值重新映射到完整范围
    double applyDeadzone(double value, double deadzone) {
        if (std::abs(value) < deadzone) {
            return 0.0;
        }
        // 将死区外的值重新映射到 [-1, 1] 范围
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
    }

    // 应用指数曲线来实现更精确的低速控制
    double applyExpo(double value, double expo) {
        return std::copysign(std::pow(std::abs(value), expo), value);
    }

    // 将输入值映射到速度，应用死区和指数映射
    double mapSpeed(double value, double max_speed) {
        // 应用死区
        value = applyDeadzone(value, deadzone_);
        // 应用指数映射获得更好的低速控制
        value = applyExpo(value, expo_factor_);
        // 映射到最终速度
        return value * max_speed;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
        geometry_msgs::Twist twist;
        bool current_input = false;

        // 获取摇杆输入
        double x_input = joy_msg->axes[1];  // 前后
        double y_input = joy_msg->axes[0];  // 左右
        double rot_input = joy_msg->axes[3];  // 旋转

        // 检查是否有摇杆输入
        if (std::abs(x_input) > deadzone_ ||
            std::abs(y_input) > deadzone_ ||
            std::abs(rot_input) > deadzone_) {

            // 映射到速度命令
            twist.linear.x = mapSpeed(x_input, max_linear_vel_);
            twist.linear.y = mapSpeed(y_input, max_linear_vel_);
            twist.angular.z = mapSpeed(rot_input, max_angular_vel_);
            current_input = true;

        } else {
            // 无输入，速度置零
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.angular.z = 0.0;

            // 检查是否是从有输入状态变为无输入状态
            if (std::abs(last_axes_[0]) > deadzone_ ||
                std::abs(last_axes_[1]) > deadzone_ ||
                std::abs(last_axes_[3]) > deadzone_) {
                // 摇杆刚回正，需要发送一次零速度
                need_publish_ = true;
            }
        }

        // 更新状态
        twist_ = twist;
        need_publish_ = need_publish_ || current_input;

        // 保存当前轴状态用于下次比较
        last_axes_[0] = y_input;
        last_axes_[1] = x_input;
        last_axes_[2] = rot_input;
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

    // 控制参数
    double deadzone_;      // 死区阈值
    double max_linear_vel_;   // 最大线速度
    double max_angular_vel_;  // 最大角速度
    double expo_factor_;      // 指数映射因子
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "amr_controller_node");
    ros::NodeHandle nh;

    AMRController controller(nh);
    controller.run();

    return 0;
}