#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <vector>
#include <string>
#include <bits/stdc++.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class TurtlBot3 : public rclcpp::Node
{
public:
    TurtlBot3()
        : Node("minmal_publsiher"), count_(0)
    {
        subscription_1 = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TurtlBot3::topic_callback, this, _1));
        subscription_2 = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TurtlBot3::obs_avoid, this, _1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtlBot3::timer_callback, this));
        TurtlBot3::set_goal();
    }
    vector<float> scan;

private:
    float x_self, y_self, x_goal, y_goal, theta_self;
    long double x, obj_dis, sum, m, n;
    void topic_callback(nav_msgs::msg::Odometry msg)
    {
        x_self = msg.pose.pose.position.x;
        y_self = msg.pose.pose.position.y;
        theta_self = quatToEuler(msg.pose.pose.orientation);
    }

    void obs_avoid(const sensor_msgs::msg::LaserScan &msg)
    {
        long double i = 0, k = 0;
        sum = 0;
        for (long double x : msg.ranges) // LiDAR data
        {
            if (x > 1.7)
            {
                x = 1.7;
            }
            obj_dis = 1 - x / 1.7;
            scan.push_back(obj_dis);
            if (i<41 || i>319)
            {
                if(i<41)
                {
                    k = i;
                }
                else
                {
                    k=i-359;
                }
                sum -= obj_dis*k;
            }
            i++;
        }
        scan.clear();
    }
    void set_goal()
    {
        cout << "Enter x coord:" << endl;
        cin >> x_goal;
        cout << "Enter y coord:" << endl;
        cin >> y_goal;
    }

    double quatToEuler(const geometry_msgs::msg::Quaternion &quaternion)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    void timer_callback()
    {
        double g_dis, angle, vel = 0, angle_diff = 0, k_lin, k_ang, obs_lin, obs_ang, goal_ang;
        auto message = geometry_msgs::msg::Twist();
        g_dis = sqrt(pow(x_goal - x_self, 2) + pow(y_goal - y_self, 2));
        if (g_dis > 0.01)
        {
            vel = 0.2 * g_dis;
            angle = atan2(y_goal - y_self, x_goal - x_self) - theta_self;
            if (angle > 3.14)
            {
                angle = angle - 6.28;
            }
            else if (angle < -3.14)
            {
                angle = angle + 6.28; // angle for avoiding spiral
            }
            if (g_dis > 0.1 and g_dis < 4)
            {
                angle_diff = (2 * angle / g_dis) + (0.2 * angle * g_dis); // insantaneous velocity change when near a goal
            }
            else
            {
                angle_diff = 0.45 * angle; // angle differnce of the current posn and goal posn
            }
            obs_lin = (exp(this->obj_dis) - 1.7);
            if (vel > 0.5)
            {
                vel = 0.5;
            }
            obs_ang = sum / 75;
            // cout << obs_ang << " ";
            //  cout << obs_ang << " ";
            //  cout << k_lin*vel << " ";
            goal_ang = exp(1) - exp(obs_ang); // calculator for goal angular velocity
            // cout << goal_ang << " ";
            k_lin = 1.26 - exp(obs_lin);                          // calculator for linear velocity due to obstacle
            k_ang = 0.125 * goal_ang * angle_diff + 8 * obs_ang; // total angular veloctiy
            cout << k_ang << " ";
            message.linear.x = 1.8 * k_lin * vel; // adjusting linear velocity
            message.angular.z = k_ang;
            if (abs(message.angular.z) > 0.5)
            {
                message.angular.z = 0.5 * abs(message.angular.z) / message.angular.z; // limiting the resultant angular velocity
            }
            // cout << message.angular.z << " ";
            publisher_->publish(message);

            // RCLCPP_INFO(this->get_logger(), "lin_vel : %lf", vel);
            // RCLCPP_INFO(this->get_logger(), "ang_vel : %lf", angle_diff);
        }

        else
        {
            set_goal();
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_2;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlBot3>());
    rclcpp::shutdown();
    return 0;
}
