#ifndef ROTATE_NODE_H
#define ROTATE_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <auv_navigation/msg/curve_reference.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <string>


class RotateNode : public BT::StatefulActionNode
{
private:

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        mux_pub_cmd_vel_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        mux_dvz_input_;

    rclcpp::Publisher<auv_navigation::msg::CurveReference>::SharedPtr
        cmd_vel_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;


    std::string axis_;

    double target_angle_;

    double current_angle_;

    nav_msgs::msg::Odometry current_odom_;

    bool action_initialized_;

    bool odom_received_;


public:

    RotateNode(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;
};

#endif // ROTATE_NODE_H
