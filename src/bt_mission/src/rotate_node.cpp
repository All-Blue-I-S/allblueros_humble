#include "rotate_node.h"

#include <cmath>
#include <functional>
#include <chrono>


RotateNode::RotateNode(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config),
      node_(node),
      action_initialized_(false),
      odom_received_(false)
{
    mux_pub_cmd_vel_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/mux/cmd_vel",
            rclcpp::QoS(1).transient_local());

    mux_dvz_input_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/mux/dvz_input",
            rclcpp::QoS(1).transient_local());

    cmd_vel_pub_ =
        node_->create_publisher<auv_navigation::msg::CurveReference>(
            "/rotate_reference",
            10);

    odom_sub_ =
        node_->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom",
            1,
            std::bind(
                &RotateNode::odomCallback,
                this,
                std::placeholders::_1));
}


BT::PortsList RotateNode::providedPorts()
{
    return {
        BT::InputPort<std::string>(
            "axis",
            "Eixo de rotação (x, y, z)"),

        BT::InputPort<double>(
            "angle",
            "Ângulo alvo em graus")
    };
}


void RotateNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Armazena a odometria atual
    current_odom_.pose = msg->pose;

    // Extrai o quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll;
    double pitch;
    double yaw;

    m.getRPY(roll, pitch, yaw);

    if (!odom_received_)
    {
        odom_received_ = true;
    }
}


BT::NodeStatus RotateNode::onStart()
{
    // Reinicializa o estado da ação.
    action_initialized_ = false;
    odom_received_ = false;

    // Lê os parâmetros de entrada do XML
    if (!getInput<std::string>("axis", axis_))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RotateNode: Missing required input [axis]");

        return BT::NodeStatus::FAILURE;
    }

    if (!getInput<double>("angle", target_angle_))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RotateNode: Missing required input [angle]");

        return BT::NodeStatus::FAILURE;
    }

    // Converte graus para radianos
    target_angle_ =
        target_angle_ * M_PI / 180.0;


    // ---------------------------------------------------------
    // Muda o mux do cmd_vel para GO2
    // ---------------------------------------------------------

    std_msgs::msg::String msg;
    msg.data = "GO2";

    mux_pub_cmd_vel_->publish(msg);


    // ---------------------------------------------------------
    // Muda o mux do DVZ para ROTATE
    // ---------------------------------------------------------

    std_msgs::msg::String dvz_msg;
    dvz_msg.data = "ROTATE";

    mux_dvz_input_->publish(dvz_msg);


    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus RotateNode::onRunning()
{
    // Ainda não recebemos odometria
    if (!odom_received_)
    {
        return BT::NodeStatus::RUNNING;
    }


    // ---------------------------------------------------------
    // Obtém orientação atual
    // ---------------------------------------------------------

    tf2::Quaternion q(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll;
    double pitch;
    double yaw;

    m.getRPY(roll, pitch, yaw);


    // ---------------------------------------------------------
    // Verifica o eixo
    // ---------------------------------------------------------

    tf2::Quaternion desired_quat;

    if (axis_ == "ROLL")
    {
        desired_quat.setRPY(
            target_angle_,
            pitch,
            yaw);
    }
    else if (axis_ == "PITCH")
    {
        desired_quat.setRPY(
            roll,
            target_angle_,
            yaw);
    }
    else if (axis_ == "YAW")
    {
        desired_quat.setRPY(
            roll,
            pitch,
            target_angle_);
    }
    else
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RotateNode: Invalid axis input [%s]",
            axis_.c_str());

        return BT::NodeStatus::FAILURE;
    }


    // ---------------------------------------------------------
    // Publica a referência somente uma vez
    // ---------------------------------------------------------

    if (!action_initialized_)
    {
        action_initialized_ = true;

        auv_navigation::msg::CurveReference cmd;

        cmd.header.stamp =
            node_->get_clock()->now();

        cmd.header.frame_id = "base_link";

        cmd.pose.position.x =
            current_odom_.pose.pose.position.x;

        cmd.pose.position.y =
            current_odom_.pose.pose.position.y;

        cmd.pose.position.z =
            current_odom_.pose.pose.position.z;

        cmd.pose.orientation.x =
            desired_quat.x();

        cmd.pose.orientation.y =
            desired_quat.y();

        cmd.pose.orientation.z =
            desired_quat.z();

        cmd.pose.orientation.w =
            desired_quat.w();

        cmd_vel_pub_->publish(cmd);
    }


    // ---------------------------------------------------------
    // Calcula erro angular
    // ---------------------------------------------------------

    double error = 0.0;

    if (axis_ == "ROLL")
    {
        error = std::abs(
            roll - target_angle_);
    }
    else if (axis_ == "PITCH")
    {
        error = std::abs(
            pitch - target_angle_);
    }
    else if (axis_ == "YAW")
    {
        error = std::abs(
            yaw - target_angle_);
    }


    // ---------------------------------------------------------
    // Verifica se atingiu o alvo
    // ---------------------------------------------------------

    if (error < 0.1)
    {
        // Volta o mux do cmd_vel para IDLE
        std_msgs::msg::String msg;
        msg.data = "IDLE";

        mux_pub_cmd_vel_->publish(msg);


        // Volta o DVZ para PATH_FOLLOWING
        std_msgs::msg::String dvz_msg;
        dvz_msg.data = "PATH_FOLLOWING";

        mux_dvz_input_->publish(dvz_msg);


        RCLCPP_INFO(
            node_->get_logger(),
            "RotateNode: Rotação concluída.");

        return BT::NodeStatus::SUCCESS;
    }


    return BT::NodeStatus::RUNNING;
}


void RotateNode::onHalted()
{
    // ---------------------------------------------------------
    // Volta o cmd_vel para IDLE
    // ---------------------------------------------------------

    std_msgs::msg::String msg;
    msg.data = "IDLE";

    mux_pub_cmd_vel_->publish(msg);


    // ---------------------------------------------------------
    // Volta o DVZ para PATH_FOLLOWING
    // ---------------------------------------------------------

    std_msgs::msg::String dvz_msg;
    dvz_msg.data = "PATH_FOLLOWING";

    mux_dvz_input_->publish(dvz_msg);


    RCLCPP_WARN(
        node_->get_logger(),
        "RotateNode Abortado pela Behavior Tree!");
}
