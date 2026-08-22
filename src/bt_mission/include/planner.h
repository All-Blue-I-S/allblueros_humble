#ifndef PLANNER_H
#define PLANNER_H

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <bt_mission/action/go2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class PlannerSimServer : public rclcpp::Node
{
public:

    using Go2 = bt_mission::action::Go2;
    using GoalHandleGo2 = rclcpp_action::ServerGoalHandle<Go2>;

    explicit PlannerSimServer(const std::string &name);

private:

    // ========================================================
    // Action Server
    // ========================================================

    rclcpp_action::Server<Go2>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Go2::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGo2> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleGo2> goal_handle);

    void execute(
        const std::shared_ptr<GoalHandleGo2> goal_handle);


    // ========================================================
    // Odometry
    // ========================================================

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        sub_odom_;

    void poseCB(
        const nav_msgs::msg::Odometry::SharedPtr msg);


    // ========================================================
    // Path
    // ========================================================

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        pub_path_;


    // ========================================================
    // Estado atual do AUV
    // ========================================================

    geometry_msgs::msg::PoseStamped current_pose_;

    bool pose_received_;

    std::mutex pose_mutex_;


    // ========================================================
    // TF2
    // ========================================================

    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;
};

#endif // PLANNER_H
