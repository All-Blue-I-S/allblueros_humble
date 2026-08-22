#include "planner.h"


// ============================================================
// Construtor
// ============================================================

PlannerSimServer::PlannerSimServer(const std::string &name)
    : Node(name),
      pose_received_(false),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    // ========================================================
    // Odometry
    // ========================================================

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth/odom",
        10,
        std::bind(
            &PlannerSimServer::poseCB,
            this,
            std::placeholders::_1));


    // ========================================================
    // Path
    // ========================================================

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path",
        10);


    // ========================================================
    // Action Server
    // ========================================================

    action_server_ = rclcpp_action::create_server<Go2>(
        this,
        "go2_server",

        std::bind(
            &PlannerSimServer::handle_goal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),

        std::bind(
            &PlannerSimServer::handle_cancel,
            this,
            std::placeholders::_1),

        std::bind(
            &PlannerSimServer::handle_accepted,
            this,
            std::placeholders::_1));


    RCLCPP_INFO(
        this->get_logger(),
        "Servidor Go2 Planejador iniciado. "
        "Aguardando odometria e objetivos...");
}


// ============================================================
// Callback da Odometria
// ============================================================

void PlannerSimServer::poseCB(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);

    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;

    pose_received_ = true;
}


// ============================================================
// Action: receber Goal
// ============================================================

rclcpp_action::GoalResponse PlannerSimServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Go2::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(
        this->get_logger(),
        "Novo objetivo Go2 recebido da Behavior Tree.");

    RCLCPP_INFO(
        this->get_logger(),
        "Objetivo: X=%.2f Y=%.2f Z=%.2f Frame=%s",
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z,
        goal->target_pose.header.frame_id.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


// ============================================================
// Action: Cancelamento
// ============================================================

rclcpp_action::CancelResponse PlannerSimServer::handle_cancel(
    const std::shared_ptr<GoalHandleGo2> goal_handle)
{
    (void)goal_handle;

    RCLCPP_WARN(
        this->get_logger(),
        "Solicitacao de cancelamento recebida.");

    return rclcpp_action::CancelResponse::ACCEPT;
}


// ============================================================
// Action: Goal aceito
// ============================================================

void PlannerSimServer::handle_accepted(
    const std::shared_ptr<GoalHandleGo2> goal_handle)
{
    // O execute precisa rodar em uma thread separada.
    //
    // Isso é importante porque execute() possui um loop
    // enquanto o AUV está navegando.

    std::thread{
        std::bind(
            &PlannerSimServer::execute,
            this,
            std::placeholders::_1),
        goal_handle
    }.detach();
}


// ============================================================
// Execução do Goal
// ============================================================

void PlannerSimServer::execute(
    const std::shared_ptr<GoalHandleGo2> goal_handle)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Iniciando execucao do Go2.");


    // ========================================================
    // 1. ESPERAR ODOMETRIA
    // ========================================================

    rclcpp::Rate wait_rate(10.0);

    while (rclcpp::ok())
    {
        if (goal_handle->is_canceling())
        {
            auto result = std::make_shared<Go2::Result>();
            result->success = false;

            RCLCPP_WARN(
                this->get_logger(),
                "Go2 cancelado antes de receber odometria.");

            goal_handle->canceled(result);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            if (pose_received_)
                break;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Aguardando primeira odometria do AUV...");

        wait_rate.sleep();
    }


    if (!rclcpp::ok())
    {
        return;
    }


    // ========================================================
    // 2. PEGAR POSE ATUAL
    // ========================================================

    geometry_msgs::msg::PoseStamped current_pose;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose = current_pose_;
    }


    // ========================================================
    // 3. PEGAR OBJETIVO
    // ========================================================

    geometry_msgs::msg::PoseStamped target_pose =
        goal_handle->get_goal()->target_pose;

    geometry_msgs::msg::PoseStamped target_pose_map;


    // ========================================================
    // 4. TRANSFORMAR PARA MAP
    // ========================================================

    try
    {
        if (target_pose.header.frame_id != "map")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Transformando do frame [%s] para [map]...",
                target_pose.header.frame_id.c_str());

            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(
                    "map",
                    target_pose.header.frame_id,
                    tf2::TimePointZero,
                    tf2::durationFromSec(1.0));

            tf2::doTransform(
                target_pose,
                target_pose_map,
                transform);
        }
        else
        {
            target_pose_map = target_pose;
        }


        // ====================================================
        // Valor -1000 significa "usar posição atual"
        // ====================================================

        if (target_pose_map.pose.position.x == -1000.0)
        {
            target_pose_map.pose.position.x =
                current_pose.pose.position.x;
        }

        if (target_pose_map.pose.position.y == -1000.0)
        {
            target_pose_map.pose.position.y =
                current_pose.pose.position.y;
        }

        if (target_pose_map.pose.position.z == -1000.0)
        {
            target_pose_map.pose.position.z =
                current_pose.pose.position.z;
        }
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Falha ao converter referencial (TF): %s",
            ex.what());

        auto result = std::make_shared<Go2::Result>();
        result->success = false;

        goal_handle->abort(result);

        return;
    }


    // ========================================================
    // 5. PUBLICAR CAMINHO
    // ========================================================

    nav_msgs::msg::Path path_msg;

    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    path_msg.poses.push_back(current_pose);
    path_msg.poses.push_back(target_pose_map);

    pub_path_->publish(path_msg);


    // ========================================================
    // 6. LOOP DE NAVEGAÇÃO
    // ========================================================

    RCLCPP_INFO(
        this->get_logger(),
        "\n[PLANNER]\n"
        "\tIniciando navegacao para:\n"
        "\tX = %.2f\n"
        "\tY = %.2f\n"
        "\tZ = %.2f\n",

        target_pose_map.pose.position.x,
        target_pose_map.pose.position.y,
        target_pose_map.pose.position.z);


    rclcpp::Rate rate(10.0);

    auto feedback =
        std::make_shared<Go2::Feedback>();


    while (rclcpp::ok())
    {
        // ====================================================
        // Cancelamento
        // ====================================================

        if (goal_handle->is_canceling())
        {
            auto result =
                std::make_shared<Go2::Result>();

            result->success = false;

            RCLCPP_WARN(
                this->get_logger(),
                "Missao Go2 cancelada pela Behavior Tree.");

            goal_handle->canceled(result);

            return;
        }


        // ====================================================
        // Pegar pose atualizada
        // ====================================================

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);

            current_pose = current_pose_;
        }


        // ====================================================
        // Calcular distância
        // ====================================================

        double dx =
            target_pose_map.pose.position.x -
            current_pose.pose.position.x;

        double dy =
            target_pose_map.pose.position.y -
            current_pose.pose.position.y;

        double dz =
            target_pose_map.pose.position.z -
            current_pose.pose.position.z;


        double distance =
            std::sqrt(
                dx * dx +
                dy * dy +
                dz * dz);


        // ====================================================
        // Feedback
        // ====================================================

        feedback->distance_to_goal = distance;

        goal_handle->publish_feedback(feedback);


        // ====================================================
        // Condição de sucesso
        // ====================================================

        if (distance < 0.3)
        {
            auto result =
                std::make_shared<Go2::Result>();

            result->success = true;

            RCLCPP_INFO(
                this->get_logger(),
                "Go2 concluido com sucesso! "
                "AUV chegou ao destino.");

            goal_handle->succeed(result);

            return;
        }


        rate.sleep();
    }
}


// ============================================================
// MAIN
// ============================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto planner =
        std::make_shared<PlannerSimServer>("go2_server");

    // ========================================================
    // IMPORTANTE:
    //
    // O execute() roda em uma thread separada, portanto
    // podemos utilizar um executor normal.
    // ========================================================

    rclcpp::spin(planner);

    rclcpp::shutdown();

    return 0;
}
