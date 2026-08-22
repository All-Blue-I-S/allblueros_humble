#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

// Nós da Behavior Tree
#include <ActivateClaw_node.h>
#include <align_node.h>
#include <check_empty_table_node.h>
#include <colect_data_node.h>
#include <detect_node.h>
#include <DropMarkers_node.h>
#include <emerge_node.h>
#include <fire_node.h>
#include <FirstItem_node.h>
#include <go2_node.h>
#include <Increment_node.h>
#include <rotate_node.h>
#include <Save_Pose.h>


int main(int argc, char** argv)
{
    // ---------------------------------------------------------
    // Inicialização do ROS 2
    // ---------------------------------------------------------

    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>("bt_executor_node");


    // ---------------------------------------------------------
    // Behavior Tree Factory
    // ---------------------------------------------------------

    BT::BehaviorTreeFactory factory;


    // ---------------------------------------------------------
    // Registro dos nós ROS
    // ---------------------------------------------------------

    factory.registerBuilder<Go2Node>(
        "Go_to",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<Go2Node>(
                name,
                config,
                node);
        });


    /*
    factory.registerBuilder<ActivateClaw>(
        "Claw",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<ActivateClaw>(
                name,
                config,
                node);
        });


    factory.registerBuilder<AlignNode>(
        "Align",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<AlignNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<CheckEmptyTableNode>(
        "CheckEmptyTable",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<CheckEmptyTableNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<ColectDataNode>(
        "ColectData",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<ColectDataNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<DetectNode>(
        "Detect",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<DetectNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<DropMarkers>(
        "DropMarkers",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<DropMarkers>(
                name,
                config,
                node);
        });


    factory.registerBuilder<EmergeNode>(
        "Emerge",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<EmergeNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<FireNode>(
        "fire",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<FireNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<RotateNode>(
        "Rotate",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<RotateNode>(
                name,
                config,
                node);
        });


    factory.registerBuilder<SavePose>(
        "SavePose",
        [node](const std::string& name,
               const BT::NodeConfiguration& config)
        {
            return std::make_unique<SavePose>(
                name,
                config,
                node);
        });
    */


    // ---------------------------------------------------------
    // Registro dos nós que NÃO usam ROS
    // ---------------------------------------------------------

    factory.registerNodeType<FirstItem>("FirstItem");

    factory.registerNodeType<Increment>("increment");


    // ---------------------------------------------------------
    // Localização da árvore XML
    // ---------------------------------------------------------

    std::string package_path =
        ament_index_cpp::get_package_share_directory(
            "bt_mission");

    std::string tree_path =
        package_path + "/trees/Testes.xml";


    // ---------------------------------------------------------
    // Criação da Behavior Tree
    // ---------------------------------------------------------

    auto tree =
        factory.createTreeFromFile(tree_path);


    // ---------------------------------------------------------
    // Logger
    // ---------------------------------------------------------

    BT::StdCoutLogger logger(tree);


    RCLCPP_INFO(
        node->get_logger(),
        "Behavior Tree iniciada.");


    // ---------------------------------------------------------
    // Loop principal
    // ---------------------------------------------------------

    rclcpp::WallRate rate(10.0);

    BT::NodeStatus status =
        BT::NodeStatus::RUNNING;


    while (
        rclcpp::ok() &&
        status == BT::NodeStatus::RUNNING)
    {
        // Processa callbacks do ROS 2
        rclcpp::spin_some(node);

        // Executa um tick da Behavior Tree
        status = tree.tickRoot();

        // Mantém aproximadamente 10 Hz
        rate.sleep();
    }


    // ---------------------------------------------------------
    // Finalização
    // ---------------------------------------------------------

    RCLCPP_INFO(
        node->get_logger(),
        "Missão Terminada com status: %d",
        static_cast<int>(status));


    rclcpp::shutdown();

    return 0;
}
