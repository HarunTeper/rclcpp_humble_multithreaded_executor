#include "multi_test/tester.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    //create a node
    auto node = std::make_shared<TestNode>();

    //add the node to the executor
    executor.add_node(node);

    //spin the node
    executor.spin();

    rclcpp::shutdown();
    return 0;
}