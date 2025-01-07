#include "multi_test/tester.hpp"

TestNode::TestNode() : Node("test")
{
    std::cout << "TestNode constructor" << std::endl;
    //create two callback groups, one for each timer
    callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //create two timers with callbacks
    timer1_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() {this->timer_callback(1);}, callback_group1_);
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() {this->timer_callback(2);}, callback_group1_);
}

void TestNode::timer_callback(int timer_number)
{
    RCLCPP_INFO(this->get_logger(), "Timer %d callback", timer_number);
}