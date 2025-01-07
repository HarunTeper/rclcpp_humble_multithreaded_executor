#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TestNode : public rclcpp::Node
{
    public:
        TestNode();

        //create timers
        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;

        //create timer callbacks
        void timer_callback(int timer_number);

        //create callback groups, one for each timer
        rclcpp::CallbackGroup::SharedPtr callback_group1_;

    private:
};