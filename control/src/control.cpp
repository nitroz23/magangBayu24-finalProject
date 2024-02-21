#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class AvailableAreaSubscriber : public rclcpp::Node
{
public:
    AvailableAreaSubscriber() : Node("available_area_subscriber")
    {
        // Subscribe to the available_area topic
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "available_area", 10, std::bind(&AvailableAreaSubscriber::availableAreaCallback, this, std::placeholders::_1));
    }

private:
    void availableAreaCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        // Print out the received data
        RCLCPP_INFO(this->get_logger(), "Received available areas:");
        for (auto area : msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "%d", area);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvailableAreaSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
