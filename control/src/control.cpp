#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()

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

        // Advertise the selected_integers topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("selected_integers", 10);
    }

private:
    void availableAreaCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        // Print out the received data
        RCLCPP_INFO(this->get_logger(), "Received available areas:");

        // Clear the vector to remove any previously received messages
        received_messages.clear();

        // Create a new vector to hold the data from the received message
        std::vector<int> received_data(msg->data.begin(), msg->data.end());

        // Add the new message data to the vector
        received_messages.push_back(received_data);

        // Print the received data
        for (auto area : received_data)
        {
            RCLCPP_INFO(this->get_logger(), "%d", area);
        }

        // Randomly select an integer from the received data and store it in a new list
        if (!received_data.empty()) {
            srand(time(nullptr)); // Seed the random number generator
            int random_index;
            int selected_integer;

            do {
                random_index = rand() % received_data.size(); // Generate a random index
                selected_integer = received_data[random_index]; // Select the integer at the random index
            } while (std::find(selected_integers.begin(), selected_integers.end(), selected_integer) != selected_integers.end());

            selected_integers.push_back(selected_integer); // Store the selected integer in the new list
            RCLCPP_INFO(this->get_logger(), "Selected integer: %d", selected_integer);

            // Publish the selected_integers list
            auto msg = std_msgs::msg::Int32MultiArray();
            msg.data = selected_integers;
            publisher_->publish(msg);
        }

        // Print the selected_integer list
        RCLCPP_INFO(this->get_logger(), "Selected integers list:");
        for (size_t i = 0; i < selected_integers.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "%d", selected_integers[i]);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    std::vector<std::vector<int>> received_messages; // List to store received messages
    std::vector<int> selected_integers; // List to store selected integers
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_; // Publisher for selected integers
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvailableAreaSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
