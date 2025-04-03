#include <memory>
#include <thread>
#include "franka_ros2_csc379/franka_state_publisher_ros2.hpp"

namespace csc379
{

FrankaStatePublisherROS2::FrankaStatePublisherROS2(
    std::shared_ptr<rclcpp::Node> node_handle)
{
    node_handle_ = node_handle;
    // Task: Create necessary Franka objects, follow any libfranka example

    robot_ = std::make_shared<franka::Robot>("192.168.1.107");
    // Task: Create a publisher
    js_publisher_ = node_handle_->create_publisher<sensor_msgs::msg::JointState>("/franka/measured_js", 10);
}

void FrankaStatePublisherROS2::ReadStateAndPublish()
{
    // Task: Read Franka State Once, do not use franka::control method
    auto robot_state = robot_->readOnce();
    this->publishState(robot_state.q);

    auto O_T_EE = robot_state.O_T_EE;
    std::cout << "--------------------------------------\n" ;
    std::cout << O_T_EE[0] << " " << O_T_EE[1] << " " << O_T_EE[2] << " " << O_T_EE[3] << "\n" ;
    std::cout << O_T_EE[4] << " " << O_T_EE[5] << " " << O_T_EE[6] << " " << O_T_EE[7] << "\n" ;
    std::cout << O_T_EE[8] << " " << O_T_EE[9] << " " << O_T_EE[10] << " " << O_T_EE[11] << "\n" ;
    std::cout << O_T_EE[12] << " " << O_T_EE[13] << " " << O_T_EE[14] << " " << O_T_EE[15] << "\n" ;
}

void FrankaStatePublisherROS2::publishState(std::array<double, 7> positions)
{
    sensor_msgs::msg::JointState js_msg;
    // Task: Create sensor_msgs JointState and Publish
    for (int i = 0; i < positions.size(); ++i)
    {
        js_msg.position.push_back(positions[i]);
    }
    js_publisher_->publish(js_msg);
}

} // namespace csc379

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_handle =
        std::make_shared<rclcpp::Node>("franka_state_publisher");
    csc379::FrankaStatePublisherROS2 franka_state_publisher =
        csc379::FrankaStatePublisherROS2(node_handle);
    auto spin_thread = std::thread([&]() { rclcpp::spin(node_handle); });

    while (rclcpp::ok())
    {
        franka_state_publisher.ReadStateAndPublish();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 Hz
        // Task: Add rate of publishing
    }

    rclcpp::shutdown();
    spin_thread.join(); // rclcpp::shutdown stops rclcpp::spin and thread
    return 0;
}
