#include <chrono>
#include <functional>
#include <cstdlib>

#include "franka_ros2_csc379/franka_impedance_control_ros2.hpp"

namespace csc379
{

FrankaImpedanceControlROS2::FrankaImpedanceControlROS2(
    std::shared_ptr<rclcpp::Node> node_handle)
{
    // This should be correct from your previous task
    fic_ = std::make_unique<FrankaImpedanceControl>();

    node_handle_ = node_handle;

    // Task: Create a publisher
    js_publisher_ = node_handle->create_publisher<sensor_msgs::msg::JointState>("/franka/measured_js", 10);

    auto publish_func = [&, this]() {
        // Task: Get current joint positions from fic and publish
        while (rclcpp::ok())
        {
            this->publishState(); // Modify accordingly
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 Hz
        }
    };
    publisher_thread_ = std::thread(publish_func);

    // Task: Create a subscriber with setJointPosition as the callback
    js_subscription_ = node_handle->create_subscription<sensor_msgs::msg::JointState>(
    "/franka/servo_jp", 10, std::bind(&FrankaImpedanceControlROS2::setJointPositions, this, std::placeholders::_1));
}

void FrankaImpedanceControlROS2::publishState()
{
    sensor_msgs::msg::JointState js_msg;
    std::array<double, 7> positions = fic_->GetCurrentJointPositions();
    for (int i = 0; i < positions.size(); ++i)
    {
        js_msg.position.push_back(positions[i]);
    }
    js_publisher_->publish(js_msg);
    // Task: Copy from your franka_state_publisher
}

void FrankaImpedanceControlROS2::setJointPositions(
    const sensor_msgs::msg::JointState& js_msg)
{
    std::array<double, 7> positions;
    for (int i = 0; i < positions.size(); ++i)
    {
        positions[i] = js_msg.position[i];
    }
    fic_->SetCommandJointPositions(positions);
    // Task: Create a subscriber and set the joint positions,
}

void FrankaImpedanceControlROS2::Join()
{
    fic_->Join();
    publisher_thread_.join();
    return;
}

} // namespace csc379
