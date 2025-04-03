#include <chrono>
#include <functional>
#include <cstdlib>
#include <cassert>
#include <iostream>

#include <franka/rate_limiting.h>

#include "franka_ros2_csc379/franka_impedance_control.hpp"

namespace csc379
{

FrankaImpedanceControl::FrankaImpedanceControl()
{
    // Task: Create the robot object with correct ip address
    // Task: Set the command_joint_positions_ as the current state of the robot
    robot_ = std::make_shared<franka::Robot>("192.168.1.107");

    auto robot_state = robot_->readOnce();
    command_joint_positions_ = robot_state.q;
    current_joint_positions_ = robot_state.q;

    // Start robot control
    k_gains_ = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    d_gains_ = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    robot_->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    model_ = std::make_shared<franka::Model>(robot_->loadModel());

    auto robot_control_func = [this]() {
        this->robot_->control(std::bind(
            &FrankaImpedanceControl::impedanceControlCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    };
    control_thread_ = std::thread(robot_control_func);
}

void FrankaImpedanceControl::Join()
{
    control_thread_.join();
}

std::array<double, 7> FrankaImpedanceControl::GetCurrentJointPositions()
{
    // Task: Get the current joint positions of the franka and return
    std::lock_guard<std::mutex> lock(current_state_mtx_);
    return current_joint_positions_;
}

void FrankaImpedanceControl::SetCommandJointPositions(
    const std::array<double, 7>& joint_positions)
{
    // Task: Set command joint positions from the argument joint positions
    // Remember to lock the mutex for thread safety, as this
    // function runs on a different thread than imepdanceControlCallback
    std::lock_guard<std::mutex> lock(joint_positions_mtx_);
    command_joint_positions_ = joint_positions;
}

franka::Torques FrankaImpedanceControl::impedanceControlCallback(
    const franka::RobotState& state, franka::Duration /*period*/)
{
    {
        std::lock_guard<std::mutex> lock(this->current_state_mtx_);
        // Task: read and set the current joint positions here
        current_joint_positions_ = state.q;
    }

    std::array<double, 7> coriolis = model_->coriolis(state);
    std::array<double, 7> tau_d_calculated;
    {
        // --- DO NOT MODIFY THIS, this is for safety --- //
        std::lock_guard<std::mutex> lock(joint_positions_mtx_);
        // validate command and state
        assert(command_joint_positions_.size() == 7);
        double tol = 0.1; // Radians
        for (size_t i = 0; i < 7; i++)
        {
            // std::cout << "joint values command vs actual\n";
            // std::cout << command_joint_positions_[i] << std::endl;
            // std::cout << state.q[i] << std::endl;

            if (abs(command_joint_positions_[i] - state.q[i]) > tol)
            {
                throw std::runtime_error(
                    "Desired joint [" + std::to_string(i) + 
                    "] position [" + std::to_string(command_joint_positions_[i]) + 
                    " - actual position [" + std::to_string(state.q[i]) + "] is larger than tolerance: " +
                    std::to_string(tol));
            }
        }
        // ----------------------------------------------- //

        for (size_t i = 0; i < 7; i++)
        {
            tau_d_calculated[i] =
                k_gains_[i] * (command_joint_positions_[i] - state.q[i]) -
                d_gains_[i] * state.dq[i] + coriolis[i];
        }
    }

    // Limit rate torque
    std::array<double, 7> tau_d_rate_limited = franka::limitRate(
        franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

    // Send torque command.
    return tau_d_rate_limited;
}

} // namespace csc379
