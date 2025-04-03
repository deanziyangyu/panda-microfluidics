#include <array>
#include <chrono>
#include <thread>
#include <iostream>

#include "franka_ros2_csc379/franka_impedance_control.hpp"

int main(int argc, char* argv[])
{
    csc379::FrankaImpedanceControl fic = csc379::FrankaImpedanceControl();

    // Task: After check impedance works properly by pushing robot.
    // Create a joint trajectory here and move robot using
    // SetCommandJointPositions()

    std::array<double, 7> start_joint_positions = fic.GetCurrentJointPositions();
    double goal_joint_1 = start_joint_positions[0] + 0.5;
    std::array<double, 7> command_joint_positions = start_joint_positions;

    double increment = 0.01; // rad
    double velocity = 0.2; // rad/s
    double sleep_s = increment / velocity;
    std::cout << "sleep_s: " << sleep_s << "\n";

    while (command_joint_positions[0] < goal_joint_1)
    {
        command_joint_positions[0] = command_joint_positions[0] + increment;
        fic.SetCommandJointPositions(command_joint_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_s * 1000)));
        std::cout << "command_joint_positions[0] " << command_joint_positions[0] << "\n";
    }
    command_joint_positions[0] = goal_joint_1;
    fic.SetCommandJointPositions(command_joint_positions);
    std::cout << "command_joint_positions[0] " << command_joint_positions[0] << "\n";
    std::cout << "Goal finished\n"; 

    // I'm lazy I want the robot to move back

    while (command_joint_positions[0] > start_joint_positions[0])
    {
        command_joint_positions[0] = command_joint_positions[0] - increment;
        fic.SetCommandJointPositions(command_joint_positions);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_s * 1000)));
        std::cout << "command_joint_positions[0] " << command_joint_positions[0] << "\n";
    }
    command_joint_positions[0] = start_joint_positions[0];
    fic.SetCommandJointPositions(command_joint_positions);
    std::cout << "command_joint_positions[0] " << command_joint_positions[0] << "\n";
    std::cout << "Back to start finished\n"; 

    fic.Join(); // waits for control thread to finish so program doesn't exit.
    return 0;
}
