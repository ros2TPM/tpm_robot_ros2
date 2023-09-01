#include "SolutionChooser.h"

// Function to calculate the Euclidean distance between two joint states
double calculateJointDistance(const std::vector<double>& joint_state1, const std::vector<double>& joint_state2) {
    double distance = 0.0;
    for (size_t i = 0; i < joint_state1.size(); ++i) {
        double diff = joint_state1[i] - joint_state2[i];
        distance += diff * diff;
    }
    return std::sqrt(distance);
}

// Function to find the solution closest to the current joint state
bool chooseSolution(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                                const std::vector<std::vector<double>>& joint_solutions,
                                std::vector<double>& chosen_joint_solution) {
    // Get the current joint state of the robot
    std::vector<double> current_joint_state = move_group_interface.getCurrentJointValues();

    // Initialize variables to store the closest joint solution and its distance
    double min_distance = std::numeric_limits<double>::max();
    std::vector<double> closest_joint_solution;

    // Loop through all the joint solutions and find the closest one
    for (const auto& joint_solution : joint_solutions) {
        double distance = calculateJointDistance(current_joint_state, joint_solution);
        if (distance < min_distance) {
            min_distance = distance;
            closest_joint_solution = joint_solution;
        }
    }

    // If a closest joint solution is found, assign it to chosen_joint_solution
    if (!closest_joint_solution.empty()) {
        chosen_joint_solution = closest_joint_solution;
        return true;
    }

    // No closest joint solution found
    return false;
}