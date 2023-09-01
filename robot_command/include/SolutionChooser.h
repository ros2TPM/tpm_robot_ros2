
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>


bool chooseSolution(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const std::vector<std::vector<double>>& joint_solutions,
    std::vector<double>& chosen_joint_solution) ;   


