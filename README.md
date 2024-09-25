# Cardinal Traversal
Lazy Theta* path planning to Twist ROS Node

## ROS 2 node 
input:
- nav_msgs::msg::OccupancyGrid weights
- geometry_msgs::msg::PoseStamped current_pose
- geometry_msgs::msg::Pose target_pose

output:
- geometry_msgs::msg::TwistStamped twist

## Build instructions
1. Navigate to (or create) your ROS 2 workspace:
- `cd /robot_ws`
2. Create and go to src directory:
- `mkdir src && cd src`
3. Clone this repository:
- `git clone https://github.com/connortynan/CardinalTraversal`
4. Build from workspace directory:
- `cd ../ && colcon build`

## Todo list:
- Make the actual ros node
- Accept occupancy_grid + poses for path planning
- Accept current_pose + calculated path for trajectory planning
- Make the trajectory planning algorithm
- fix bugs currently unbeknownst
