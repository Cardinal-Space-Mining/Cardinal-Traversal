#ifndef TRAVERSAL_NODE_H
#define TRAVERSAL_NODE_H

#include <node_grid.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

enum TRAVERSAL_RESULT : int
{
    SUCCESS = 0,
    PATH_NOT_FOUND = 1,
    OFF_PATH = 2,
    OUTDATED_MAP = 4,
    OUTDATED_POSE = 8,
    THE_ROBOT_FUCKING_EXPLODED = 16,
    UNKNOWN = 32,
};

inline TRAVERSAL_RESULT operator|(TRAVERSAL_RESULT a, TRAVERSAL_RESULT b)
{
    return static_cast<TRAVERSAL_RESULT>(
        static_cast<std::underlying_type_t<TRAVERSAL_RESULT>>(a) | static_cast<std::underlying_type_t<TRAVERSAL_RESULT>>(b));
}

class TraversalNode : public rclcpp::Node
{
public:
    TraversalNode() = default;
    ~TraversalNode() = default;

private:
    geometry_msgs::msg::PoseStamped &current_pose;
    geometry_msgs::msg::Pose &target_pose;

    std::vector<std::tuple<double, double>> &current_path;
    lazythetastar::NodeGrid &node_grid;
    const int8_t *weight_grid;

    void logError(TRAVERSAL_RESULT err)
    {
        if (err) // err = 0 means success
        {
            std::string error_message = " ERROR (traversal node): ";

            if (err & PATH_NOT_FOUND)
                error_message += "\n  Path not found! (internal error)";
            if (err & OFF_PATH)
                error_message += "\n  Unrecoverably off path, update current_pose ASAP";
            if (err & OUTDATED_MAP)
                error_message += "\n  Haven't recieved map update recently, freezing until updated";
            if (err & OUTDATED_POSE)
                error_message += "\n  Haven't recieved pose update recently, freezing until updated";
            if (err & THE_ROBOT_FUCKING_EXPLODED)
                error_message += "\n  Debug error (or the robot fucking exploded)";
            if (err & UNKNOWN)
                error_message += "\n  Unknown error";

            RCLCPP_INFO(this->get_logger(), error_message.c_str());
        }
    }

    TRAVERSAL_RESULT update_path();

    // MESSAGE CALLBACKS
    TRAVERSAL_RESULT occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid &msg)
    {
        node_grid.resize(msg.info.width * msg.info.height);
        weight_grid = msg.data.data();

        return update_path();
    }

    TRAVERSAL_RESULT current_pose_callback(const geometry_msgs::msg::PoseStamped &msg)
    {
        current_pose = msg;
        return update_path();
    }
    TRAVERSAL_RESULT target_pose_callback(const geometry_msgs::msg::Pose &msg)
    {
        target_pose = msg;
        return update_path();
    }
};

#endif // TRAVERSAL_NODE_H