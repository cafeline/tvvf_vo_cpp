#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Position goal_position(msg->point.x, msg->point.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());
    planned_path_.reset();
    RCLCPP_INFO(this->get_logger(), "Goal set: (%.2f, %.2f)", goal_position.x, goal_position.y);
  }

  void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                msg->info.width, msg->info.height, msg->info.resolution);
  }

  void TVVFVONode::obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, bool is_dynamic)
  {
    auto& obstacles = is_dynamic ? dynamic_obstacles_ : static_obstacles_;
    obstacles.clear();

    for (const auto &marker : msg->markers)
    {
      if (marker.action == visualization_msgs::msg::Marker::ADD)
      {
        Position position(marker.pose.position.x, marker.pose.position.y);
        Velocity velocity(0.0, 0.0);
        double radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
        obstacles.emplace_back(marker.id, position, velocity, radius);
      }
    }
  }

  void TVVFVONode::path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      planned_path_.reset();
      RCLCPP_WARN(this->get_logger(), "Received empty path from external planner");
      return;
    }

    Path new_path;
    for (const auto& pose : msg->poses)
    {
      Position position(pose.position.x, pose.position.y);
      new_path.add_point(position, 0.0);
    }

    planned_path_ = new_path;
    RCLCPP_INFO(this->get_logger(), "Received external path: %zu points", planned_path_->size());
  }

} // namespace tvvf_vo_c