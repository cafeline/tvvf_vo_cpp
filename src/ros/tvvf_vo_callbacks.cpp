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
    
    // GlobalFieldGeneratorで静的場を再計算
    if (current_map_.has_value() && global_field_generator_) {
      global_field_generator_->precomputeStaticField(current_map_.value(), goal_position);
      RCLCPP_INFO(this->get_logger(), "Static field precomputed for new goal");
    }
  }

  void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                msg->info.width, msg->info.height, msg->info.resolution);
    
    // マップを保存
    current_map_ = *msg;
    
    // GlobalFieldGeneratorで静的場を再計算（ゴールが設定されている場合）
    if (goal_.has_value() && global_field_generator_) {
      global_field_generator_->precomputeStaticField(*msg, goal_->position);
      RCLCPP_INFO(this->get_logger(), "Static field precomputed for new map");
    }
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
        // DynamicObstacle構造体は3つの引数を取る（IDは持たない）
        obstacles.emplace_back(position, velocity, radius);
      }
    }
  }

} // namespace tvvf_vo_c