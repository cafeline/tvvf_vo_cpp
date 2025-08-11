#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace tvvf_vo_c
{

  void TVVFVONode::publish_vector_field_visualization()
  {
    try
    {
      if (!robot_state_.has_value() || !goal_.has_value())
      {
        return;
      }

      // パラメータから可視化設定を取得
      double grid_spacing = this->get_parameter("vector_field_resolution").as_double();
      double grid_range = this->get_parameter("vector_field_range").as_double();

      auto marker_array = visualization_msgs::msg::MarkerArray();
      std::string global_frame = this->get_parameter("global_frame").as_string();

      // 既存マーカーをクリア
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = global_frame;
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker.id = 0;
      marker_array.markers.push_back(delete_marker);

      // 動的・静的障害物を統合
      std::vector<DynamicObstacle> all_obstacles;
      all_obstacles.insert(all_obstacles.end(), dynamic_obstacles_.begin(), dynamic_obstacles_.end());
      all_obstacles.insert(all_obstacles.end(), static_obstacles_.begin(), static_obstacles_.end());

      // ベクトル場計算とマーカー生成
      if (!controller_) return;
      const auto &tvvf_generator = controller_->get_tvvf_generator();

      Position robot_pos = robot_state_->position;
      int marker_id = 0;

      for (double x = robot_pos.x - grid_range; x <= robot_pos.x + grid_range; x += grid_spacing)
      {
        for (double y = robot_pos.y - grid_range; y <= robot_pos.y + grid_range; y += grid_spacing)
        {
          Position grid_pos(x, y);
          
          // ベクトル場計算
          auto force = tvvf_generator.compute_vector(
              grid_pos, 0.0, goal_.value(), all_obstacles, planned_path_);

          double force_magnitude = std::sqrt(force[0] * force[0] + force[1] * force[1]);
          
          // 小さすぎる力は表示しない
          if (force_magnitude < 0.05) continue;

          // 矢印マーカー作成
          auto arrow_marker = visualization_msgs::msg::Marker();
          arrow_marker.header.frame_id = global_frame;
          arrow_marker.header.stamp = this->get_clock()->now();
          arrow_marker.id = marker_id++;
          arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
          arrow_marker.action = visualization_msgs::msg::Marker::ADD;

          // 矢印の位置
          arrow_marker.pose.position.x = x;
          arrow_marker.pose.position.y = y;
          arrow_marker.pose.position.z = 0.1;

          // 矢印の向き
          double yaw = std::atan2(force[1], force[0]);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          arrow_marker.pose.orientation.x = q.x();
          arrow_marker.pose.orientation.y = q.y();
          arrow_marker.pose.orientation.z = q.z();
          arrow_marker.pose.orientation.w = q.w();

          // 矢印のサイズ（固定長）
          arrow_marker.scale.x = grid_spacing * 0.6;  // 長さ
          arrow_marker.scale.y = 0.05;                // 幅
          arrow_marker.scale.z = 0.05;                // 高さ

          // 色（緑）
          arrow_marker.color.r = 0.0;
          arrow_marker.color.g = 1.0;
          arrow_marker.color.b = 0.0;
          arrow_marker.color.a = 0.8;

          arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);
          marker_array.markers.push_back(arrow_marker);
        }
      }

      vector_field_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Vector field visualization error: %s", e.what());
    }
  }

  void TVVFVONode::publish_empty_visualization()
  {
    auto empty_marker_array = visualization_msgs::msg::MarkerArray();
    auto delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = this->get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.id = 0;
    empty_marker_array.markers.push_back(delete_marker);

    marker_pub_->publish(empty_marker_array);
    path_pub_->publish(empty_marker_array);
    vector_field_pub_->publish(empty_marker_array);
  }

} // namespace tvvf_vo_c