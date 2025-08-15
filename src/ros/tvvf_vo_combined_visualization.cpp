#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace tvvf_vo_c
{

  void TVVFVONode::publish_combined_field_visualization(const VectorField& field, const Position& robot_pos)
  {
    try
    {
      // 合成ベクトル場のみを表示
      auto marker_array = visualization_msgs::msg::MarkerArray();
      
      std::string global_frame = this->get_parameter("global_frame").as_string();

      // 既存マーカーをクリア
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = global_frame;
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker.id = 0;
      marker_array.markers.push_back(delete_marker);

      // 可視化パラメータ
      int stride = 5;  // 5セルごとに表示
      int marker_id = 1;
      
      for (int y = 0; y < field.height; y += stride)
      {
        for (int x = 0; x < field.width; x += stride)
        {
          // ワールド座標に変換
          Position world_pos = field.gridToWorld(x, y);
          
          // 元のベクトル取得
          auto original_vec = field.vectors[y][x];
          
          // 斥力を計算
          Vector2D repulsive_force(0.0, 0.0);
          if (static_obstacles_.has_value() && repulsive_force_calculator_) {
            repulsive_force = repulsive_force_calculator_->calculateTotalForce(
                world_pos, static_obstacles_.value());
          }
          
          // 合成ベクトルを計算
          double combined_vx = original_vec[0] + repulsive_force.x;
          double combined_vy = original_vec[1] + repulsive_force.y;
          
          // 合成ベクトルの可視化のみ
          if (std::abs(combined_vx) > 0.01 || std::abs(combined_vy) > 0.01)
          {
            auto combined_marker = visualization_msgs::msg::Marker();
            combined_marker.header.frame_id = global_frame;
            combined_marker.header.stamp = this->get_clock()->now();
            combined_marker.id = marker_id++;
            combined_marker.type = visualization_msgs::msg::Marker::ARROW;
            combined_marker.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point start, end;
            start.x = world_pos.x;
            start.y = world_pos.y;
            start.z = 0.0;  // Z座標を0に戻す
            
            // 合成ベクトルを正規化してから表示
            double magnitude = std::sqrt(combined_vx * combined_vx + combined_vy * combined_vy);
            double arrow_length = 0.4;  // 合成は少し長く
            
            if (magnitude > 0.01) {
              end.x = world_pos.x + (combined_vx / magnitude) * arrow_length;
              end.y = world_pos.y + (combined_vy / magnitude) * arrow_length;
            } else {
              end.x = world_pos.x + combined_vx * arrow_length;
              end.y = world_pos.y + combined_vy * arrow_length;
            }
            end.z = 0.0;
            
            combined_marker.points.push_back(start);
            combined_marker.points.push_back(end);
            
            combined_marker.scale.x = 0.05;  // シャフトの太さ
            combined_marker.scale.y = 0.1;   // ヘッドの太さ
            combined_marker.scale.z = 0.0;
            
            // 緑色で合成結果を表示
            combined_marker.color.r = 0.0;
            combined_marker.color.g = 1.0;
            combined_marker.color.b = 0.0;
            combined_marker.color.a = 0.9;
            
            combined_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(combined_marker);
          }
        }
      }
      
      // 統合された可視化を一つのトピックに配信
      vector_field_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Combined field visualization error: %s", e.what());
    }
  }

} // namespace tvvf_vo_c