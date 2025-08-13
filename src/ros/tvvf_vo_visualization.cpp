#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace tvvf_vo_c
{

  void TVVFVONode::publish_vector_field_visualization()
  {
    // 旧実装用の関数（使用されない）
    return;
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
    vector_field_pub_->publish(empty_marker_array);
  }

  void TVVFVONode::publish_global_field_visualization(const VectorField& field)
  {
    try
    {
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
          // ベクトル取得
          auto vec = field.vectors[y][x];
          
          // ゼロベクトルはスキップ
          if (std::abs(vec[0]) < 0.01 && std::abs(vec[1]) < 0.01) continue;
          
          // ワールド座標に変換
          Position world_pos = field.gridToWorld(x, y);
          
          // 矢印マーカーを作成
          auto marker = visualization_msgs::msg::Marker();
          marker.header.frame_id = global_frame;
          marker.header.stamp = this->get_clock()->now();
          marker.id = marker_id++;
          marker.type = visualization_msgs::msg::Marker::ARROW;
          marker.action = visualization_msgs::msg::Marker::ADD;
          
          // 矢印の始点と終点
          geometry_msgs::msg::Point start, end;
          start.x = world_pos.x;
          start.y = world_pos.y;
          start.z = 0.0;
          
          double arrow_length = 0.3;  // 矢印の長さ
          end.x = world_pos.x + vec[0] * arrow_length;
          end.y = world_pos.y + vec[1] * arrow_length;
          end.z = 0.0;
          
          marker.points.push_back(start);
          marker.points.push_back(end);
          
          // 矢印のスケール
          marker.scale.x = 0.05;  // シャフトの太さ
          marker.scale.y = 0.1;   // ヘッドの太さ
          marker.scale.z = 0.0;
          
          // 色（速度に基づく）
          double speed = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
          marker.color.r = speed;
          marker.color.g = 1.0 - speed;
          marker.color.b = 0.0;
          marker.color.a = 0.7;
          
          marker.lifetime = rclcpp::Duration::from_seconds(0.1);
          
          marker_array.markers.push_back(marker);
        }
      }
      
      vector_field_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Global field visualization error: %s", e.what());
    }
  }

  void TVVFVONode::log_performance_stats()
  {
    if (global_field_generator_)
    {
      double computation_time = global_field_generator_->getLastComputationTime();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Global field computation time: %.3f ms", 
                           computation_time * 1000.0);
    }
    
  }

} // namespace tvvf_vo_c