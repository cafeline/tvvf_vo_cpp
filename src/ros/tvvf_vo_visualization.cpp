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
          // ワールド座標に変換
          Position world_pos = field.gridToWorld(x, y);
          
          // GlobalFieldGeneratorを使用して統合されたベクトルを取得
          // (マップ端斥力を含む)
          auto vec = global_field_generator_->getVelocityAt(world_pos, dynamic_obstacles_);
          
          // NaNチェックとゼロベクトルのスキップ
          if (std::isnan(vec[0]) || std::isnan(vec[1])) continue;
          if (std::abs(vec[0]) < 0.01 && std::abs(vec[1]) < 0.01) continue;
          
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
          
          // 斥力が強い場所を赤く表示
          // 通常の引力場を緑で表示
          // マップ端からの距離を計算して色を決定
          auto [grid_x, grid_y] = field.worldToGrid(world_pos);
          bool near_obstacle = false;
          
          // 周囲に障害物があるかチェック
          for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
              int nx = grid_x + dx;
              int ny = grid_y + dy;
              if (nx < 0 || nx >= field.width || ny < 0 || ny >= field.height) {
                near_obstacle = true;
                break;
              }
              if (field.grid[ny][nx].is_obstacle) {
                near_obstacle = true;
                break;
              }
            }
            if (near_obstacle) break;
          }
          
          if (near_obstacle) {
            // 障害物近くは赤系の色
            marker.color.r = 0.9;
            marker.color.g = 0.2;
            marker.color.b = 0.1;
          } else {
            // 通常のベクトル場は緑系
            marker.color.r = 0.1;
            marker.color.g = 0.8;
            marker.color.b = 0.2;
          }
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
    
  }

} // namespace tvvf_vo_c