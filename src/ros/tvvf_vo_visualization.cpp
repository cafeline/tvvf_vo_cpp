#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <cmath>

namespace tvvf_vo_c
{
  using namespace visualization_constants;

  void TVVFVONode::publish_empty_visualization()
  {
    auto empty_marker_array = visualization_msgs::msg::MarkerArray();
    auto delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = this->get_parameter("global_frame").as_string();
    delete_marker.header.stamp = this->get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.id = 0;
    empty_marker_array.markers.push_back(delete_marker);

    vector_field_pub_->publish(empty_marker_array);
  }

  void TVVFVONode::publish_combined_field_visualization(const VectorField& field)
  {
    try
    {
      auto marker_array = visualization_msgs::msg::MarkerArray();
      const std::string global_frame = this->get_parameter("global_frame").as_string();

      // 既存マーカーをクリア
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = global_frame;
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker.id = 0;
      marker_array.markers.push_back(delete_marker);

      int marker_id = 1;
      
      // グリッドをサンプリングして可視化
      for (int y = 0; y < field.height; y += DEFAULT_GRID_STRIDE)
      {
        for (int x = 0; x < field.width; x += DEFAULT_GRID_STRIDE)
        {
          // ワールド座標に変換
          const Position world_pos = field.gridToWorld(x, y);
          
          // 合成ベクトルを計算
          const auto combined_vector = calculate_combined_vector(field.vectors[y][x], world_pos);
          
          // ベクトルが小さすぎる場合はスキップ
          if (!should_visualize_vector(combined_vector)) {
            continue;
          }
          
          // 矢印マーカーを作成して追加
          auto arrow_marker = create_arrow_marker(world_pos, combined_vector, marker_id++, global_frame);
          marker_array.markers.push_back(arrow_marker);
        }
      }
      
      vector_field_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Combined field visualization error: %s", e.what());
    }
  }

  // ヘルパー関数: 合成ベクトルを計算
  std::array<double, 2> TVVFVONode::calculate_combined_vector(
      const std::array<double, 2>& original_vector, 
      const Position& world_pos) const
  {
    // 元のベクトル場の値
    std::array<double, 2> combined = original_vector;
    
    // 斥力を追加
    if (static_obstacles_.has_value() && repulsive_force_calculator_) {
      const auto repulsive_force = repulsive_force_calculator_->calculateTotalForce(
          world_pos, static_obstacles_.value());
      combined[0] += repulsive_force.x;
      combined[1] += repulsive_force.y;
    }
    
    return combined;
  }

  // ヘルパー関数: ベクトルを可視化すべきか判定
  bool TVVFVONode::should_visualize_vector(const std::array<double, 2>& vector) const
  {
    return std::abs(vector[0]) > MIN_VECTOR_MAGNITUDE || 
           std::abs(vector[1]) > MIN_VECTOR_MAGNITUDE;
  }

  // ヘルパー関数: 矢印マーカーを作成
  visualization_msgs::msg::Marker TVVFVONode::create_arrow_marker(
      const Position& position,
      const std::array<double, 2>& vector,
      int marker_id,
      const std::string& frame_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 矢印の始点と終点を設定
    geometry_msgs::msg::Point start, end;
    start.x = position.x;
    start.y = position.y;
    start.z = 0.0;
    
    // ベクトルを正規化してから矢印の長さを適用
    const double magnitude = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (magnitude > MIN_VECTOR_MAGNITUDE) {
      end.x = position.x + (vector[0] / magnitude) * DEFAULT_ARROW_LENGTH;
      end.y = position.y + (vector[1] / magnitude) * DEFAULT_ARROW_LENGTH;
    } else {
      end.x = position.x + vector[0] * DEFAULT_ARROW_LENGTH;
      end.y = position.y + vector[1] * DEFAULT_ARROW_LENGTH;
    }
    end.z = 0.0;
    
    marker.points.push_back(start);
    marker.points.push_back(end);
    
    // 矢印のスケール設定
    marker.scale.x = ARROW_SHAFT_WIDTH;
    marker.scale.y = ARROW_HEAD_WIDTH;
    marker.scale.z = 0.0;
    
    // 速度に基づく色設定（赤〜緑のグラデーション）
    const double normalized_magnitude = std::min(magnitude, 1.0);
    marker.color.r = normalized_magnitude;
    marker.color.g = 1.0 - normalized_magnitude;
    marker.color.b = 0.0;
    marker.color.a = ARROW_ALPHA;
    
    marker.lifetime = rclcpp::Duration::from_seconds(MARKER_LIFETIME_SEC);
    
    return marker;
  }

} // namespace tvvf_vo_c