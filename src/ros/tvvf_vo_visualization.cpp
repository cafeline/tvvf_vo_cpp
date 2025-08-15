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
    // この関数は現在使用されていません
    // publish_combined_field_visualizationが全ての可視化を担当します
  }

  void TVVFVONode::log_performance_stats()
  {
    
  }

} // namespace tvvf_vo_c