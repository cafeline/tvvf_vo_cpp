#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"

namespace tvvf_vo_c
{

  void TVVFVONode::publish_empty_visualization()
  {
    auto empty_marker_array = visualization_msgs::msg::MarkerArray();
    auto delete_marker = visualization_msgs::msg::Marker();
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = this->get_clock()->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.id = 0;
    empty_marker_array.markers.push_back(delete_marker);

    vector_field_pub_->publish(empty_marker_array);
  }

} // namespace tvvf_vo_c