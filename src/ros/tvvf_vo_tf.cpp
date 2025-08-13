#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tvvf_vo_c
{

  std::optional<RobotState> TVVFVONode::get_robot_pose_from_tf()
  {
    // デバッグ/テスト用: 固定ロボット位置を使用
    if (this->get_parameter("use_fixed_robot_pose").as_bool())
    {
      Position position(
        this->get_parameter("fixed_robot_x").as_double(),
        this->get_parameter("fixed_robot_y").as_double()
      );
      double yaw = this->get_parameter("fixed_robot_theta").as_double();
      Velocity velocity(0.0, 0.0);
      
      RobotState robot_state(
        position, velocity, yaw,
        this->get_parameter("max_linear_velocity").as_double(),
        1.0,  // max_acceleration固定値
        this->get_parameter("robot_radius").as_double()
      );
      
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Using fixed robot pose: (%.2f, %.2f, %.2f)",
                           position.x, position.y, yaw);
      return robot_state;
    }
    
    try
    {
      std::string base_frame = this->get_parameter("base_frame").as_string();
      std::string global_frame = this->get_parameter("global_frame").as_string();

      // TF取得
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform(
          global_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));

      // 位置の取得
      Position position(transform_stamped.transform.translation.x,
                        transform_stamped.transform.translation.y);

      // 姿勢角の取得
      tf2::Quaternion q(
          transform_stamped.transform.rotation.x,
          transform_stamped.transform.rotation.y,
          transform_stamped.transform.rotation.z,
          transform_stamped.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // 速度計算（簡易版）
      Velocity velocity(0.0, 0.0);

      // ロボット状態作成
      RobotState robot_state(
          position, velocity, yaw,
          this->get_parameter("max_linear_velocity").as_double(),
          1.0,  // max_acceleration固定値
          this->get_parameter("robot_radius").as_double());

      return robot_state;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

} // namespace tvvf_vo_c