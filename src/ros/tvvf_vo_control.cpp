#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c
{

  void TVVFVONode::control_loop()
  {
    try
    {
      // ロボット状態の更新
      robot_state_ = get_robot_pose_from_tf();

      // 状態チェック
      if (!robot_state_.has_value() || !goal_.has_value())
      {
        return;
      }

      // 目標到達チェック
      double distance_to_goal = robot_state_->position.distance_to(goal_->position);
      if (distance_to_goal < goal_->tolerance)
      {
        publish_stop_command();
        planned_path_.reset();
        goal_.reset();
        publish_empty_visualization();
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
      }

      // 動的・静的障害物を統合
      std::vector<DynamicObstacle> all_obstacles;
      all_obstacles.insert(all_obstacles.end(), dynamic_obstacles_.begin(), dynamic_obstacles_.end());
      all_obstacles.insert(all_obstacles.end(), static_obstacles_.begin(), static_obstacles_.end());

      // GlobalFieldGeneratorでベクトル場生成
      VectorField global_field;
      if (global_field_generator_ && current_map_.has_value()) {
        global_field = global_field_generator_->generateField(dynamic_obstacles_);
        
        // パフォーマンス統計をログ出力
        if (this->count_subscribers("tvvf_vo_vector_field") > 0) {
          double computation_time = global_field_generator_->getLastComputationTime();
          RCLCPP_DEBUG(this->get_logger(), "Global field computation time: %.3f ms", 
                       computation_time * 1000.0);
        }
      }

      // TVVF-VO制御更新
      ControlOutput control_output = controller_->update(
          robot_state_.value(), all_obstacles, goal_.value(), planned_path_);

      // 制御コマンド発行
      publish_control_command(control_output);

      // ベクトル場可視化
      if (this->get_parameter("enable_vector_field_viz").as_bool())
      {
        if (global_field.width > 0 && global_field.height > 0) {
          publish_global_field_visualization(global_field);
        } else {
          publish_vector_field_visualization();
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", e.what());
      publish_stop_command();
    }
  }

  void TVVFVONode::publish_control_command(const ControlOutput &control_output)
  {
    // ベクトル場からの速度指令を差動二輪の制約に変換
    double desired_vx = control_output.velocity_command.vx;
    double desired_vy = control_output.velocity_command.vy;

    auto [linear_x, angular_z] = convert_to_differential_drive(
        desired_vx, desired_vy, robot_state_->orientation);

    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = linear_x;
    cmd_msg.angular.z = angular_z;

    // 速度制限
    double max_linear_vel = this->get_parameter("max_linear_velocity").as_double();
    double max_angular_vel = this->get_parameter("max_angular_velocity").as_double();

    cmd_msg.linear.x = std::clamp(cmd_msg.linear.x, -max_linear_vel, max_linear_vel);
    cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -max_angular_vel, max_angular_vel);

    cmd_vel_pub_->publish(cmd_msg);
  }

  std::pair<double, double> TVVFVONode::convert_to_differential_drive(
      double desired_vx, double desired_vy, double current_orientation)
  {
    // 目標速度ベクトルの大きさと方向
    double target_speed = std::sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
    double target_angle = std::atan2(desired_vy, desired_vx);

    // 現在の姿勢との角度差
    double angle_diff = math_utils::normalize_angle(target_angle - current_orientation);
    
    // 固定の許容角度（簡略化）
    const double orientation_tolerance = 0.2;

    // 角度差が大きい場合は回転を優先
    double linear_velocity, angular_velocity;
    if (std::abs(angle_diff) > orientation_tolerance)
    {
      // 回転優先モード
      linear_velocity = target_speed * std::cos(angle_diff) * 0.3;
      angular_velocity = 2.0 * angle_diff;
    }
    else
    {
      // 前進優先モード
      linear_velocity = target_speed * std::cos(angle_diff);
      angular_velocity = 1.0 * angle_diff;
    }

    return {linear_velocity, angular_velocity};
  }

  void TVVFVONode::publish_stop_command()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd_msg);
  }

} // namespace tvvf_vo_c