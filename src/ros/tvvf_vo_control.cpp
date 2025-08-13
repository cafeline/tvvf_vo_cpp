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
      if (!robot_state_.has_value())
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Cannot get robot pose from TF");
        return;
      }

      if (!goal_.has_value())
      {
        // ゴールが設定されていない場合は何もしない（これは正常）
        return;
      }

      // 目標到達チェック
      double distance_to_goal = robot_state_->position.distance_to(goal_->position);
      if (distance_to_goal < goal_->tolerance)
      {
        publish_stop_command();
        goal_.reset();
        publish_empty_visualization();
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        return;
      }

      // 動的障害物のみ使用（静的障害物はマップに含まれる）

      // GlobalFieldGeneratorでベクトル場生成
      ControlOutput control_output;

      if (global_field_generator_ && global_field_generator_->isStaticFieldReady()) {
        // GlobalFieldGeneratorから速度ベクトルを取得
        auto velocity_vector = global_field_generator_->getVelocityAt(
            robot_state_->position, dynamic_obstacles_);

        // デバッグ: 速度ベクトルを出力
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Global field velocity at robot: (%.6f, %.6f)",
                            velocity_vector[0], velocity_vector[1]);

        // 速度ベクトルをスケーリング（正規化されているので最大速度を乗算）
        double max_vel = this->get_parameter("max_linear_velocity").as_double();
        velocity_vector[0] *= max_vel;
        velocity_vector[1] *= max_vel;

        // ControlOutputを作成
        control_output = ControlOutput(
            Velocity(velocity_vector[0], velocity_vector[1]),
            0.01,  // computation_time
            0.01,  // dt
            1.0    // confidence
        );

        // ベクトル場の可視化用にフィールドを生成
        if (this->count_subscribers("tvvf_vo_vector_field") > 0) {
          VectorField global_field = global_field_generator_->generateField(dynamic_obstacles_);
          if (global_field.width > 0 && global_field.height > 0) {
            publish_global_field_visualization(global_field);
          }
        }
      } else {
        // GlobalFieldGeneratorが準備できていない場合は停止
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Global field not ready - stopping");
        control_output = ControlOutput(
            Velocity(0.0, 0.0),
            0.01,
            0.01,
            0.0
        );
      }

      // 制御コマンド発行
      publish_control_command(control_output);
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

    // デバッグ: 変換前後の値を出力
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Velocity conversion: (vx:%.6f, vy:%.6f) -> (linear:%.6f, angular:%.6f)",
                        desired_vx, desired_vy, linear_x, angular_z);

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