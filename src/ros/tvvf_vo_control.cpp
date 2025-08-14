#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace tvvf_vo_c
{

  void TVVFVONode::control_loop()
  {
    // 処理時間計測開始
    auto loop_start = std::chrono::high_resolution_clock::now();
    
    try
    {
      // ロボット状態の更新
      auto tf_start = std::chrono::high_resolution_clock::now();
      robot_state_ = get_robot_pose_from_tf();
      auto tf_end = std::chrono::high_resolution_clock::now();

      // 状態チェック
      if (!robot_state_.has_value())
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Cannot get robot pose from TF");
        
        // 処理時間出力
        auto loop_end = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        RCLCPP_INFO(this->get_logger(), "[TVVF-VO] Loop time: %.2f ms (TF failed)", total_time);
        return;
      }

      if (!goal_.has_value())
      {
        // ゴールが設定されていない場合は何もしない（これは正常）
        auto loop_end = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        double tf_time = std::chrono::duration<double, std::milli>(tf_end - tf_start).count();
        RCLCPP_INFO(this->get_logger(), "[TVVF-VO] Loop time: %.2f ms (No goal, TF: %.2f ms)", 
                    total_time, tf_time);
        return;
      }

      // 目標到達チェック
      double distance_to_goal = robot_state_->position.distance_to(goal_->position);
      if (distance_to_goal < goal_->tolerance)
      {
        publish_stop_command();
        goal_.reset();
        publish_empty_visualization();
        
        // 処理時間出力
        auto loop_end = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
        RCLCPP_INFO(this->get_logger(), "[TVVF-VO] Loop time: %.2f ms (Goal reached)", total_time);
        return;
      }

      // 動的障害物のみ使用（静的障害物はマップに含まれる）

      // GlobalFieldGeneratorでベクトル場生成
      ControlOutput control_output;

      auto field_start = std::chrono::high_resolution_clock::now();
      auto field_end = field_start;  // デフォルト値
      auto vis_start = field_start;  // デフォルト値
      auto vis_end = vis_start;  // デフォルト値
      if (global_field_generator_ && global_field_generator_->isStaticFieldReady()) {
        // GlobalFieldGeneratorから速度ベクトルを取得
        auto velocity_vector = global_field_generator_->getVelocityAt(
            robot_state_->position, dynamic_obstacles_);
        field_end = std::chrono::high_resolution_clock::now();


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

        // ベクトル場の可視化用にフィールドを生成（指定間隔でのみ実行）
        vis_start = std::chrono::high_resolution_clock::now();
        visualization_counter_++;
        if (visualization_counter_ >= visualization_interval_) {
          visualization_counter_ = 0;  // カウンタリセット
          if (this->count_subscribers("tvvf_vo_vector_field") > 0) {
            VectorField global_field = global_field_generator_->generateField(dynamic_obstacles_);
            if (global_field.width > 0 && global_field.height > 0) {
              publish_global_field_visualization(global_field);
            }
          }
        }
        vis_end = std::chrono::high_resolution_clock::now();
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
      auto cmd_start = std::chrono::high_resolution_clock::now();
      publish_control_command(control_output);
      auto cmd_end = std::chrono::high_resolution_clock::now();
      
      // 処理時間出力（毎周期）
      auto loop_end = std::chrono::high_resolution_clock::now();
      double total_time = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
      double tf_time = std::chrono::duration<double, std::milli>(tf_end - tf_start).count();
      double field_time = 0.0;
      double vis_time = 0.0;
      if (global_field_generator_ && global_field_generator_->isStaticFieldReady()) {
        field_time = std::chrono::duration<double, std::milli>(field_end - field_start).count();
        vis_time = std::chrono::duration<double, std::milli>(vis_end - vis_start).count();
      }
      double cmd_time = std::chrono::duration<double, std::milli>(cmd_end - cmd_start).count();
      
      RCLCPP_INFO(this->get_logger(), 
                  "[TVVF-VO] Loop: %.2f ms (TF: %.2f, Field: %.2f, Vis: %.2f, Cmd: %.2f) Dist: %.2f m", 
                  total_time, tf_time, field_time, vis_time, cmd_time, distance_to_goal);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", e.what());
      publish_stop_command();
      
      // エラー時も処理時間出力
      auto loop_end = std::chrono::high_resolution_clock::now();
      double total_time = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
      RCLCPP_INFO(this->get_logger(), "[TVVF-VO] Loop time: %.2f ms (Error)", total_time);
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