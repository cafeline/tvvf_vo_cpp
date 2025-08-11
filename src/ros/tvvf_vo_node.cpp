#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/ros/visualizer.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>

namespace tvvf_vo_c
{

  TVVFVONode::TVVFVONode() : Node("tvvf_vo_c_node")
  {
    // パラメータ設定
    setup_parameters();

    // TVVF-VO制御器初期化
    config_ = create_config_from_parameters();
    controller_ = std::make_unique<TVVFVOController>(config_);

    // TF2関連初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パブリッシャー初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_markers", 10);
    vector_field_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_vector_field", 10);
    path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_planned_path", 10);

    // サブスクライバー初期化
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&TVVFVONode::map_callback, this, std::placeholders::_1));

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&TVVFVONode::clicked_point_callback, this, std::placeholders::_1));

    // 障害物データをsubscribe（統合コールバック使用）
    dynamic_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "dynamic_obstacles", 10, 
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { obstacles_callback(msg, true); });

    static_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "static_obstacles", 10, 
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) { obstacles_callback(msg, false); });

    // adaptive_A_star からのA*経路データをsubscribe
    path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "planned_path", 10, std::bind(&TVVFVONode::path_callback, this, std::placeholders::_1));

    // タイマー初期化（制御ループ：20Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TVVFVONode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "TVVF-VO C++ Node initialized");
  }

  void TVVFVONode::setup_parameters()
  {
    // TVVF基本パラメータ
    this->declare_parameter("k_attraction", 1.0);
    this->declare_parameter("k_repulsion", 2.0);
    this->declare_parameter("influence_radius", 3.0);

    // 経路統合パラメータ
    this->declare_parameter("k_path_attraction", 2.0);
    this->declare_parameter("path_influence_radius", 2.0);
    this->declare_parameter("lookahead_distance", 1.5);

    // 障害物回避関連パラメータ
    this->declare_parameter("safety_margin", 0.2);

    // ロボット仕様パラメータ
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("robot_radius", 0.25);

    // フレーム名
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("global_frame", "map");

    // 制御関連
    this->declare_parameter("goal_tolerance", 0.1);

    // ベクトル場可視化パラメータ
    this->declare_parameter("enable_vector_field_viz", true);
    this->declare_parameter("vector_field_resolution", 0.5);
    this->declare_parameter("vector_field_range", 4.0);
  }

  TVVFVOConfig TVVFVONode::create_config_from_parameters()
  {
    TVVFVOConfig config;

    config.k_attraction = this->get_parameter("k_attraction").as_double();
    config.k_repulsion = this->get_parameter("k_repulsion").as_double();
    config.influence_radius = this->get_parameter("influence_radius").as_double();
    config.k_path_attraction = this->get_parameter("k_path_attraction").as_double();
    config.path_influence_radius = this->get_parameter("path_influence_radius").as_double();
    config.lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    config.safety_margin = this->get_parameter("safety_margin").as_double();
    config.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
    
    // デフォルト値設定（簡略化のため固定値）
    config.fluid_strength_factor = 1.0;
    config.repulsive_weight = 0.4;
    config.fluid_weight = 0.6;
    config.path_direction_weight = 1.0;
    config.enable_exponential_repulsion = false;
    config.exponential_base = 2.0;
    config.exponential_scale_factor = 1.5;
    config.max_exponential_distance = 1.5;
    config.exponential_smoothing_threshold = 0.1;
    config.max_computation_time = 0.05;

    return config;
  }

  std::optional<RobotState> TVVFVONode::get_robot_pose_from_tf()
  {
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

  void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Position goal_position(msg->point.x, msg->point.y);
    goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());
    planned_path_.reset();
    RCLCPP_INFO(this->get_logger(), "Goal set: (%.2f, %.2f)", goal_position.x, goal_position.y);
  }

  void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                msg->info.width, msg->info.height, msg->info.resolution);
  }

  void TVVFVONode::obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, bool is_dynamic)
  {
    auto& obstacles = is_dynamic ? dynamic_obstacles_ : static_obstacles_;
    obstacles.clear();

    for (const auto &marker : msg->markers)
    {
      if (marker.action == visualization_msgs::msg::Marker::ADD)
      {
        Position position(marker.pose.position.x, marker.pose.position.y);
        Velocity velocity(0.0, 0.0);
        double radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
        obstacles.emplace_back(marker.id, position, velocity, radius);
      }
    }
  }

  void TVVFVONode::path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty())
    {
      planned_path_.reset();
      RCLCPP_WARN(this->get_logger(), "Received empty path from external planner");
      return;
    }

    Path new_path;
    for (const auto& pose : msg->poses)
    {
      Position position(pose.position.x, pose.position.y);
      new_path.add_point(position, 0.0);
    }

    planned_path_ = new_path;
    RCLCPP_INFO(this->get_logger(), "Received external path: %zu points", planned_path_->size());
  }

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

      // TVVF-VO制御更新
      ControlOutput control_output = controller_->update(
          robot_state_.value(), all_obstacles, goal_.value(), planned_path_);

      // 制御コマンド発行
      publish_control_command(control_output);

      // ベクトル場可視化
      if (this->get_parameter("enable_vector_field_viz").as_bool())
      {
        publish_vector_field_visualization();
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

  void TVVFVONode::publish_vector_field_visualization()
  {
    try
    {
      if (!robot_state_.has_value() || !goal_.has_value())
      {
        return;
      }

      // パラメータから可視化設定を取得
      double grid_spacing = this->get_parameter("vector_field_resolution").as_double();
      double grid_range = this->get_parameter("vector_field_range").as_double();

      auto marker_array = visualization_msgs::msg::MarkerArray();
      std::string global_frame = this->get_parameter("global_frame").as_string();

      // 既存マーカーをクリア
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = global_frame;
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker.id = 0;
      marker_array.markers.push_back(delete_marker);

      // 動的・静的障害物を統合
      std::vector<DynamicObstacle> all_obstacles;
      all_obstacles.insert(all_obstacles.end(), dynamic_obstacles_.begin(), dynamic_obstacles_.end());
      all_obstacles.insert(all_obstacles.end(), static_obstacles_.begin(), static_obstacles_.end());

      // ベクトル場計算とマーカー生成
      if (!controller_) return;
      const auto &tvvf_generator = controller_->get_tvvf_generator();

      Position robot_pos = robot_state_->position;
      int marker_id = 0;

      for (double x = robot_pos.x - grid_range; x <= robot_pos.x + grid_range; x += grid_spacing)
      {
        for (double y = robot_pos.y - grid_range; y <= robot_pos.y + grid_range; y += grid_spacing)
        {
          Position grid_pos(x, y);
          
          // ベクトル場計算
          auto force = tvvf_generator.compute_vector(
              grid_pos, 0.0, goal_.value(), all_obstacles, planned_path_);

          double force_magnitude = std::sqrt(force[0] * force[0] + force[1] * force[1]);
          
          // 小さすぎる力は表示しない
          if (force_magnitude < 0.05) continue;

          // 矢印マーカー作成
          auto arrow_marker = visualization_msgs::msg::Marker();
          arrow_marker.header.frame_id = global_frame;
          arrow_marker.header.stamp = this->get_clock()->now();
          arrow_marker.id = marker_id++;
          arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
          arrow_marker.action = visualization_msgs::msg::Marker::ADD;

          // 矢印の位置
          arrow_marker.pose.position.x = x;
          arrow_marker.pose.position.y = y;
          arrow_marker.pose.position.z = 0.1;

          // 矢印の向き
          double yaw = std::atan2(force[1], force[0]);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          arrow_marker.pose.orientation.x = q.x();
          arrow_marker.pose.orientation.y = q.y();
          arrow_marker.pose.orientation.z = q.z();
          arrow_marker.pose.orientation.w = q.w();

          // 矢印のサイズ（固定長）
          arrow_marker.scale.x = grid_spacing * 0.6;  // 長さ
          arrow_marker.scale.y = 0.05;                // 幅
          arrow_marker.scale.z = 0.05;                // 高さ

          // 色（緑）
          arrow_marker.color.r = 0.0;
          arrow_marker.color.g = 1.0;
          arrow_marker.color.b = 0.0;
          arrow_marker.color.a = 0.8;

          arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);
          marker_array.markers.push_back(arrow_marker);
        }
      }

      vector_field_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Vector field visualization error: %s", e.what());
    }
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
    path_pub_->publish(empty_marker_array);
    vector_field_pub_->publish(empty_marker_array);
  }

} // namespace tvvf_vo_c