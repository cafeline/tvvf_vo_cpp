#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/ros/visualizer.hpp"
#include "tvvf_vo_c/utils/time_utils.hpp"
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

    // 状態変数初期化
    last_planning_time_ = 0.0;
    planning_interval_ = 2.0;
    last_update_time_ = time_utils::get_current_time();

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

    // obstacle_tracker からの障害物データをsubscribe
    dynamic_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "dynamic_obstacles", 10, std::bind(&TVVFVONode::dynamic_obstacles_callback, this, std::placeholders::_1));

    static_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "static_obstacles", 10, std::bind(&TVVFVONode::static_obstacles_callback, this, std::placeholders::_1));

    // adaptive_A_star からのA*経路データをsubscribe
    path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "planned_path", 10, std::bind(&TVVFVONode::path_callback, this, std::placeholders::_1));

    // タイマー初期化（制御ループ：20Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TVVFVONode::control_loop, this));

    // パフォーマンス統計タイマー（5秒間隔で出力）
    profile_timer_ = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&TVVFVONode::publish_performance_stats, this));

    RCLCPP_INFO(this->get_logger(), "TVVF-VO C++ Node initialized");
  }

  void TVVFVONode::setup_parameters()
  {
    // TVVF基本パラメータ
    this->declare_parameter("k_attraction", 1.0);
    this->declare_parameter("k_repulsion", 2.0);
    this->declare_parameter("influence_radius", 3.0);

    // A*経路統合パラメータ
    this->declare_parameter("k_path_attraction", 2.0);
    this->declare_parameter("path_influence_radius", 2.0);
    this->declare_parameter("lookahead_distance", 1.5);
    this->declare_parameter("path_smoothing_factor", 0.8);
    this->declare_parameter("wall_clearance_distance", 0.8);
    this->declare_parameter("enable_dynamic_replanning", false);

    // VO関連パラメータ
    this->declare_parameter("time_horizon", 3.0);
    this->declare_parameter("safety_margin", 0.2);
    this->declare_parameter("vo_resolution", 0.1);
    
    // 流体ベクトル場専用パラメータ
    this->declare_parameter("fluid_influence_radius", 1.5);
    this->declare_parameter("fluid_strength_factor", 1.0);
    this->declare_parameter("repulsive_weight", 0.4);
    this->declare_parameter("fluid_weight", 0.6);
    this->declare_parameter("path_direction_weight", 1.0);
    
    // 距離別重み調整パラメータ
    this->declare_parameter("near_repulsive_weight", 0.7);
    this->declare_parameter("near_fluid_weight", 0.2);
    this->declare_parameter("near_path_weight", 0.1);
    this->declare_parameter("mid_repulsive_weight", 0.4);
    this->declare_parameter("mid_fluid_weight", 0.4);
    this->declare_parameter("mid_path_weight", 0.2);

    // ロボット仕様パラメータ
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("max_acceleration", 1.0);
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("wheel_base", 0.5);
    this->declare_parameter("orientation_tolerance", 0.2);

    // フレーム名
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("global_frame", "map");

    // 制御関連
    this->declare_parameter("goal_tolerance", 0.1);
    this->declare_parameter("max_computation_time", 0.05);

    // 可視化
    this->declare_parameter("enable_visualization", true);
    this->declare_parameter("enable_vector_field_viz", true);
    this->declare_parameter("vector_field_resolution", 0.5);
    this->declare_parameter("vector_field_range", 4.0);
    this->declare_parameter("vector_scale_factor", 0.3);
    this->declare_parameter("max_vector_points", 500);
    this->declare_parameter("min_vector_magnitude", 0.05);
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
    config.path_smoothing_factor = this->get_parameter("path_smoothing_factor").as_double();
    config.time_horizon = this->get_parameter("time_horizon").as_double();
    config.safety_margin = this->get_parameter("safety_margin").as_double();
    config.vo_resolution = this->get_parameter("vo_resolution").as_double();
    config.fluid_influence_radius = this->get_parameter("fluid_influence_radius").as_double();
    config.fluid_strength_factor = this->get_parameter("fluid_strength_factor").as_double();
    config.repulsive_weight = this->get_parameter("repulsive_weight").as_double();
    config.fluid_weight = this->get_parameter("fluid_weight").as_double();
    config.path_direction_weight = this->get_parameter("path_direction_weight").as_double();
    
    // 距離別重み調整パラメータ
    config.near_repulsive_weight = this->get_parameter("near_repulsive_weight").as_double();
    config.near_fluid_weight = this->get_parameter("near_fluid_weight").as_double();
    config.near_path_weight = this->get_parameter("near_path_weight").as_double();
    config.mid_repulsive_weight = this->get_parameter("mid_repulsive_weight").as_double();
    config.mid_fluid_weight = this->get_parameter("mid_fluid_weight").as_double();
    config.mid_path_weight = this->get_parameter("mid_path_weight").as_double();
    
    config.max_computation_time = this->get_parameter("max_computation_time").as_double();

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
      Velocity velocity(0.0, 0.0); // TODO: 速度推定の実装

      // ロボット状態作成
      RobotState robot_state(
          position, velocity, yaw,
          this->get_parameter("max_linear_velocity").as_double(),
          this->get_parameter("max_acceleration").as_double(),
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
    try
    {
      std::string global_frame = this->get_parameter("global_frame").as_string();

      Position goal_position(msg->point.x, msg->point.y);
      goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());

      // 既存の経路をクリア
      planned_path_.reset();

      RCLCPP_INFO(this->get_logger(), "Goal set: (%.2f, %.2f)",
                  goal_position.x, goal_position.y);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Clicked point callback error: %s", e.what());
    }
  }

  void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    try
    {
      occupancy_grid_ = *msg;

      // A*経路計画器を初期化
      Position origin(msg->info.origin.position.x, msg->info.origin.position.y);
      double wall_clearance = this->get_parameter("wall_clearance_distance").as_double();

      path_planner_ = std::make_unique<AStarPathPlanner>(*msg, msg->info.resolution, origin, wall_clearance);

      RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                  msg->info.width, msg->info.height, msg->info.resolution);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Map callback error: %s", e.what());
    }
  }

  void TVVFVONode::dynamic_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    try
    {
      dynamic_obstacles_.clear();

      for (const auto &marker : msg->markers)
      {
        if (marker.action == visualization_msgs::msg::Marker::ADD)
        {
          // マーカーから障害物情報を抽出
          Position position(marker.pose.position.x, marker.pose.position.y);
          Velocity velocity(0.0, 0.0); // 速度情報は予測しないため0
          double radius = std::max(marker.scale.x, marker.scale.y) / 2.0;

          dynamic_obstacles_.emplace_back(
              marker.id,
              position,
              velocity,
              radius);
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Dynamic obstacles callback error: %s", e.what());
    }
  }

  void TVVFVONode::static_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    try
    {
      static_obstacles_.clear();

      for (const auto &marker : msg->markers)
      {
        if (marker.action == visualization_msgs::msg::Marker::ADD)
        {
          // マーカーから障害物情報を抽出
          Position position(marker.pose.position.x, marker.pose.position.y);
          Velocity velocity(0.0, 0.0); // 静的障害物の速度は0
          double radius = std::max(marker.scale.x, marker.scale.y) / 2.0;

          static_obstacles_.emplace_back(
              marker.id,
              position,
              velocity,
              radius);
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Static obstacles callback error: %s", e.what());
    }
  }

  void TVVFVONode::path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    try
    {
      if (msg->poses.empty())
      {
        planned_path_.reset();
        RCLCPP_WARN(this->get_logger(), "Received empty path from adaptive_A_star");
        return;
      }

      // PoseArrayをPathに変換
      Path new_path;
      for (const auto& pose : msg->poses)
      {
        Position position(pose.position.x, pose.position.y);
        new_path.add_point(position, 0.0);
      }

      planned_path_ = new_path;
      
      RCLCPP_INFO(this->get_logger(), "Received A* path from adaptive_A_star: %zu points", 
                  planned_path_->size());
      
      // 経路を受け取ったら可視化
      publish_path_visualization();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Path callback error: %s", e.what());
    }
  }

  void TVVFVONode::plan_path_to_goal()
  {
    PROFILE_MODULE(profiler_, "A*_PathPlanning");

    try
    {
      if (!path_planner_ || !robot_state_.has_value() || !goal_.has_value())
      {
        RCLCPP_WARN(this->get_logger(), "Path planning: missing requirements");
        return;
      }

      // A*アルゴリズムで経路計画
      double start_time = time_utils::get_current_time();
      auto planned_path = path_planner_->plan_path(robot_state_->position, goal_->position);
      double planning_time = (time_utils::get_current_time() - start_time) * 1000; // ms

      if (planned_path.has_value())
      {
        planned_path_ = planned_path.value();
        RCLCPP_INFO(this->get_logger(), "A* Path Planning - Points: %zu, Time: %.1f ms, Freq: %.1f Hz",
                    planned_path_->size(), planning_time, 1000.0 / planning_time);
        publish_path_visualization();
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "A* Path planning failed after %.1f ms", planning_time);
        planned_path_.reset();
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Path planning error: %s", e.what());
      planned_path_.reset();
    }
  }

  void TVVFVONode::control_loop()
  {
    PROFILE_MODULE(profiler_, "ControlLoop_Total");

    try
    {
      // ロボット状態の更新
      {
        PROFILE_MODULE(profiler_, "TF_Lookup");
        robot_state_ = get_robot_pose_from_tf();
      }

      // 状態チェック
      if (!robot_state_.has_value() || !goal_.has_value())
      {
        return;
      }

      // A*経路は外部のadaptive_A_starパッケージから受け取るため、内部プランニングは無効化
      // if (!planned_path_.has_value() && path_planner_)
      // {
      //   RCLCPP_INFO(this->get_logger(), "Planning A* path...");
      //   plan_path_to_goal();
      // }

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
      {
        PROFILE_MODULE(profiler_, "ObstacleProcessing");
        all_obstacles.insert(all_obstacles.end(), dynamic_obstacles_.begin(), dynamic_obstacles_.end());
        all_obstacles.insert(all_obstacles.end(), static_obstacles_.begin(), static_obstacles_.end());
      }

      // TVVF-VO制御更新
      ControlOutput control_output;
      {
        PROFILE_MODULE(profiler_, "TVVF_VO_Control");
        control_output = controller_->update(
            robot_state_.value(), all_obstacles, goal_.value(), planned_path_);
      }

      // 制御コマンド発行
      {
        PROFILE_MODULE(profiler_, "PublishControl");
        publish_control_command(control_output);
      }

      // 可視化
      if (this->get_parameter("enable_visualization").as_bool())
      {
        PROFILE_MODULE(profiler_, "Visualization");
        publish_visualization();
      }

      // ベクトル場可視化（毎フレーム実行でチカチカを防止）
      if (this->get_parameter("enable_vector_field_viz").as_bool())
      {
        PROFILE_MODULE(profiler_, "VectorField_Viz");
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
    double orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();

    // 角度差が大きい場合は回転を優先
    double linear_velocity, angular_velocity;
    if (std::abs(angle_diff) > orientation_tolerance)
    {
      // 回転優先モード
      linear_velocity = target_speed * std::cos(angle_diff) * 0.3; // 減速
      angular_velocity = 2.0 * angle_diff;                         // 角度差に比例した角速度
    }
    else
    {
      // 前進優先モード
      linear_velocity = target_speed * std::cos(angle_diff);
      angular_velocity = 1.0 * angle_diff; // 小さな補正
    }

    return {linear_velocity, angular_velocity};
  }

  void TVVFVONode::publish_stop_command()
  {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd_msg);
  }

  void TVVFVONode::publish_visualization()
  {
    try
    {
      auto marker_array = visualization_msgs::msg::MarkerArray();

      if (goal_.has_value())
      {
        auto goal_marker = create_goal_marker(0);
        marker_array.markers.push_back(goal_marker);
      }

      // 現在の目標点（lookahead point）を可視化
      if (robot_state_.has_value() && planned_path_.has_value() && !planned_path_->empty())
      {
        try
        {
          size_t closest_idx = 0;
          double min_distance = std::numeric_limits<double>::max();

          // 最も近い経路点を見つける
          for (size_t i = 0; i < planned_path_->points.size(); ++i)
          {
            double distance = robot_state_->position.distance_to(planned_path_->points[i].position);
            if (distance < min_distance)
            {
              min_distance = distance;
              closest_idx = i;
            }
          }

          // 先読み点を計算（簡易版）
          double lookahead_distance = this->get_parameter("lookahead_distance").as_double();
          double accumulated_distance = 0.0;
          size_t target_idx = closest_idx;

          for (size_t i = closest_idx; i < planned_path_->points.size() - 1; ++i)
          {
            double segment_distance = planned_path_->points[i].position.distance_to(
                planned_path_->points[i + 1].position);
            accumulated_distance += segment_distance;

            if (accumulated_distance >= lookahead_distance)
            {
              target_idx = i + 1;
              break;
            }
          }

          // 現在の目標点マーカー
          auto target_marker = visualization_msgs::msg::Marker();
          target_marker.header.frame_id = this->get_parameter("global_frame").as_string();
          target_marker.header.stamp = this->get_clock()->now();
          target_marker.id = 1;
          target_marker.type = visualization_msgs::msg::Marker::CYLINDER;
          target_marker.action = visualization_msgs::msg::Marker::ADD;

          target_marker.pose.position.x = planned_path_->points[target_idx].position.x;
          target_marker.pose.position.y = planned_path_->points[target_idx].position.y;
          target_marker.pose.position.z = 0.0;
          target_marker.pose.orientation.w = 1.0;

          target_marker.scale.x = 0.3; // 直径30cm
          target_marker.scale.y = 0.3;
          target_marker.scale.z = 0.15; // 高さ15cm

          target_marker.color.r = 1.0; // 黄色
          target_marker.color.g = 1.0;
          target_marker.color.b = 0.0;
          target_marker.color.a = 0.7;
          target_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

          marker_array.markers.push_back(target_marker);
        }
        catch (const std::exception &e)
        {
        }
      }

      marker_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Visualization error: %s", e.what());
    }
  }

  void TVVFVONode::publish_path_visualization()
  {
    try
    {
      if (!planned_path_.has_value() || planned_path_->empty())
      {
        return;
      }

      auto marker_array = visualization_msgs::msg::MarkerArray();
      std::string global_frame = this->get_parameter("global_frame").as_string();

      // 線分マーカー（経路全体）
      auto line_marker = visualization_msgs::msg::Marker();
      line_marker.header.frame_id = global_frame;
      line_marker.header.stamp = this->get_clock()->now();
      line_marker.id = 0;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;

      // 経路の点を追加
      for (const auto &path_point : planned_path_->points)
      {
        geometry_msgs::msg::Point point;
        point.x = path_point.position.x;
        point.y = path_point.position.y;
        point.z = 0.05; // 地面から少し浮かせる
        line_marker.points.push_back(point);
      }

      // 線の設定
      line_marker.scale.x = 0.03; // 線の太さ
      line_marker.color.r = 1.0;  // 赤色
      line_marker.color.g = 0.0;
      line_marker.color.b = 0.0;
      line_marker.color.a = 0.6; // 透明度
      line_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

      marker_array.markers.push_back(line_marker);

      // A*経路点を個別に球体マーカーで可視化
      for (size_t i = 0; i < planned_path_->points.size(); ++i)
      {
        auto point_marker = visualization_msgs::msg::Marker();
        point_marker.header.frame_id = global_frame;
        point_marker.header.stamp = this->get_clock()->now();
        point_marker.id = static_cast<int>(i + 1); // line_markerと重複しないようにi+1
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置設定
        point_marker.pose.position.x = planned_path_->points[i].position.x;
        point_marker.pose.position.y = planned_path_->points[i].position.y;
        point_marker.pose.position.z = 0.1; // 線より少し高く
        point_marker.pose.orientation.w = 1.0;

        // サイズ設定
        point_marker.scale.x = 0.1; // 直径10cm
        point_marker.scale.y = 0.1;
        point_marker.scale.z = 0.1;

        // 色設定（開始点、中間点、終了点で色分け）
        if (i == 0)
        {
          // 開始点：青色
          point_marker.color.r = 0.0;
          point_marker.color.g = 0.0;
          point_marker.color.b = 1.0;
          point_marker.color.a = 1.0;
        }
        else if (i == planned_path_->points.size() - 1)
        {
          // 終了点：緑色
          point_marker.color.r = 0.0;
          point_marker.color.g = 1.0;
          point_marker.color.b = 0.0;
          point_marker.color.a = 1.0;
        }
        else
        {
          // 中間点：オレンジ色
          point_marker.color.r = 1.0;
          point_marker.color.g = 0.5;
          point_marker.color.b = 0.0;
          point_marker.color.a = 0.8;
        }
        
        point_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム
        marker_array.markers.push_back(point_marker);
      }
      path_pub_->publish(marker_array);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Path visualization error: %s", e.what());
    }
  }

  void TVVFVONode::publish_empty_visualization()
  {
    try
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
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Empty visualization error: %s", e.what());
    }
  }

  void TVVFVONode::publish_vector_field_visualization()
  {
    try
    {
      if (!robot_state_.has_value() || !goal_.has_value())
      {
        return;
      }

      // 統合ベクトル場計算用のグリッドを作成
      std::vector<Position> grid_positions;
      std::vector<std::array<double, 2>> grid_forces;

      // パラメータから可視化設定を取得
      double grid_spacing = this->get_parameter("vector_field_resolution").as_double();
      double grid_range = this->get_parameter("vector_field_range").as_double();
      double scale_factor = this->get_parameter("vector_scale_factor").as_double();
      int max_points = this->get_parameter("max_vector_points").as_int();
      double min_magnitude = this->get_parameter("min_vector_magnitude").as_double();

      Position robot_pos = robot_state_->position;

      // 動的・静的障害物を統合
      std::vector<DynamicObstacle> all_obstacles;
      all_obstacles.insert(all_obstacles.end(), dynamic_obstacles_.begin(), dynamic_obstacles_.end());
      all_obstacles.insert(all_obstacles.end(), static_obstacles_.begin(), static_obstacles_.end());

      // ロボット周囲にグリッドを配置してTVVFを計算
      for (double x = robot_pos.x - grid_range; x <= robot_pos.x + grid_range; x += grid_spacing)
      {
        for (double y = robot_pos.y - grid_range; y <= robot_pos.y + grid_range; y += grid_spacing)
        {
          Position grid_pos(x, y);

          // TVVFGeneratorを使用して統合ベクトル場を計算
          if (controller_)
          {
            const auto &tvvf_generator = controller_->get_tvvf_generator();
            auto integrated_force = tvvf_generator.compute_vector(
                grid_pos, 0.0, goal_.value(), all_obstacles, planned_path_);

            // 力が十分大きい場合のみ可視化対象とする
            double force_magnitude = std::sqrt(integrated_force[0] * integrated_force[0] +
                                               integrated_force[1] * integrated_force[1]);
            if (force_magnitude > min_magnitude)
            {
              grid_positions.push_back(grid_pos);
              grid_forces.push_back(integrated_force);
            }
          }
        }
      }

      // 最大点数制限を適用
      if (static_cast<int>(grid_positions.size()) > max_points)
      {
        // 等間隔で間引く
        int step = grid_positions.size() / max_points;
        std::vector<Position> limited_positions;
        std::vector<std::array<double, 2>> limited_forces;

        for (size_t i = 0; i < grid_positions.size(); i += step)
        {
          limited_positions.push_back(grid_positions[i]);
          limited_forces.push_back(grid_forces[i]);
        }

        grid_positions = std::move(limited_positions);
        grid_forces = std::move(limited_forces);
      }

      // 統合ベクトル場マーカーを作成・発行
      if (!grid_positions.empty())
      {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        std::string global_frame = this->get_parameter("global_frame").as_string();
        
        // 最初に既存マーカーをクリア
        auto delete_marker = visualization_msgs::msg::Marker();
        delete_marker.header.frame_id = global_frame;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.id = 0;
        marker_array.markers.push_back(delete_marker);

        for (size_t i = 0; i < grid_positions.size(); ++i)
        {
          auto arrow_marker = visualization_msgs::msg::Marker();
          arrow_marker.header.frame_id = global_frame;
          arrow_marker.header.stamp = this->get_clock()->now();
          arrow_marker.id = static_cast<int>(i);
          arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
          arrow_marker.action = visualization_msgs::msg::Marker::ADD;

          // 矢印の開始点
          arrow_marker.pose.position.x = grid_positions[i].x;
          arrow_marker.pose.position.y = grid_positions[i].y;
          arrow_marker.pose.position.z = 0.1;

          // 矢印の方向（ベクトル場の方向）
          double force_magnitude = std::sqrt(grid_forces[i][0] * grid_forces[i][0] +
                                             grid_forces[i][1] * grid_forces[i][1]);
          if (force_magnitude > 1e-6)
          {
            double yaw = std::atan2(grid_forces[i][1], grid_forces[i][0]);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            arrow_marker.pose.orientation.x = q.x();
            arrow_marker.pose.orientation.y = q.y();
            arrow_marker.pose.orientation.z = q.z();
            arrow_marker.pose.orientation.w = q.w();
          }

          // 矢印のサイズ（力の大きさとスケール係数に比例）
          double arrow_length = std::min(1.0, force_magnitude * scale_factor);
          arrow_marker.scale.x = arrow_length; // 長さ
          arrow_marker.scale.y = 0.05;         // 幅
          arrow_marker.scale.z = 0.05;         // 高さ

          // 色設定（統合ベクトル場は緑色）
          arrow_marker.color.r = 0.0;
          arrow_marker.color.g = 1.0;
          arrow_marker.color.b = 0.0;
          arrow_marker.color.a = 0.8;

          // 生存時間（無限に設定）
          arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

          marker_array.markers.push_back(arrow_marker);
        }

        vector_field_pub_->publish(marker_array);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Vector field visualization error: %s", e.what());
    }
  }

  visualization_msgs::msg::Marker TVVFVONode::create_goal_marker(int marker_id)
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = this->get_parameter("global_frame").as_string();
    marker.header.stamp = this->get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal_->position.x;
    marker.pose.position.y = goal_->position.y;
    marker.pose.position.z = 0.0;

    marker.scale.x = goal_->tolerance * 2;
    marker.scale.y = goal_->tolerance * 2;
    marker.scale.z = 0.05;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 無限ライフタイム

    return marker;
  }

  void TVVFVONode::publish_performance_stats()
  {
    try
    {
      auto all_stats = profiler_.get_all_stats();

      if (all_stats.empty())
      {
        return;
      }

      std::ostringstream oss;
      oss << "\n"
          << profiler_.format_performance_summary(8);

      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      // 統計を出力した後、5分以上経ったデータをクリア
      static auto last_clear = time_utils::get_current_time_point();
      auto now = time_utils::get_current_time_point();
      if (time_utils::get_elapsed_time(last_clear, now) > 300.0)
      { // 5分
        profiler_.reset_stats();
        last_clear = now;
        RCLCPP_INFO(this->get_logger(), "Performance statistics reset after 5 minutes");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Performance stats error: %s", e.what());
    }
  }

} // namespace tvvf_vo_c