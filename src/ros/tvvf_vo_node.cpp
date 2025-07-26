#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include "tvvf_vo_c/ros/visualizer.hpp"
#include "tvvf_vo_c/utils/time_utils.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c {

TVVFVONode::TVVFVONode() : Node("tvvf_vo_c_node") {
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
    path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("planned_path", 10);

    // サブスクライバー初期化
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&TVVFVONode::map_callback, this, std::placeholders::_1));

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", 10, std::bind(&TVVFVONode::clicked_point_callback, this, std::placeholders::_1));

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TVVFVONode::laser_callback, this, std::placeholders::_1));

    // タイマー初期化（制御ループ：20Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TVVFVONode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "TVVF-VO C++ Node initialized");
}

void TVVFVONode::setup_parameters() {
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
    this->declare_parameter("laser_frame", "lidar_link");

    // 制御関連
    this->declare_parameter("goal_tolerance", 0.1);
    this->declare_parameter("max_computation_time", 0.05);

    // 可視化
    this->declare_parameter("enable_visualization", true);
}

TVVFVOConfig TVVFVONode::create_config_from_parameters() {
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
    config.max_computation_time = this->get_parameter("max_computation_time").as_double();

    return config;
}

std::optional<RobotState> TVVFVONode::get_robot_pose_from_tf() {
    try {
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
        Velocity velocity(0.0, 0.0);  // TODO: 速度推定の実装

        // ロボット状態作成
        RobotState robot_state(
            position, velocity, yaw,
            this->get_parameter("max_linear_velocity").as_double(),
            this->get_parameter("max_acceleration").as_double(),
            this->get_parameter("robot_radius").as_double()
        );

        return robot_state;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
        return std::nullopt;
    }
}

void TVVFVONode::clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    try {
        std::string global_frame = this->get_parameter("global_frame").as_string();

        Position goal_position(msg->point.x, msg->point.y);
        goal_ = Goal(goal_position, this->get_parameter("goal_tolerance").as_double());

        // 既存の経路をクリア
        planned_path_.reset();

        RCLCPP_INFO(this->get_logger(), "Goal set: (%.2f, %.2f)", 
                    goal_position.x, goal_position.y);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Clicked point callback error: %s", e.what());
    }
}

void TVVFVONode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    try {
        occupancy_grid_ = *msg;

        // A*経路計画器を初期化
        Position origin(msg->info.origin.position.x, msg->info.origin.position.y);
        double wall_clearance = this->get_parameter("wall_clearance_distance").as_double();
        
        path_planner_ = std::make_unique<AStarPathPlanner>(*msg, msg->info.resolution, origin, wall_clearance);

        RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                    msg->info.width, msg->info.height, msg->info.resolution);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Map callback error: %s", e.what());
    }
}

void TVVFVONode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    try {
        obstacles_ = detect_obstacles_from_laser(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Laser callback error: %s", e.what());
    }
}

void TVVFVONode::plan_path_to_goal() {
    try {
        if (!path_planner_ || !robot_state_.has_value() || !goal_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "Path planning: missing requirements");
            return;
        }

        // A*アルゴリズムで経路計画
        double start_time = time_utils::get_current_time();
        auto planned_path = path_planner_->plan_path(robot_state_->position, goal_->position);
        double planning_time = (time_utils::get_current_time() - start_time) * 1000;  // ms

        if (planned_path.has_value()) {
            planned_path_ = planned_path.value();
            RCLCPP_INFO(this->get_logger(), "A* Path Planning - Points: %zu, Time: %.1f ms, Freq: %.1f Hz",
                        planned_path_->size(), planning_time, 1000.0 / planning_time);
            publish_path_visualization();
        } else {
            RCLCPP_WARN(this->get_logger(), "A* Path planning failed after %.1f ms", planning_time);
            planned_path_.reset();
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Path planning error: %s", e.what());
        planned_path_.reset();
    }
}

std::vector<DynamicObstacle> TVVFVONode::detect_obstacles_from_laser(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
    
    std::vector<DynamicObstacle> obstacles;

    // 個別点による障害物検出
    double min_distance = 0.5;  // 最小検出距離
    double max_distance = config_.influence_radius;
    double default_radius = 0.1;  // 個別点の標準半径

    std::string laser_frame = laser_msg->header.frame_id;
    std::string global_frame = this->get_parameter("global_frame").as_string();

    // レーザーフレーム名のデバッグ出力
    static int frame_debug_counter = 0;
    if (++frame_debug_counter % 50 == 0) {
        RCLCPP_INFO(this->get_logger(), "Laser frame_id: '%s', Global frame: '%s'", 
                    laser_frame.c_str(), global_frame.c_str());
    }

    // フレーム名が空の場合はデフォルト値を使用
    if (laser_frame.empty()) {
        laser_frame = this->get_parameter("laser_frame").as_string();
        RCLCPP_WARN(this->get_logger(), "Empty laser frame_id detected, using parameter: '%s'", 
                    laser_frame.c_str());
    }

    try {
        // TF変換の準備
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_->lookupTransform(
            global_frame, laser_frame, laser_msg->header.stamp, tf2::durationFromSec(0.1));

        int valid_count = 0;
        for (size_t i = 0; i < laser_msg->ranges.size(); ++i) {
            double distance = laser_msg->ranges[i];
            
            if (distance > min_distance && distance < max_distance && 
                std::isfinite(distance)) {
                
                double angle = laser_msg->angle_min + i * laser_msg->angle_increment;

                // レーザーフレームでの障害物位置
                geometry_msgs::msg::PointStamped laser_point;
                laser_point.header.frame_id = laser_frame;
                laser_point.header.stamp = laser_msg->header.stamp;
                laser_point.point.x = distance * std::cos(angle);
                laser_point.point.y = distance * std::sin(angle);
                laser_point.point.z = 0.0;

                // グローバルフレームに変換
                geometry_msgs::msg::PointStamped global_point;
                tf2::doTransform(laser_point, global_point, transform_stamped);

                // 個別点を直接障害物として追加
                obstacles.emplace_back(
                    valid_count,
                    Position(global_point.point.x, global_point.point.y),
                    Velocity(0.0, 0.0),  // 静的障害物として扱う
                    default_radius
                );
                valid_count++;
            }
        }

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed for laser: %s", ex.what());
    }

    return obstacles;
}

void TVVFVONode::control_loop() {
    double loop_start_time = time_utils::get_current_time();

    try {
        // ロボット状態の更新
        double tf_start_time = time_utils::get_current_time();
        robot_state_ = get_robot_pose_from_tf();
        double tf_time = (time_utils::get_current_time() - tf_start_time) * 1000;  // ms

        // 状態チェック
        if (!robot_state_.has_value() || !goal_.has_value()) {
            return;
        }

        // A*経路計画チェック
        if (!planned_path_.has_value() && path_planner_) {
            RCLCPP_INFO(this->get_logger(), "Planning A* path...");
            plan_path_to_goal();
        }

        // 目標到達チェック
        double distance_to_goal = robot_state_->position.distance_to(goal_->position);
        if (distance_to_goal < goal_->tolerance) {
            publish_stop_command();
            planned_path_.reset();
            goal_.reset();
            publish_empty_visualization();
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        // TVVF-VO制御更新
        double control_start_time = time_utils::get_current_time();
        auto control_output = controller_->update(
            robot_state_.value(), obstacles_, goal_.value(), planned_path_);
        double control_time = (time_utils::get_current_time() - control_start_time) * 1000;  // ms

        // 制御コマンド発行
        publish_control_command(control_output);

        // 統計情報と可視化
        auto stats = controller_->get_stats();
        print_debug_info(stats, distance_to_goal);

        double viz_start_time = time_utils::get_current_time();
        if (this->get_parameter("enable_visualization").as_bool()) {
            publish_visualization();
        }
        double viz_time = (time_utils::get_current_time() - viz_start_time) * 1000;  // ms
        
        // 処理時間の合計
        double total_time = (time_utils::get_current_time() - loop_start_time) * 1000;  // ms
        
        // 処理速度情報を出力（5回に1回）
        static int counter = 0;
        if (++counter % 5 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Control Loop - Total: %.2f ms, TF: %.2f ms, Control: %.2f ms, Viz: %.2f ms, Freq: %.1f Hz",
                total_time, tf_time, control_time, viz_time, 1000.0 / total_time);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", e.what());
        publish_stop_command();
    }
}

void TVVFVONode::publish_control_command(const ControlOutput& control_output) {
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
    double desired_vx, double desired_vy, double current_orientation) {
    
    // 目標速度ベクトルの大きさと方向
    double target_speed = std::sqrt(desired_vx * desired_vx + desired_vy * desired_vy);
    double target_angle = std::atan2(desired_vy, desired_vx);

    // 現在の姿勢との角度差
    double angle_diff = math_utils::normalize_angle(target_angle - current_orientation);
    double orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();

    // 角度差が大きい場合は回転を優先
    double linear_velocity, angular_velocity;
    if (std::abs(angle_diff) > orientation_tolerance) {
        // 回転優先モード
        linear_velocity = target_speed * std::cos(angle_diff) * 0.3;  // 減速
        angular_velocity = 2.0 * angle_diff;  // 角度差に比例した角速度
    } else {
        // 前進優先モード
        linear_velocity = target_speed * std::cos(angle_diff);
        angular_velocity = 1.0 * angle_diff;  // 小さな補正
    }

    return {linear_velocity, angular_velocity};
}

void TVVFVONode::publish_stop_command() {
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd_msg);
}

void TVVFVONode::publish_visualization() {
    try {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        if (goal_.has_value()) {
            auto goal_marker = create_goal_marker(0);
            marker_array.markers.push_back(goal_marker);
        }

        marker_pub_->publish(marker_array);

        // ベクトル場の可視化を追加
        publish_vector_field_visualization();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Visualization error: %s", e.what());
    }
}

void TVVFVONode::publish_path_visualization() {
    try {
        if (!planned_path_.has_value() || planned_path_->empty()) {
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
        for (const auto& path_point : planned_path_->points) {
            geometry_msgs::msg::Point point;
            point.x = path_point.position.x;
            point.y = path_point.position.y;
            point.z = 0.05;  // 地面から少し浮かせる
            line_marker.points.push_back(point);
        }

        // 線の設定
        line_marker.scale.x = 0.05;  // 線の太さ
        line_marker.color.r = 1.0;   // 赤色
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;   // 透明度

        marker_array.markers.push_back(line_marker);
        path_pub_->publish(marker_array);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Path visualization error: %s", e.what());
    }
}

void TVVFVONode::publish_empty_visualization() {
    try {
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
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Empty visualization error: %s", e.what());
    }
}

void TVVFVONode::publish_vector_field_visualization() {
    try {
        double vf_start_time = time_utils::get_current_time();
        
        if (!robot_state_.has_value() || !goal_.has_value()) {
            return;
        }

        // ベクトル場計算用のグリッドを作成
        std::vector<Position> grid_positions;
        std::vector<Force> grid_forces;
        
        double grid_spacing = 0.5;  // グリッド間隔 [m]
        double grid_range = 3.0;    // ロボット周囲の範囲 [m]
        
        Position robot_pos = robot_state_->position;
        
        // ロボット周囲にグリッドを配置
        for (double x = robot_pos.x - grid_range; x <= robot_pos.x + grid_range; x += grid_spacing) {
            for (double y = robot_pos.y - grid_range; y <= robot_pos.y + grid_range; y += grid_spacing) {
                Position grid_pos(x, y);
                
                // TVVF力を計算
                Force tvvf_force(0.0, 0.0);
                
                // 簡略化された引力場計算
                Position goal_pos = goal_->position;
                Position relative_pos = Position(
                    goal_pos.x - grid_pos.x,
                    goal_pos.y - grid_pos.y
                );
                
                double distance = relative_pos.distance_to(Position(0, 0));
                if (distance > 0.1) {
                    // 正規化された引力方向
                    double attraction_strength = std::min(2.0, 1.0 / distance);
                    tvvf_force = Force(
                        (relative_pos.x / distance) * attraction_strength,
                        (relative_pos.y / distance) * attraction_strength
                    );
                }
                
                // 力が十分大きい場合のみ追加
                if (tvvf_force.magnitude() > 0.1) {
                    grid_positions.push_back(grid_pos);
                    grid_forces.push_back(tvvf_force);
                }
            }
        }
        
        // ベクトル場マーカーを作成・発行
        if (!grid_positions.empty()) {
            std::string global_frame = this->get_parameter("global_frame").as_string();
            TVVFVOVisualizer visualizer(*this, global_frame);
            
            auto vector_field_markers = visualizer.create_vector_field_markers(
                grid_positions, grid_forces, 1000);
            
            vector_field_pub_->publish(vector_field_markers);
            
            double vf_time = (time_utils::get_current_time() - vf_start_time) * 1000;  // ms
            RCLCPP_INFO(this->get_logger(), "Vector Field - Markers: %zu, Time: %.2f ms", 
                        vector_field_markers.markers.size(), vf_time);
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Vector field visualization error: %s", e.what());
    }
}

visualization_msgs::msg::Marker TVVFVONode::create_goal_marker(int marker_id) {
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

    return marker;
}

void TVVFVONode::print_debug_info(const std::unordered_map<std::string, double>& stats, 
                                 double distance_to_goal) {
    double safety = stats.at("safety_margin");
    std::string safety_str = (safety > 100.0) ? "INF" : std::to_string(safety).substr(0, 4) + "m";
    
    RCLCPP_INFO(this->get_logger(),
        "TVVF-VO: computation=%.1fms, VO_cones=%d, safety=%s, goal_dist=%.2fm",
        stats.at("computation_time"), static_cast<int>(stats.at("num_vo_cones")),
        safety_str.c_str(), distance_to_goal);
}


} // namespace tvvf_vo_c