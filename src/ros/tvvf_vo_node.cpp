#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <chrono>

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

    // 流体ベクトル場専用パラメータ
    this->declare_parameter("fluid_strength_factor", 1.0);
    this->declare_parameter("repulsive_weight", 0.4);
    this->declare_parameter("fluid_weight", 0.6);
    this->declare_parameter("path_direction_weight", 1.0);

    // 指数的斥力パラメータ
    this->declare_parameter("enable_exponential_repulsion", false);
    this->declare_parameter("exponential_base", 2.0);
    this->declare_parameter("exponential_scale_factor", 1.5);
    this->declare_parameter("max_exponential_distance", 1.5);
    this->declare_parameter("exponential_smoothing_threshold", 0.1);

    // 数値安定性
    this->declare_parameter("min_distance", 0.000001);

    // 性能関連
    this->declare_parameter("max_computation_time", 0.05);
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
    
    // 流体ベクトル場専用パラメータ
    config.fluid_strength_factor = this->get_parameter("fluid_strength_factor").as_double();
    config.repulsive_weight = this->get_parameter("repulsive_weight").as_double();
    config.fluid_weight = this->get_parameter("fluid_weight").as_double();
    config.path_direction_weight = this->get_parameter("path_direction_weight").as_double();
    
    // 指数的斥力パラメータ
    config.enable_exponential_repulsion = this->get_parameter("enable_exponential_repulsion").as_bool();
    config.exponential_base = this->get_parameter("exponential_base").as_double();
    config.exponential_scale_factor = this->get_parameter("exponential_scale_factor").as_double();
    config.max_exponential_distance = this->get_parameter("max_exponential_distance").as_double();
    config.exponential_smoothing_threshold = this->get_parameter("exponential_smoothing_threshold").as_double();
    
    // 数値安定性
    config.min_distance = this->get_parameter("min_distance").as_double();
    
    // 性能関連
    config.max_computation_time = this->get_parameter("max_computation_time").as_double();

    return config;
  }

} // namespace tvvf_vo_c