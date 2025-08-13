#include "tvvf_vo_c/ros/tvvf_vo_node.hpp"
#include <chrono>

namespace tvvf_vo_c
{

  TVVFVONode::TVVFVONode() : Node("tvvf_vo_c_node")
  {
    // パラメータ設定
    setup_parameters();

    // 設定初期化
    config_ = create_config_from_parameters();
    
    // グローバルフィールドジェネレータ初期化
    global_field_generator_ = std::make_unique<GlobalFieldGenerator>();
    
    // マップ端斥力パラメータを設定
    global_field_generator_->setMapRepulsionGain(
        this->get_parameter("map_repulsion_gain").as_double());
    global_field_generator_->setMapRepulsionRange(
        this->get_parameter("map_repulsion_range").as_double());

    // TF2関連初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // パブリッシャー初期化
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_markers", 10);
    vector_field_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tvvf_vo_vector_field", 10);

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

    // タイマー初期化（制御ループ：20Hz）
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&TVVFVONode::control_loop, this));

  }

  void TVVFVONode::setup_parameters()
  {

    // ロボット仕様パラメータ
    this->declare_parameter("max_linear_velocity", 2.0);
    this->declare_parameter("max_angular_velocity", 2.0);
    this->declare_parameter("robot_radius", 0.25);

    // フレーム名
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("global_frame", "map");

    // 制御関連
    this->declare_parameter("goal_tolerance", 0.1);
    
    // マップ端斥力パラメータ
    this->declare_parameter("map_repulsion_gain", 0.5);
    this->declare_parameter("map_repulsion_range", 1.0);
    
  }

  TVVFVOConfig TVVFVONode::create_config_from_parameters()
  {
    TVVFVOConfig config;

    config.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();

    return config;
  }

} // namespace tvvf_vo_c