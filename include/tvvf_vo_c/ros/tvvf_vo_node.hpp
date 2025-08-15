#ifndef TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_
#define TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tvvf_vo_c/core/types.hpp"
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include "tvvf_vo_c/core/repulsive_force.hpp"
#include <memory>
#include <vector>
#include <optional>
#include <string>

namespace tvvf_vo_c {

// 可視化パラメータの定数
namespace visualization_constants {
    constexpr int DEFAULT_GRID_STRIDE = 5;           // グリッドサンプリング間隔
    constexpr double DEFAULT_ARROW_LENGTH = 0.3;     // 矢印の長さ[m]
    constexpr double ARROW_SHAFT_WIDTH = 0.05;       // 矢印シャフトの太さ
    constexpr double ARROW_HEAD_WIDTH = 0.1;         // 矢印ヘッドの太さ
    constexpr double ARROW_ALPHA = 0.7;              // 矢印の透明度
    constexpr double MIN_VECTOR_MAGNITUDE = 0.01;    // 表示する最小ベクトル大きさ
    constexpr double MARKER_LIFETIME_SEC = 0.1;      // マーカーの寿命[秒]
}

/**
 * @brief TVVF-VO ROS2ナビゲーションノード
 */
class TVVFVONode : public rclcpp::Node {
public:
    /**
     * @brief コンストラクタ
     */
    TVVFVONode();

private:
    // コア機能
    std::unique_ptr<GlobalFieldGenerator> global_field_generator_;
    std::unique_ptr<RepulsiveForceCalculator> repulsive_force_calculator_;
    TVVFVOConfig config_;
    RepulsiveForceConfig repulsive_config_;

    // 状態変数
    std::optional<RobotState> robot_state_;
    std::optional<Goal> goal_;
    std::vector<DynamicObstacle> dynamic_obstacles_;
    std::optional<nav_msgs::msg::OccupancyGrid> current_map_;
    std::optional<visualization_msgs::msg::MarkerArray> static_obstacles_;
    
    // TF2関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // パブリッシャー
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_field_pub_;

    // サブスクライバー
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacles_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr static_obstacles_sub_;

    // タイマー
    rclcpp::TimerBase::SharedPtr control_timer_;


    /**
     * @brief パラメータ設定
     */
    void setup_parameters();

    /**
     * @brief ROS2パラメータからTVVFVOConfigを作成
     * @return TVVF-VO設定
     */
    TVVFVOConfig create_config_from_parameters();

    /**
     * @brief TFからロボット位置を取得
     * @return ロボット状態（失敗時はstd::nullopt）
     */
    std::optional<RobotState> get_robot_pose_from_tf();

    /**
     * @brief クリックされたポイントをゴールとして設定
     * @param msg PointStampedメッセージ
     */
    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    /**
     * @brief 地図データコールバック
     * @param msg OccupancyGridメッセージ
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 障害物データコールバック（動的・静的共通）
     * @param msg MarkerArrayメッセージ
     * @param is_dynamic 動的障害物かどうか
     */
    void obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg, bool is_dynamic);


    /**
     * @brief メイン制御ループ
     */
    void control_loop();

    /**
     * @brief 制御コマンドの配信（差動二輪用）
     * @param control_output 制御出力
     */
    void publish_control_command(const ControlOutput& control_output);

    /**
     * @brief ベクトル場からの速度指令を差動二輪用に変換
     * @param desired_vx 目標x方向速度
     * @param desired_vy 目標y方向速度
     * @param current_orientation 現在の姿勢角
     * @return 変換された（線形速度、角速度）
     */
    std::pair<double, double> convert_to_differential_drive(
        double desired_vx, double desired_vy, double current_orientation);

    /**
     * @brief 停止コマンドの配信
     */
    void publish_stop_command();

    /**
     * @brief 空の可視化マーカーを送信（クリア用）
     */
    void publish_empty_visualization();
    
    /**
     * @brief 合成ベクトル場の可視化（斥力込み）
     * @param field 元のベクトル場
     */
    void publish_combined_field_visualization(const VectorField& field);

private:
    // 制御ループヘルパー関数
    bool update_robot_state();
    bool has_valid_goal() const;
    bool is_goal_reached() const;
    void handle_goal_reached();
    ControlOutput compute_control_output();
    void apply_repulsive_force(std::array<double, 2>& velocity_vector);
    void scale_velocity_vector(std::array<double, 2>& velocity_vector);
    void update_visualization();
    
private:
    // 可視化ヘルパー関数
    std::array<double, 2> calculate_combined_vector(
        const std::array<double, 2>& original_vector, 
        const Position& world_pos) const;
    
    bool should_visualize_vector(const std::array<double, 2>& vector) const;
    
    visualization_msgs::msg::Marker create_arrow_marker(
        const Position& position,
        const std::array<double, 2>& vector,
        int marker_id,
        const std::string& frame_id) const;

};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_