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
#include "tvvf_vo_c/core/controller.hpp"
#include "tvvf_vo_c/core/global_field_generator.hpp"
#include <memory>
#include <vector>
#include <optional>
#include <string>

namespace tvvf_vo_c {

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
    std::unique_ptr<TVVFVOController> controller_;
    std::unique_ptr<GlobalFieldGenerator> global_field_generator_;
    TVVFVOConfig config_;

    // 状態変数
    std::optional<RobotState> robot_state_;
    std::optional<Goal> goal_;
    std::vector<DynamicObstacle> dynamic_obstacles_;
    std::vector<DynamicObstacle> static_obstacles_;
    std::optional<nav_msgs::msg::OccupancyGrid> current_map_;
    
    // TF2関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // パブリッシャー
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
     * @brief ベクトル場可視化マーカーを配信
     */
    void publish_vector_field_visualization();

    /**
     * @brief 空の可視化マーカーを送信（クリア用）
     */
    void publish_empty_visualization();

    /**
     * @brief グローバルベクトル場の可視化
     * @param field ベクトル場
     */
    void publish_global_field_visualization(const VectorField& field);

    /**
     * @brief パフォーマンス統計の表示
     */
    void log_performance_stats();

};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_ROS_TVVF_VO_NODE_HPP_