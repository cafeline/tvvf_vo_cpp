#ifndef TVVF_VO_C_ROS_VISUALIZER_HPP_
#define TVVF_VO_C_ROS_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "tvvf_vo_c/core/types.hpp"

#include <vector>
#include <string>

namespace tvvf_vo_c {

/**
 * @brief TVVF-VO用RViz可視化マネージャー
 */
class TVVFVOVisualizer {
public:
    /**
     * @brief コンストラクタ
     * @param node ROSノード参照
     * @param global_frame グローバル座標フレーム名
     */
    TVVFVOVisualizer(rclcpp::Node& node, const std::string& global_frame);

    /**
     * @brief ベクトル場可視化マーカーを作成
     * @param positions ベクトル場の位置リスト
     * @param forces ベクトル場の力ベクトルリスト
     * @param marker_id マーカーID
     * @return ベクトル場マーカー配列
     */
    visualization_msgs::msg::MarkerArray create_vector_field_markers(
        const std::vector<Position>& positions,
        const std::vector<Force>& forces,
        int marker_id = 0);

    /**
     * @brief 障害物可視化マーカーを作成
     * @param obstacles 障害物リスト
     * @param marker_id マーカーID
     * @return 障害物マーカー配列
     */
    visualization_msgs::msg::MarkerArray create_obstacle_markers(
        const std::vector<DynamicObstacle>& obstacles,
        int marker_id = 1000);

    /**
     * @brief VO円錐可視化マーカーを作成
     * @param robot_state ロボット状態
     * @param obstacles 障害物リスト
     * @param time_horizon VO時間ホライズン
     * @param marker_id マーカーID
     * @return VO円錐マーカー配列
     */
    visualization_msgs::msg::MarkerArray create_vo_cone_markers(
        const RobotState& robot_state,
        const std::vector<DynamicObstacle>& obstacles,
        double time_horizon,
        int marker_id = 2000);

    /**
     * @brief 経路可視化マーカーを作成
     * @param path 経路
     * @param marker_id マーカーID
     * @return 経路マーカー配列
     */
    visualization_msgs::msg::MarkerArray create_path_markers(
        const Path& path,
        int marker_id = 3000);

    /**
     * @brief 目標位置可視化マーカーを作成
     * @param goal 目標
     * @param marker_id マーカーID
     * @return 目標マーカー
     */
    visualization_msgs::msg::Marker create_goal_marker(
        const Goal& goal,
        int marker_id = 4000);

    /**
     * @brief ロボット軌跡可視化マーカーを作成
     * @param robot_positions ロボット位置履歴
     * @param marker_id マーカーID
     * @return 軌跡マーカー
     */
    visualization_msgs::msg::Marker create_trajectory_marker(
        const std::vector<Position>& robot_positions,
        int marker_id = 5000);

    /**
     * @brief マーカー削除配列を作成
     * @return 削除マーカー配列
     */
    visualization_msgs::msg::MarkerArray create_delete_all_markers();

private:
    rclcpp::Node& node_;
    std::string global_frame_;

    /**
     * @brief 標準色を作成
     * @param r 赤成分 (0.0-1.0)
     * @param g 緑成分 (0.0-1.0)
     * @param b 青成分 (0.0-1.0)
     * @param a アルファ成分 (0.0-1.0)
     * @return 色メッセージ
     */
    std_msgs::msg::ColorRGBA create_color(double r, double g, double b, double a = 1.0);

    /**
     * @brief 矢印マーカーを作成
     * @param start 開始点
     * @param end 終了点
     * @param color 色
     * @param marker_id マーカーID
     * @param scale スケール
     * @return 矢印マーカー
     */
    visualization_msgs::msg::Marker create_arrow_marker(
        const Position& start,
        const Position& end,
        const std_msgs::msg::ColorRGBA& color,
        int marker_id,
        double scale = 0.05);

    /**
     * @brief 円柱マーカーを作成
     * @param center 中心位置
     * @param radius 半径
     * @param height 高さ
     * @param color 色
     * @param marker_id マーカーID
     * @return 円柱マーカー
     */
    visualization_msgs::msg::Marker create_cylinder_marker(
        const Position& center,
        double radius,
        double height,
        const std_msgs::msg::ColorRGBA& color,
        int marker_id);

    /**
     * @brief 線分マーカーを作成
     * @param points 点リスト
     * @param color 色
     * @param marker_id マーカーID
     * @param line_width 線幅
     * @return 線分マーカー
     */
    visualization_msgs::msg::Marker create_line_marker(
        const std::vector<Position>& points,
        const std_msgs::msg::ColorRGBA& color,
        int marker_id,
        double line_width = 0.02);

    /**
     * @brief VO円錐の頂点を計算
     * @param robot_pos ロボット位置
     * @param obstacle_pos 障害物位置
     * @param obstacle_radius 障害物半径
     * @param time_horizon 時間ホライズン
     * @return 円錐の頂点リスト
     */
    std::vector<Position> compute_vo_cone_vertices(
        const Position& robot_pos,
        const Position& obstacle_pos,
        double obstacle_radius,
        double time_horizon);
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_ROS_VISUALIZER_HPP_