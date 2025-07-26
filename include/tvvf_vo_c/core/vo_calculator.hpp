#ifndef TVVF_VO_C_CORE_VO_CALCULATOR_HPP_
#define TVVF_VO_C_CORE_VO_CALCULATOR_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include <vector>
#include <optional>
#include <array>

namespace tvvf_vo_c {

/**
 * @brief Velocity Obstacle計算器
 */
class VelocityObstacleCalculator {
public:
    /**
     * @brief コンストラクタ
     * @param config TVVF-VO設定
     */
    explicit VelocityObstacleCalculator(const TVVFVOConfig& config);

    /**
     * @brief VO集合計算
     * @param robot_state ロボット状態
     * @param obstacles 障害物リスト
     * @param time_horizon 時間ホライズン
     * @return VO錐体のリスト
     */
    std::vector<VOCone> compute_vo_set(const RobotState& robot_state,
                                      const std::vector<DynamicObstacle>& obstacles,
                                      double time_horizon);

    /**
     * @brief 指定速度がVO錐体内にあるかチェック
     * @param velocity 速度ベクトル
     * @param vo_cone VO錐体
     * @return VO錐体内にある場合true
     */
    bool is_velocity_in_vo(const std::array<double, 2>& velocity, const VOCone& vo_cone);

    /**
     * @brief VO制約を満たす実行可能速度のサンプリング
     * @param robot_state ロボット状態
     * @param vo_cones VO錐体リスト
     * @param resolution 速度解像度（オプション）
     * @return 実行可能速度のリスト
     */
    std::vector<std::array<double, 2>> get_vo_free_velocities(const RobotState& robot_state,
                                                             const std::vector<VOCone>& vo_cones,
                                                             double resolution = 0.1);

private:
    TVVFVOConfig config_;

    /**
     * @brief 単一障害物に対するVO錐体計算
     * @param robot_state ロボット状態
     * @param obstacle 障害物
     * @param time_horizon 時間ホライズン
     * @return VO錐体（失敗時はstd::nullopt）
     */
    std::optional<VOCone> compute_single_vo(const RobotState& robot_state,
                                           const DynamicObstacle& obstacle,
                                           double time_horizon);

    /**
     * @brief ベクトルの大きさ計算
     * @param vector 2Dベクトル
     * @return 大きさ
     */
    double vector_magnitude(const std::array<double, 2>& vector);

    /**
     * @brief ベクトルの正規化
     * @param vector 2Dベクトル
     * @return 正規化されたベクトル
     */
    std::array<double, 2> normalize_vector(const std::array<double, 2>& vector);

    /**
     * @brief 2Dベクトルの外積計算
     * @param a ベクトルa
     * @param b ベクトルb
     * @return 外積のz成分
     */
    double cross_product_2d(const std::array<double, 2>& a, const std::array<double, 2>& b);
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_VO_CALCULATOR_HPP_