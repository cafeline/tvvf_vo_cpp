#ifndef TVVF_VO_C_CORE_VELOCITY_SELECTOR_HPP_
#define TVVF_VO_C_CORE_VELOCITY_SELECTOR_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include "tvvf_vo_c/core/vo_calculator.hpp"
#include <vector>
#include <array>

namespace tvvf_vo_c {

/**
 * @brief 実行可能速度選択器
 */
class FeasibleVelocitySelector {
public:
    /**
     * @brief コンストラクタ
     * @param config TVVF-VO設定
     */
    explicit FeasibleVelocitySelector(const TVVFVOConfig& config);

    /**
     * @brief TVVFベクトルとVO制約から実行可能速度選択
     * @param tvvf_vector TVVFベクトル
     * @param vo_cones VO錐体リスト
     * @param robot_state ロボット状態
     * @return 選択された実行可能速度
     */
    Velocity select_feasible_velocity(const std::array<double, 2>& tvvf_vector,
                                     const std::vector<VOCone>&,
                                     const RobotState& robot_state);

private:
    TVVFVOConfig config_;
    VelocityObstacleCalculator vo_calc_;

    /**
     * @brief 候補速度の生成
     * @param tvvf_vector TVVFベクトル
     * @param vo_cones VO錐体リスト
     * @param robot_state ロボット状態
     * @return 候補速度のリスト
     */
    std::vector<std::array<double, 2>> generate_candidate_velocities(
        const std::array<double, 2>& tvvf_vector,
        const std::vector<VOCone>& vo_cones,
        const RobotState& robot_state);

    /**
     * @brief 速度がVO制約を満たすかチェック
     * @param velocity 速度
     * @param vo_cones VO錐体リスト
     * @return 実行可能な場合true
     */
    bool is_velocity_feasible(const std::array<double, 2>& velocity,
                             const std::vector<VOCone>& vo_cones);

    /**
     * @brief 最適速度の選択
     * @param candidates 候補速度リスト
     * @param tvvf_vector TVVFベクトル
     * @param vo_cones VO錐体リスト
     * @param robot_state ロボット状態
     * @return 最適速度
     */
    std::array<double, 2> select_optimal_velocity(
        const std::vector<std::array<double, 2>>& candidates,
        const std::array<double, 2>& tvvf_vector,
        const std::vector<VOCone>& vo_cones,
        const RobotState& robot_state);

    /**
     * @brief 速度の評価スコア計算
     * @param velocity 速度
     * @param tvvf_direction TVVF方向ベクトル
     * @param tvvf_magnitude TVVF大きさ
     * @param vo_cones VO錐体リスト
     * @param robot_state ロボット状態
     * @return 評価スコア
     */
    double compute_velocity_score(const std::array<double, 2>& velocity,
                                 const std::array<double, 2>& tvvf_direction,
                                 double tvvf_magnitude,
                                 const std::vector<VOCone>& vo_cones,
                                 const RobotState& robot_state);

    /**
     * @brief VO安全マージンスコア計算
     * @param velocity 速度
     * @param vo_cones VO錐体リスト
     * @return 安全マージンスコア
     */
    double compute_safety_score(const std::array<double, 2>& velocity,
                               const std::vector<VOCone>& vo_cones);

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

#endif // TVVF_VO_C_CORE_VELOCITY_SELECTOR_HPP_