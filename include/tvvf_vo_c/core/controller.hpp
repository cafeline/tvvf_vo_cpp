#ifndef TVVF_VO_C_CORE_CONTROLLER_HPP_
#define TVVF_VO_C_CORE_CONTROLLER_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include "tvvf_vo_c/utils/time_utils.hpp"
#include "tvvf_vo_c/utils/math_utils.hpp"
#include <optional>
#include <unordered_map>
#include <string>

namespace tvvf_vo_c {

/**
 * @brief TVVF + VO統合制御器
 */
class TVVFVOController {
public:
    /**
     * @brief コンストラクタ
     * @param config TVVF-VO設定
     */
    explicit TVVFVOController(const TVVFVOConfig& config);

    /**
     * @brief 制御更新関数
     * @param robot_state ロボット状態
     * @param obstacles 動的障害物リスト
     * @param goal 目標
     * @param planned_path 計画された経路（オプション）
     * @return 制御出力
     */
    ControlOutput update(const RobotState& robot_state,
                        const std::vector<DynamicObstacle>& obstacles,
                        const Goal& goal,
                        const std::optional<Path>& planned_path = std::nullopt);

    /**
     * @brief 統計情報取得
     * @return 統計情報マップ
     */
    std::unordered_map<std::string, double> get_stats() const;


private:
    TVVFVOConfig config_;

    // 統計情報とプロファイラー
    mutable std::unordered_map<std::string, double> stats_;
    mutable time_utils::ModuleProfiler internal_profiler_;

    /**
     * @brief 現在の安全マージン計算
     * @param robot_state ロボット状態
     * @param obstacles 動的障害物リスト
     * @return 安全マージン
     */

    /**
     * @brief 角度差の正規化（-π to π）
     * @param angle 角度（ラジアン）
     * @return 正規化された角度
     */
    double normalize_angle(double angle);

    /**
     * @brief 経過時間計測用の高精度タイマー
     * @return 現在時刻（秒）
     */
    double get_current_time();
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_CONTROLLER_HPP_