#ifndef TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_
#define TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include <vector>
#include <optional>
#include <array>

namespace tvvf_vo_c {

/**
 * @brief Time-Varying Vector Field生成器
 */
class TVVFGenerator {
public:
    /**
     * @brief コンストラクタ
     * @param config TVVF-VO設定
     */
    explicit TVVFGenerator(const TVVFVOConfig& config);

    /**
     * @brief TVVF計算関数（A*経路統合版）
     * @param position 現在位置
     * @param time 現在時刻
     * @param goal 目標
     * @param obstacles 動的障害物リスト
     * @param planned_path 計画された経路（オプション）
     * @return 計算されたベクトル場（2D）
     */
    std::array<double, 2> compute_vector(const Position& position, double time,
                                        const Goal& goal,
                                        const std::vector<DynamicObstacle>& obstacles,
                                        const std::optional<Path>& planned_path = std::nullopt);

private:
    TVVFVOConfig config_;

    /**
     * @brief 引力場計算
     * @param position 現在位置
     * @param goal 目標
     * @return 引力ベクトル
     */
    std::array<double, 2> compute_attractive_force(const Position& position, const Goal& goal);

    /**
     * @brief 時間依存斥力場計算
     * @param position 現在位置
     * @param time 現在時刻
     * @param obstacles 動的障害物リスト
     * @return 斥力ベクトル
     */
    std::array<double, 2> compute_repulsive_force(const Position& position, double time,
                                                 const std::vector<DynamicObstacle>& obstacles);

    /**
     * @brief 個別障害物の動的ポテンシャル勾配計算
     * @param position 現在位置
     * @param obstacle 障害物
     * @param time 現在時刻
     * @return ポテンシャル勾配ベクトル
     */
    std::array<double, 2> compute_dynamic_potential_gradient(const Position& position,
                                                            const DynamicObstacle& obstacle,
                                                            double time);

    /**
     * @brief 最小距離到達時間の計算
     * @param position 現在位置
     * @param obstacle 障害物
     * @param horizon 時間ホライズン
     * @return 最小距離到達時間
     */
    double compute_minimum_distance_time(const Position& position,
                                        const DynamicObstacle& obstacle,
                                        double horizon);

    /**
     * @brief 時間依存重み計算
     * @param min_distance_time 最小距離到達時間
     * @param current_distance 現在距離
     * @param obstacle 障害物
     * @return 時間依存重み
     */
    double compute_time_dependent_weight(double min_distance_time,
                                        double current_distance,
                                        const DynamicObstacle& obstacle);

    /**
     * @brief 時間依存補正項の計算
     * @param position 現在位置
     * @param time 現在時刻
     * @param obstacles 動的障害物リスト
     * @param goal 目標
     * @return 補正ベクトル
     */
    std::array<double, 2> compute_time_correction(const Position& position, double time,
                                                 const std::vector<DynamicObstacle>& obstacles,
                                                 const Goal& goal);

    /**
     * @brief A*経路追従力計算
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @return 経路追従ベクトル
     */
    std::array<double, 2> compute_path_following_force(const Position& position,
                                                      const Path& planned_path);

    /**
     * @brief 経路考慮型ゴール引力計算
     * @param position 現在位置
     * @param goal 目標
     * @param planned_path 計画された経路
     * @return ゴール引力ベクトル
     */
    std::array<double, 2> compute_adaptive_goal_force(const Position& position,
                                                     const Goal& goal,
                                                     const Path& planned_path);

    /**
     * @brief 現在位置に最も近い経路点のインデックスを見つける
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @return 最も近い経路点のインデックス
     */
    size_t find_closest_path_point(const Position& position, const Path& planned_path);

    /**
     * @brief 先読み距離に基づく目標点インデックスを見つける
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @param start_idx 開始インデックス
     * @return 先読み点のインデックス
     */
    size_t find_lookahead_point(const Position& position, const Path& planned_path, size_t start_idx);

    /**
     * @brief 経路からの横方向誤差ベクトル計算
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @param closest_idx 最も近い経路点のインデックス
     * @return 横方向誤差ベクトル
     */
    std::array<double, 2> compute_cross_track_error(const Position& position, 
                                                   const Path& planned_path, 
                                                   size_t closest_idx);

    /**
     * @brief 指定時間後の予測に基づく補正ベクトル計算
     * @param position 現在位置
     * @param obstacles 動的障害物リスト
     * @param prediction_time 予測時間
     * @param goal 目標
     * @return 補正ベクトル
     */
    std::array<double, 2> compute_prediction_correction(const Position& position,
                                                       const std::vector<DynamicObstacle>& obstacles,
                                                       double prediction_time,
                                                       const Goal& goal);

    /**
     * @brief ベクトルの大きさを制限
     * @param vector 入力ベクトル
     * @param max_magnitude 最大大きさ
     * @return 制限されたベクトル
     */
    std::array<double, 2> clip_magnitude(const std::array<double, 2>& vector, double max_magnitude);

    /**
     * @brief 安全な正規化（ゼロ除算回避）
     * @param vector 入力ベクトル
     * @param min_norm 最小ノルム
     * @return 正規化されたベクトル
     */
    std::array<double, 2> safe_normalize(const std::array<double, 2>& vector, double min_norm = 1e-8);

    /**
     * @brief 安全な正規化（デフォルト値指定可能）
     * @param vector 入力ベクトル
     * @param min_norm 最小ノルム
     * @param default_value デフォルト値
     * @return 正規化されたベクトル
     */
    std::array<double, 2> safe_normalize_with_default(const std::array<double, 2>& vector,
                                                     double min_norm = 1e-8,
                                                     const std::array<double, 2>& default_value = {1.0, 0.0});
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_