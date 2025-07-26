#ifndef TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_
#define TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_

#include "tvvf_vo_c/core/types.hpp"
#include <vector>
#include <optional>
#include <array>

namespace tvvf_vo_c {

/**
 * @brief 経路投影情報
 */
struct PathProjection {
    bool valid;                    // 投影が有効かどうか
    size_t segment_start_idx;      // 投影されたセグメントの開始インデックス
    double parameter_t;            // セグメント内での位置パラメータ [0,1]
    Position projection_point;     // 投影点の位置
    double cross_track_distance;   // 横方向距離（符号付き）
    double along_track_distance;   // 経路に沿った距離
};

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
                                        const std::optional<Path>& planned_path = std::nullopt) const;

private:
    TVVFVOConfig config_;

    /**
     * @brief 引力場計算
     * @param position 現在位置
     * @param goal 目標
     * @return 引力ベクトル
     */
    std::array<double, 2> compute_attractive_force(const Position& position, const Goal& goal) const;

    /**
     * @brief 時間依存斥力場計算
     * @param position 現在位置
     * @param time 現在時刻
     * @param obstacles 動的障害物リスト
     * @return 斥力ベクトル
     */
    std::array<double, 2> compute_repulsive_force(const Position& position, double time,
                                                 const std::vector<DynamicObstacle>& obstacles) const;

    /**
     * @brief 個別障害物の動的ポテンシャル勾配計算
     * @param position 現在位置
     * @param obstacle 障害物
     * @param time 現在時刻
     * @return ポテンシャル勾配ベクトル
     */
    std::array<double, 2> compute_dynamic_potential_gradient(const Position& position,
                                                            const DynamicObstacle& obstacle,
                                                            double time) const;

    /**
     * @brief 最小距離到達時間の計算
     * @param position 現在位置
     * @param obstacle 障害物
     * @param horizon 時間ホライズン
     * @return 最小距離到達時間
     */
    double compute_minimum_distance_time(const Position& position,
                                        const DynamicObstacle& obstacle,
                                        double horizon) const;

    /**
     * @brief 時間依存重み計算
     * @param min_distance_time 最小距離到達時間
     * @param current_distance 現在距離
     * @param obstacle 障害物
     * @return 時間依存重み
     */
    double compute_time_dependent_weight(double min_distance_time,
                                        double current_distance,
                                        const DynamicObstacle& obstacle) const;

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
                                                 const Goal& goal) const;

    /**
     * @brief A*経路追従力計算
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @return 経路追従ベクトル
     */
    std::array<double, 2> compute_path_following_force(const Position& position,
                                                      const Path& planned_path) const;

    /**
     * @brief 経路考慮型ゴール引力計算
     * @param position 現在位置
     * @param goal 目標
     * @param planned_path 計画された経路
     * @return ゴール引力ベクトル
     */
    std::array<double, 2> compute_adaptive_goal_force(const Position& position,
                                                     const Goal& goal,
                                                     const Path& planned_path) const;

    /**
     * @brief 現在位置に最も近い経路点のインデックスを見つける
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @return 最も近い経路点のインデックス
     */
    size_t find_closest_path_point(const Position& position, const Path& planned_path) const;

    /**
     * @brief 先読み距離に基づく目標点インデックスを見つける
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @param start_idx 開始インデックス
     * @return 先読み点のインデックス
     */
    size_t find_lookahead_point(const Position& position, const Path& planned_path, size_t start_idx) const;

    /**
     * @brief 経路からの横方向誤差ベクトル計算
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @param closest_idx 最も近い経路点のインデックス
     * @return 横方向誤差ベクトル
     */
    std::array<double, 2> compute_cross_track_error(const Position& position, 
                                                   const Path& planned_path, 
                                                   size_t closest_idx) const;

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
                                                       const Goal& goal) const;

    /**
     * @brief ベクトルの大きさを制限
     * @param vector 入力ベクトル
     * @param max_magnitude 最大大きさ
     * @return 制限されたベクトル
     */
    std::array<double, 2> clip_magnitude(const std::array<double, 2>& vector, double max_magnitude) const;

    /**
     * @brief 安全な正規化（ゼロ除算回避）
     * @param vector 入力ベクトル
     * @param min_norm 最小ノルム
     * @return 正規化されたベクトル
     */
    std::array<double, 2> safe_normalize(const std::array<double, 2>& vector, double min_norm = 1e-8) const;

    /**
     * @brief 安全な正規化（デフォルト値指定可能）
     * @param vector 入力ベクトル
     * @param min_norm 最小ノルム
     * @param default_value デフォルト値
     * @return 正規化されたベクトル
     */
    std::array<double, 2> safe_normalize_with_default(const std::array<double, 2>& vector,
                                                     double min_norm = 1e-8,
                                                     const std::array<double, 2>& default_value = {1.0, 0.0}) const;

    /**
     * @brief スタック回避のための適応的力合成
     * @param position 現在位置
     * @param path_force A*経路追従力
     * @param goal_force ゴール引力
     * @param repulsive_force 斥力
     * @param time_correction 時間補正
     * @param obstacles 障害物リスト
     * @return 統合された力ベクトル
     */
    std::array<double, 2> adaptive_force_composition(const Position& position,
                                                    const std::array<double, 2>& path_force,
                                                    const std::array<double, 2>& goal_force,
                                                    const std::array<double, 2>& repulsive_force,
                                                    const std::array<double, 2>& time_correction,
                                                    const std::vector<DynamicObstacle>& obstacles) const;

    /**
     * @brief 前進方向を保証する経路投影計算
     * @param position 現在位置
     * @param planned_path 計画された経路
     * @return 経路投影情報
     */
    PathProjection find_forward_path_projection(const Position& position, const Path& planned_path) const;

    /**
     * @brief 投影点から先読み距離分前進した目標点を計算
     * @param projection 経路投影情報
     * @param planned_path 計画された経路
     * @return 目標位置
     */
    Position compute_lookahead_target(const PathProjection& projection, const Path& planned_path) const;

    /**
     * @brief 軽微な横方向復帰力計算
     * @param projection 経路投影情報
     * @return 横方向復帰力ベクトル
     */
    std::array<double, 2> compute_gentle_lateral_correction(const PathProjection& projection) const;
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_TVVF_GENERATOR_HPP_