#include "tvvf_vo_c/core/tvvf_generator.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace tvvf_vo_c {

TVVFGenerator::TVVFGenerator(const TVVFVOConfig& config) : config_(config) {}

std::array<double, 2> TVVFGenerator::compute_vector(const Position& position, double /*time*/,
                                                    const Goal& goal,
                                                    const std::vector<DynamicObstacle>& obstacles,
                                                    const std::optional<Path>& planned_path) const {
    // 1. A*経路の存在チェック
    if (!planned_path.has_value() || planned_path->empty()) {
        // A*経路がない場合は基本的なゴール向けベクトル場を生成（指数的斥力対応）
        auto goal_force = compute_attractive_force(position, goal);
        std::array<double, 2> repulsive_force = {0.0, 0.0};

        // 障害物回避力計算（指数的斥力対応）

        for (const auto& obstacle : obstacles) {
            // 指数的斥力を使用
            std::array<double, 2> single_repulsive = compute_exponential_repulsive_force(position, obstacle);
            repulsive_force[0] += single_repulsive[0];
            repulsive_force[1] += single_repulsive[1];
        }


        return {goal_force[0] + repulsive_force[0], goal_force[1] + repulsive_force[1]};
    }

    // A*経路追従ベクトル場を使用（必須）
    auto path_force = compute_path_following_force(position, planned_path.value());
    auto goal_force = compute_adaptive_goal_force(position, goal, planned_path.value());

    // 2. 経路の先読み方向を計算
    std::array<double, 2> path_direction = compute_path_lookahead_direction(position, planned_path.value());

    // 3. 経路方向統合型の障害物回避力計算（指数的斥力対応）
    std::array<double, 2> repulsive_force = {0.0, 0.0};


    for (const auto& obstacle : obstacles) {

        // 指数的斥力統合版を使用
        std::array<double, 2> integrated_vector = compute_exponential_integrated_avoidance_vector(position, obstacle, path_direction, planned_path.value());


        repulsive_force[0] += integrated_vector[0];
        repulsive_force[1] += integrated_vector[1];
    }


    // 4. スタック回避のための適応的合成（時間補正なし）
    std::array<double, 2> zero_time_correction = {0.0, 0.0};
    std::array<double, 2> total_force = adaptive_force_composition(
        position, path_force, goal_force, repulsive_force, zero_time_correction, obstacles);

    // 5. 数値安定性（max_force制限を削除）

    return total_force;
}

std::array<double, 2> TVVFGenerator::compute_attractive_force(const Position& position, const Goal& goal) const {
    std::array<double, 2> goal_vector = {goal.position.x - position.x, goal.position.y - position.y};
    double distance = std::sqrt(goal_vector[0] * goal_vector[0] + goal_vector[1] * goal_vector[1]);

    if (distance < config_.min_distance) {
        return {0.0, 0.0};
    }

    std::array<double, 2> attractive_force;
    if (distance > goal.tolerance * 2) {
        auto normalized_goal = safe_normalize(goal_vector);
        attractive_force = {config_.k_attraction * normalized_goal[0],
                           config_.k_attraction * normalized_goal[1]};
    } else {
        attractive_force = {config_.k_attraction * goal_vector[0] * 0.5,
                           config_.k_attraction * goal_vector[1] * 0.5};
    }

    return attractive_force;
}


std::array<double, 2> TVVFGenerator::compute_dynamic_potential_gradient(const Position& position,
                                                                       const DynamicObstacle& obstacle,
                                                                       double /* time */) const {
    std::array<double, 2> current_relative_pos = {position.x - obstacle.position.x,
                                                  position.y - obstacle.position.y};
    double current_distance = std::sqrt(current_relative_pos[0] * current_relative_pos[0] +
                                       current_relative_pos[1] * current_relative_pos[1]);

    // 斥力の影響範囲を設定
    double max_repulsive_distance = config_.influence_radius;
    if (current_distance > max_repulsive_distance) {
        return {0.0, 0.0};
    }

    // シンプルな斥力計算（距離別重み調整機能を削除）
    std::array<double, 2> force_direction;
    double force_magnitude;

    if (current_distance < config_.min_distance) {
        // 極近距離：強制的な反発
        force_direction = safe_normalize_with_default(current_relative_pos, 1e-8, {1.0, 0.0});
        force_magnitude = config_.k_repulsion * 100.0;
    } else {
        force_direction = safe_normalize(current_relative_pos);

        // 距離の逆二乗に比例する斥力
        force_magnitude = config_.k_repulsion / (current_distance * current_distance);
    }

    return {force_magnitude * force_direction[0],
            force_magnitude * force_direction[1]};
}

double TVVFGenerator::compute_minimum_distance_time(const Position& position,
                                                   const DynamicObstacle& obstacle,
                                                   double horizon) const {
    std::array<double, 2> rel_pos = {position.x - obstacle.position.x, position.y - obstacle.position.y};
    std::array<double, 2> rel_vel = {-obstacle.velocity.vx, -obstacle.velocity.vy};

    double rel_speed = std::sqrt(rel_vel[0] * rel_vel[0] + rel_vel[1] * rel_vel[1]);
    if (rel_speed < 0.1) {
        return 0.0;
    }

    double t_min = -(rel_pos[0] * rel_vel[0] + rel_pos[1] * rel_vel[1]) / (rel_speed * rel_speed);
    return std::max(0.0, std::min(t_min, horizon));
}

double TVVFGenerator::compute_time_dependent_weight(double min_distance_time,
                                                   double current_distance,
                                                   const DynamicObstacle& obstacle) const {
    double base_weight = 1.0;
    double time_urgency = std::exp(-min_distance_time / 1.0);
    double distance_urgency = std::exp(-current_distance / obstacle.radius);
    double total_weight = base_weight * (1.0 + time_urgency + distance_urgency);
    return std::min(total_weight, 10.0);
}


std::array<double, 2> TVVFGenerator::compute_path_following_force(const Position& position,
                                                                 const Path& planned_path) const {
    if (planned_path.empty()) {
        return {0.0, 0.0};
    }

    // 1. 前進方向を保証する投影ベースの経路追従
    auto path_projection = find_forward_path_projection(position, planned_path);
    if (!path_projection.valid) {
        // 経路終端に近づいた場合はゴール方向
        const Position& final_point = planned_path[planned_path.size() - 1].position;
        std::array<double, 2> direction = {final_point.x - position.x, final_point.y - position.y};
        return {config_.k_path_attraction * direction[0], config_.k_path_attraction * direction[1]};
    }

    // 2. 投影点から先読み距離分前進した目標点を計算
    Position target_position = compute_lookahead_target(path_projection, planned_path);

    // 3. 目標点への方向力（前進方向のみ）
    std::array<double, 2> direction_vector = {target_position.x - position.x,
                                             target_position.y - position.y};
    double distance = std::sqrt(direction_vector[0] * direction_vector[0] +
                               direction_vector[1] * direction_vector[1]);

    if (distance < config_.min_distance) {
        return {0.0, 0.0};
    }

    // 4. 正規化された前進力
    auto forward_direction = safe_normalize(direction_vector);
    double base_strength = config_.k_path_attraction * 2.0;  // 前進力を強化

    // 5. 経路からの横方向誤差による減衰
    double lateral_distance = path_projection.cross_track_distance;
    double decay_factor = std::exp(-std::abs(lateral_distance) / config_.lookahead_distance);

    // 6. 最終的な前進力（横方向誤差で減衰）
    double effective_strength = base_strength * decay_factor;
    std::array<double, 2> forward_force = {effective_strength * forward_direction[0],
                                          effective_strength * forward_direction[1]};

    // 7. 軽微な横方向復帰力（前進を妨げない程度）
    if (std::abs(lateral_distance) > 0.1) {
        std::array<double, 2> lateral_correction = compute_gentle_lateral_correction(path_projection);
        forward_force[0] += 0.3 * lateral_correction[0];
        forward_force[1] += 0.3 * lateral_correction[1];
    }

    return forward_force;
}

std::array<double, 2> TVVFGenerator::compute_adaptive_goal_force(const Position& position,
                                                                const Goal& goal,
                                                                const Path& planned_path) const {
    // 経路の最終点に近い場合のみゴール引力を適用
    if (planned_path.empty()) {
        return compute_attractive_force(position, goal);
    }

    const Position& final_path_point = planned_path[planned_path.size() - 1].position;
    double distance_to_final = position.distance_to(final_path_point);

    // 経路終端付近でのみゴール引力を強化
    if (distance_to_final < config_.lookahead_distance * 2) {
        auto goal_force = compute_attractive_force(position, goal);
        // 距離に応じて重みを調整
        double weight = std::min(1.0, distance_to_final / config_.lookahead_distance);
        return {goal_force[0] * weight, goal_force[1] * weight};
    } else {
        // 経路追従を優先
        return {0.0, 0.0};
    }
}

size_t TVVFGenerator::find_closest_path_point(const Position& position, const Path& planned_path) const {
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < planned_path.size(); ++i) {
        double distance = position.distance_to(planned_path[i].position);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }

    return closest_idx;
}

size_t TVVFGenerator::find_lookahead_point(const Position& position, const Path& planned_path, size_t start_idx) const {
    double accumulated_distance = 0.0;
    Position current_pos = position;

    for (size_t i = start_idx; i < planned_path.size(); ++i) {
        const Position& path_point_pos = planned_path[i].position;
        double segment_distance = current_pos.distance_to(path_point_pos);
        accumulated_distance += segment_distance;

        if (accumulated_distance >= config_.lookahead_distance) {
            return i;
        }

        current_pos = path_point_pos;
    }

    // 経路終端に到達
    return planned_path.size() - 1;
}

std::array<double, 2> TVVFGenerator::compute_cross_track_error(const Position& position,
                                                              const Path& planned_path,
                                                              size_t closest_idx) const {
    if (closest_idx >= planned_path.size() - 1) {
        return {0.0, 0.0};
    }

    // 現在の経路セグメント
    const Position& current_point = planned_path[closest_idx].position;
    const Position& next_point = planned_path[closest_idx + 1].position;

    // 経路方向ベクトル
    std::array<double, 2> path_direction = {next_point.x - current_point.x,
                                           next_point.y - current_point.y};
    double path_length = std::sqrt(path_direction[0] * path_direction[0] +
                                  path_direction[1] * path_direction[1]);

    if (path_length < config_.min_distance) {
        return {0.0, 0.0};
    }

    std::array<double, 2> path_unit = {path_direction[0] / path_length,
                                      path_direction[1] / path_length};

    // 現在位置から経路点へのベクトル
    std::array<double, 2> position_vector = {position.x - current_point.x,
                                            position.y - current_point.y};

    // 経路に垂直な成分（横方向誤差）
    double cross_track_distance = position_vector[0] * (-path_unit[1]) + position_vector[1] * path_unit[0];
    std::array<double, 2> cross_track_vector = {-path_unit[1] * cross_track_distance,
                                               path_unit[0] * cross_track_distance};

    return cross_track_vector;
}


std::array<double, 2> TVVFGenerator::clip_magnitude(const std::array<double, 2>& vector, double max_magnitude) const {
    double magnitude = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (magnitude > max_magnitude) {
        double scale = max_magnitude / magnitude;
        return {vector[0] * scale, vector[1] * scale};
    }
    return vector;
}

std::array<double, 2> TVVFGenerator::safe_normalize(const std::array<double, 2>& vector, double min_norm) const {
    double norm = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (norm < min_norm) {
        return {0.0, 0.0};
    }
    return {vector[0] / norm, vector[1] / norm};
}

std::array<double, 2> TVVFGenerator::safe_normalize_with_default(const std::array<double, 2>& vector,
                                                                double min_norm,
                                                                const std::array<double, 2>& default_value) const {
    double norm = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (norm < min_norm) {
        return default_value;
    }
    return {vector[0] / norm, vector[1] / norm};
}

std::array<double, 2> TVVFGenerator::adaptive_force_composition(const Position& position,
                                                              const std::array<double, 2>& path_force,
                                                              const std::array<double, 2>& goal_force,
                                                              const std::array<double, 2>& repulsive_force,
                                                              const std::array<double, 2>& /* time_correction */,
                                                              const std::vector<DynamicObstacle>& obstacles) const {

    // ============== 改善された統合ベクトル場合成手法 ==============

    // 1. 各力の大きさと方向を計算
    double path_magnitude = std::sqrt(path_force[0] * path_force[0] + path_force[1] * path_force[1]);
    double repulsive_magnitude = std::sqrt(repulsive_force[0] * repulsive_force[0] + repulsive_force[1] * repulsive_force[1]);
    double goal_magnitude = std::sqrt(goal_force[0] * goal_force[0] + goal_force[1] * goal_force[1]);

    // 2. 正規化された方向ベクトル
    std::array<double, 2> path_dir = safe_normalize({path_force[0], path_force[1]});
    std::array<double, 2> repulsive_dir = safe_normalize({repulsive_force[0], repulsive_force[1]});

    // 3. 簡素化された障害物距離評価
    double min_obstacle_distance = std::numeric_limits<double>::max();

    for (const auto& obstacle : obstacles) {
        double distance = position.distance_to(obstacle.position) - obstacle.radius;
        min_obstacle_distance = std::min(min_obstacle_distance, distance);
    }

    // 4. 影響圈ベースの危険度レベル計算
    double influence_level = 0.0;
    if (min_obstacle_distance < config_.influence_radius) {
        influence_level = std::max(0.0, 1.0 - min_obstacle_distance / config_.influence_radius);
    }

    // 5. 経路と斥力の方向一致度分析
    double path_repulsive_alignment = 0.0;
    if (path_magnitude > 1e-6 && repulsive_magnitude > 1e-6) {
        path_repulsive_alignment = path_dir[0] * repulsive_dir[0] + path_dir[1] * repulsive_dir[1];
    }

    // 6. 滑らかな回避のための接線ベクトル計算
    std::array<double, 2> tangent_force = {0.0, 0.0};
    if (repulsive_magnitude > 1e-6 && path_magnitude > 1e-6 && path_repulsive_alignment < -0.2) {
        // 経路方向に直交する成分を抽出（滑らかな回避）
        double cross_product = path_dir[0] * repulsive_dir[1] - path_dir[1] * repulsive_dir[0];
        std::array<double, 2> tangent_dir = {-path_dir[1], path_dir[0]};
        if (cross_product < 0) {
            tangent_dir[0] = -tangent_dir[0];
            tangent_dir[1] = -tangent_dir[1];
        }

        double tangent_strength = repulsive_magnitude * 0.8 * std::abs(path_repulsive_alignment);
        tangent_force[0] = tangent_strength * tangent_dir[0];
        tangent_force[1] = tangent_strength * tangent_dir[1];
    }

    // 7. シンプルな統合戦略（指数的斥力に依存）
    std::array<double, 2> result_force;

    if (influence_level > 0.1) {
        // ==== 障害物影響圏内：経路と斥力のバランス ====
        double path_weight = 2.0;
        double repulsive_weight = 1.0 + influence_level * 2.0;
        double tangent_weight = 0.8;

        result_force = {
            path_weight * path_force[0] +
            repulsive_weight * repulsive_force[0] +
            tangent_weight * tangent_force[0],
            path_weight * path_force[1] +
            repulsive_weight * repulsive_force[1] +
            tangent_weight * tangent_force[1]
        };

    } else {
        // ==== 自由航行モード（障害物なし） ====
        double free_path_weight = 3.0;
        double free_goal_weight = (goal_magnitude > 0.1 && goal_magnitude < 2.0) ? 1.0 : 0.3;

        result_force = {
            free_path_weight * path_force[0] + free_goal_weight * goal_force[0],
            free_path_weight * path_force[1] + free_goal_weight * goal_force[1]
        };
    }

    return result_force;
}

PathProjection TVVFGenerator::find_forward_path_projection(const Position& position, const Path& planned_path) const {
    PathProjection result;
    result.valid = false;

    if (planned_path.size() < 2) {
        return result;
    }

    double min_distance = std::numeric_limits<double>::max();
    size_t best_segment = 0;
    double best_t = 0.0;
    Position best_projection;

    // 各経路セグメントに対して投影を計算
    for (size_t i = 0; i < planned_path.size() - 1; ++i) {
        const Position& p1 = planned_path[i].position;
        const Position& p2 = planned_path[i + 1].position;

        // セグメントベクトル
        std::array<double, 2> segment = {p2.x - p1.x, p2.y - p1.y};
        double segment_length_sq = segment[0] * segment[0] + segment[1] * segment[1];

        if (segment_length_sq < config_.min_distance * config_.min_distance) {
            continue; // セグメントが短すぎる
        }

        // 現在位置からセグメント開始点へのベクトル
        std::array<double, 2> to_position = {position.x - p1.x, position.y - p1.y};

        // セグメント上での投影パラメータ計算
        double t = (to_position[0] * segment[0] + to_position[1] * segment[1]) / segment_length_sq;
        t = std::max(0.0, std::min(1.0, t)); // [0,1]の範囲にクランプ

        // 投影点の計算
        Position projection(p1.x + t * segment[0], p1.y + t * segment[1]);
        double distance = position.distance_to(projection);

        // 前進方向を保証：現在のベストより前進している場合のみ更新
        if (distance < min_distance && (result.valid == false || i >= best_segment)) {
            min_distance = distance;
            best_segment = i;
            best_t = t;
            best_projection = projection;
            result.valid = true;
        }
    }

    if (!result.valid) {
        return result;
    }

    // 結果の設定
    result.segment_start_idx = best_segment;
    result.parameter_t = best_t;
    result.projection_point = best_projection;

    // 横方向距離の計算（符号付き）
    const Position& p1 = planned_path[best_segment].position;
    const Position& p2 = planned_path[best_segment + 1].position;
    std::array<double, 2> segment_dir = {p2.x - p1.x, p2.y - p1.y};
    double segment_length = std::sqrt(segment_dir[0] * segment_dir[0] + segment_dir[1] * segment_dir[1]);

    if (segment_length > config_.min_distance) {
        segment_dir[0] /= segment_length;
        segment_dir[1] /= segment_length;

        std::array<double, 2> to_robot = {position.x - best_projection.x, position.y - best_projection.y};
        result.cross_track_distance = to_robot[0] * (-segment_dir[1]) + to_robot[1] * segment_dir[0];
    } else {
        result.cross_track_distance = 0.0;
    }

    // 経路に沿った距離の計算
    result.along_track_distance = 0.0;
    for (size_t i = 0; i < best_segment; ++i) {
        result.along_track_distance += planned_path[i].position.distance_to(planned_path[i + 1].position);
    }
    result.along_track_distance += best_t * planned_path[best_segment].position.distance_to(planned_path[best_segment + 1].position);

    return result;
}

Position TVVFGenerator::compute_lookahead_target(const PathProjection& projection, const Path& planned_path) const {
    if (!projection.valid || planned_path.empty()) {
        return Position(0.0, 0.0);
    }

    double remaining_distance = config_.lookahead_distance;
    Position current_pos = projection.projection_point;

    // 投影点から経路に沿って先読み距離分進む
    for (size_t i = projection.segment_start_idx; i < planned_path.size() - 1; ++i) {
        const Position& next_point = planned_path[i + 1].position;
        double segment_distance = current_pos.distance_to(next_point);

        if (segment_distance >= remaining_distance) {
            // このセグメント内で目標点に到達
            std::array<double, 2> direction = {next_point.x - current_pos.x, next_point.y - current_pos.y};
            double norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1]);
            if (norm > config_.min_distance) {
                direction[0] /= norm;
                direction[1] /= norm;
                return Position(current_pos.x + remaining_distance * direction[0],
                              current_pos.y + remaining_distance * direction[1]);
            }
        }

        remaining_distance -= segment_distance;
        current_pos = next_point;
    }

    // 経路終端に到達
    return planned_path[planned_path.size() - 1].position;
}

std::array<double, 2> TVVFGenerator::compute_gentle_lateral_correction(const PathProjection& projection) const {
    if (!projection.valid) {
        return {0.0, 0.0};
    }

    // 横方向誤差に基づく軽微な復帰力
    double lateral_strength = config_.k_path_attraction * 0.5; // 軽微な力
    double decay_factor = std::exp(-std::abs(projection.cross_track_distance) / config_.lookahead_distance);

    // 横方向誤差を直接減らす力（簡略化）
    double correction_magnitude = lateral_strength * decay_factor * std::abs(projection.cross_track_distance);

    // 横方向誤差の符号に基づいて復帰方向を決定
    if (projection.cross_track_distance > 0) {
        return {0.0, -correction_magnitude};
    } else {
        return {0.0, correction_magnitude};
    }
}

std::array<double, 2> TVVFGenerator::compute_fluid_avoidance_vector(const Position& position,
                                                                   const DynamicObstacle& obstacle,
                                                                   const std::array<double, 2>& goal_direction) const {
    // 障害物への相対位置ベクトル（ロボットから障害物へ）
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x,
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);


    // 極近距離での緊急回避
    if (distance < config_.min_distance) {
        std::array<double, 2> escape_direction = safe_normalize_with_default(
            {position.x - obstacle.position.x, position.y - obstacle.position.y},
            1e-8, {1.0, 0.0});
        return {config_.k_repulsion * 10.0 * escape_direction[0],
                config_.k_repulsion * 10.0 * escape_direction[1]};
    }

    // 障害物への方向を正規化
    std::array<double, 2> obstacle_direction = safe_normalize(to_obstacle);

    // 水が流れるような滑らかなベクトル場の計算
    // 1. 接線成分：障害物の周りを流れるような成分
    std::array<double, 2> tangent_vector = {-obstacle_direction[1], obstacle_direction[0]};

    // 2. ゴール方向との内積で流れの向きを決定
    double goal_tangent_dot = goal_direction[0] * tangent_vector[0] + goal_direction[1] * tangent_vector[1];
    if (goal_tangent_dot < 0) {
        tangent_vector[0] = -tangent_vector[0];
        tangent_vector[1] = -tangent_vector[1];
    }

    // 3. 障害物半径ベースの距離因子計算
    double distance_factor;
    if (distance <= obstacle.radius * 2.0) {
        // 障害物近く：強い流れ
        distance_factor = 1.0;
    } else if (distance <= obstacle.radius * 4.0) {
        // 中間距離：中程度の流れ
        double normalized_dist = (distance - obstacle.radius * 2.0) / (obstacle.radius * 2.0);
        distance_factor = 0.8 * (1.0 - normalized_dist);
    } else {
        // 遠距離：指数的減衰
        double decay_distance = distance - obstacle.radius * 4.0;
        distance_factor = 0.3 * std::exp(-decay_distance * 0.5);
    }

    // 4. 放射成分：障害物から離れる成分（弱い）
    std::array<double, 2> radial_vector = {position.x - obstacle.position.x,
                                          position.y - obstacle.position.y};
    std::array<double, 2> radial_direction = safe_normalize(radial_vector);

    // 5. 流体の特性を模擬：接線成分を主体とし、放射成分を補助的に使用
    double tangent_weight = 0.8; // 接線成分の重み（流れの主成分）
    double radial_weight = 0.2;   // 放射成分の重み（軽微な反発）

    // 6. 最終的な滑らかなベクトル場（流体強度係数を適用）
    std::array<double, 2> fluid_vector = {
        config_.k_repulsion * config_.fluid_strength_factor * distance_factor * (
            tangent_weight * tangent_vector[0] + radial_weight * radial_direction[0]
        ),
        config_.k_repulsion * config_.fluid_strength_factor * distance_factor * (
            tangent_weight * tangent_vector[1] + radial_weight * radial_direction[1]
        )
    };

    return fluid_vector;
}



std::array<double, 2> TVVFGenerator::compute_path_lookahead_direction(const Position& position,
                                                                     const Path& planned_path) const {
    if (planned_path.empty()) {
        return {1.0, 0.0}; // デフォルト方向
    }

    // 1. 現在位置に最も近い経路点を見つける
    size_t closest_idx = find_closest_path_point(position, planned_path);

    // 2. 先読み距離に基づいて目標点を見つける
    double accumulated_distance = 0.0;
    Position current_pos = position;

    // 最も近い点から先読み距離分前進
    for (size_t i = closest_idx; i < planned_path.size() - 1; ++i) {
        const Position& next_point = planned_path[i + 1].position;
        double segment_distance = current_pos.distance_to(next_point);

        if (accumulated_distance + segment_distance >= config_.lookahead_distance) {
            // 先読み距離に到達
            double remaining_distance = config_.lookahead_distance - accumulated_distance;
            double ratio = remaining_distance / segment_distance;

            Position target_point(
                current_pos.x + ratio * (next_point.x - current_pos.x),
                current_pos.y + ratio * (next_point.y - current_pos.y)
            );

            std::array<double, 2> direction = {target_point.x - position.x, target_point.y - position.y};
            return safe_normalize_with_default(direction, 1e-8, {1.0, 0.0});
        }

        accumulated_distance += segment_distance;
        current_pos = next_point;
    }

    // 経路終端の場合は最終点への方向
    const Position& final_point = planned_path[planned_path.size() - 1].position;
    std::array<double, 2> direction = {final_point.x - position.x, final_point.y - position.y};
    return safe_normalize_with_default(direction, 1e-8, {1.0, 0.0});
}


std::array<double, 2> TVVFGenerator::compute_exponential_repulsive_force(const Position& position,
                                                                        const DynamicObstacle& obstacle) const {
    // 障害物への相対位置ベクトル（ロボットから障害物へ）
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x,
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);


    // 最大指数的斥力適用距離外の場合はゼロベクトル
    if (distance > config_.max_exponential_distance) {
        return {0.0, 0.0};
    }


    // 極近距離での緊急回避
    if (distance < config_.min_distance) {
        std::array<double, 2> escape_direction = safe_normalize_with_default(
            {position.x - obstacle.position.x, position.y - obstacle.position.y},
            1e-8, {1.0, 0.0});
        return {config_.k_repulsion * 50.0 * escape_direction[0],
                config_.k_repulsion * 50.0 * escape_direction[1]};
    }

    // 障害物から離れる方向
    std::array<double, 2> repulsive_direction = safe_normalize(
        {position.x - obstacle.position.x, position.y - obstacle.position.y});

    // 指数的斥力強度計算
    double force_magnitude;

    if (distance <= config_.exponential_smoothing_threshold) {
        // 滑らか化閾値以下：非常に強い線形斥力（数値的安定性のため）
        double linear_factor = (config_.exponential_smoothing_threshold - distance) /
                              config_.exponential_smoothing_threshold;
        force_magnitude = config_.k_repulsion * config_.exponential_scale_factor *
                         (10.0 + linear_factor * 40.0); // 強力な線形増加
    } else {
        // 指数的斥力計算：距離が近づくほど指数的に増加
        // f(d) = k * scale * base^((max_dist - d) / max_dist)
        double normalized_distance = (config_.max_exponential_distance - distance) /
                                   config_.max_exponential_distance;
        double exponential_factor = std::pow(config_.exponential_base,
                                           normalized_distance * 3.0); // 3.0は指数の強度調整

        force_magnitude = config_.k_repulsion * config_.exponential_scale_factor * exponential_factor;


        // 数値オーバーフロー防止
        // max_force制限を削除
    }

    // 障害物半径ベースの遠距離減衰
    if (distance > obstacle.radius * 4.0) {
        double decay_factor = std::exp(-(distance - obstacle.radius * 4.0) * 0.3);
        force_magnitude *= decay_factor;
    }

    std::array<double, 2> result = {force_magnitude * repulsive_direction[0],
                                    force_magnitude * repulsive_direction[1]};


    return result;
}

std::array<double, 2> TVVFGenerator::compute_exponential_integrated_avoidance_vector(const Position& position,
                                                                                   const DynamicObstacle& obstacle,
                                                                                   const std::array<double, 2>& path_direction,
                                                                                   const Path& /*planned_path*/) const {
    // 1. 指数的斥力を計算
    auto exponential_repulsive_vector = compute_exponential_repulsive_force(position, obstacle);

    // 2. 経路方向を考慮した流体ベクトル場を計算（既存の実装を再利用）
    auto fluid_vector = compute_fluid_avoidance_vector(position, obstacle, path_direction);

    // 3. 経路方向成分を計算（既存の実装と同じ）
    double path_strength = config_.k_path_attraction * config_.path_direction_weight;
    std::array<double, 2> path_component = {path_strength * path_direction[0],
                                           path_strength * path_direction[1]};

    // 4. 重み計算（距離別重み調整機能を削除）
    double exponential_repulsive_weight = config_.repulsive_weight;
    double fluid_weight = config_.fluid_weight;
    double path_weight = config_.path_direction_weight;

    // 7. 数値安定性（max_force制限を削除）

    // 8. 重み付き統合
    std::array<double, 2> integrated_vector = {
        exponential_repulsive_weight * exponential_repulsive_vector[0] +
        fluid_weight * fluid_vector[0] +
        path_weight * path_component[0],
        exponential_repulsive_weight * exponential_repulsive_vector[1] +
        fluid_weight * fluid_vector[1] +
        path_weight * path_component[1]
    };

    // 9. 最終統合（max_force制限を削除）

    return integrated_vector;
}

} // namespace tvvf_vo_c