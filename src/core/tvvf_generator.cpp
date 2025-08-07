#include "tvvf_vo_c/core/tvvf_generator.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace tvvf_vo_c {

TVVFGenerator::TVVFGenerator(const TVVFVOConfig& config) : config_(config) {}

std::array<double, 2> TVVFGenerator::compute_vector(const Position& position, double time,
                                                    const Goal& goal,
                                                    const std::vector<DynamicObstacle>& obstacles,
                                                    const std::optional<Path>& planned_path) const {
    // 1. A*経路の存在チェック
    if (!planned_path.has_value() || planned_path->empty()) {
        // A*経路がない場合は基本的なゴール向けベクトル場を生成（指数的斥力対応）
        auto goal_force = compute_attractive_force(position, goal);
        std::array<double, 2> repulsive_force = {0.0, 0.0};
        
        // 障害物回避力計算（指数的斥力対応）
        static int no_path_debug_counter = 0;
        no_path_debug_counter++;
        if (no_path_debug_counter % 50 == 0) {
            std::cout << "[NO PATH DEBUG] Obstacles: " << obstacles.size() 
                      << ", Exponential enabled: " << config_.enable_exponential_repulsion << std::endl;
        }
        
        for (const auto& obstacle : obstacles) {
            std::array<double, 2> single_repulsive;
            if (config_.enable_exponential_repulsion) {
                // 指数的斥力を使用
                single_repulsive = compute_exponential_repulsive_force(position, obstacle);
                if (no_path_debug_counter % 50 == 0) {
                    double distance = std::sqrt((obstacle.position.x - position.x) * (obstacle.position.x - position.x) + 
                                               (obstacle.position.y - position.y) * (obstacle.position.y - position.y));
                    std::cout << "[NO PATH DEBUG] Exponential force for obstacle " << obstacle.id 
                              << " (dist: " << distance << "): (" << single_repulsive[0] << ", " << single_repulsive[1] << ")" << std::endl;
                }
            } else {
                // 従来の放射状斥力を使用
                single_repulsive = compute_radial_repulsive_force(position, obstacle);
            }
            repulsive_force[0] += single_repulsive[0];
            repulsive_force[1] += single_repulsive[1];
        }
        
        if (no_path_debug_counter % 50 == 0) {
            std::cout << "[NO PATH DEBUG] Total repulsive force: (" << repulsive_force[0] << ", " << repulsive_force[1] << ")" << std::endl;
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
    
    // デバッグ出力: 障害物数と指数的斥力設定
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter % 100 == 0) { // 100フレームごとに出力
        std::cout << "[TVVF DEBUG] Obstacles count: " << obstacles.size() 
                  << ", Exponential repulsion enabled: " << config_.enable_exponential_repulsion << std::endl;
    }
    
    for (const auto& obstacle : obstacles) {
        // 距離計算（デバッグ用）
        double distance = std::sqrt((obstacle.position.x - position.x) * (obstacle.position.x - position.x) + 
                                   (obstacle.position.y - position.y) * (obstacle.position.y - position.y));
        
        std::array<double, 2> integrated_vector;
        if (config_.enable_exponential_repulsion) {
            // 指数的斥力統合版を使用
            integrated_vector = compute_exponential_integrated_avoidance_vector(position, obstacle, path_direction, planned_path.value());
            
            // デバッグ出力: 指数的斥力ベクトル
            if (debug_counter % 50 == 0) {
                std::cout << "[TVVF DEBUG] Exponential repulsion - Obstacle ID: " << obstacle.id 
                          << ", Distance: " << distance 
                          << ", Force: (" << integrated_vector[0] << ", " << integrated_vector[1] << ")" << std::endl;
            }
        } else {
            // 従来の統合版を使用
            integrated_vector = compute_path_integrated_avoidance_vector(position, obstacle, path_direction, planned_path.value());
            
            // デバッグ出力: 従来の斥力ベクトル
            if (debug_counter % 50 == 0) {
                std::cout << "[TVVF DEBUG] Traditional repulsion - Obstacle ID: " << obstacle.id 
                          << ", Distance: " << distance 
                          << ", Force: (" << integrated_vector[0] << ", " << integrated_vector[1] << ")" << std::endl;
            }
        }
        repulsive_force[0] += integrated_vector[0];
        repulsive_force[1] += integrated_vector[1];
    }
    
    // デバッグ出力: 総斥力
    if (debug_counter % 100 == 0) {
        std::cout << "[TVVF DEBUG] Total repulsive force: (" << repulsive_force[0] << ", " << repulsive_force[1] << ")" << std::endl;
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

std::array<double, 2> TVVFGenerator::compute_repulsive_force(const Position& position, double time,
                                                            const std::vector<DynamicObstacle>& obstacles) const {
    // この関数は後方互換性のために残しておき、元の実装を保持
    std::array<double, 2> total_repulsive = {0.0, 0.0};
    
    for (const auto& obstacle : obstacles) {
        auto potential_gradient = compute_dynamic_potential_gradient(position, obstacle, time);
        total_repulsive[0] += potential_gradient[0];
        total_repulsive[1] += potential_gradient[1];
    }
    
    return total_repulsive;
}

std::array<double, 2> TVVFGenerator::compute_dynamic_potential_gradient(const Position& position,
                                                                       const DynamicObstacle& obstacle,
                                                                       double /* time */) const {
    std::array<double, 2> current_relative_pos = {position.x - obstacle.position.x,
                                                  position.y - obstacle.position.y};
    double current_distance = std::sqrt(current_relative_pos[0] * current_relative_pos[0] + 
                                       current_relative_pos[1] * current_relative_pos[1]);
    
    // 斥力の影響範囲を中距離閾値で制限
    double max_repulsive_distance = std::max(config_.mid_distance_threshold, config_.safety_margin * 3.0);
    if (current_distance > max_repulsive_distance) {
        return {0.0, 0.0};
    }
    
    // 改善された斥力計算：段階的で滑らかな力の変化
    std::array<double, 2> force_direction;
    double force_magnitude;
    
    if (current_distance < config_.min_distance) {
        // 極近距離：強制的な反発
        force_direction = safe_normalize_with_default(current_relative_pos, 1e-8, {1.0, 0.0});
        force_magnitude = config_.k_repulsion * 100.0;
    } else {
        force_direction = safe_normalize(current_relative_pos);
        double effective_radius = obstacle.radius + config_.safety_margin;
        
        if (current_distance <= config_.near_distance_threshold) {
            // 近距離：強い反比例斥力
            double distance_ratio = current_distance / config_.near_distance_threshold;
            force_magnitude = config_.k_repulsion * (1.0 / distance_ratio - 1.0) * 2.0;
            
        } else if (current_distance <= config_.mid_distance_threshold) {
            // 中距離：中程度の斥力（二次関数的減衰）
            double range = config_.mid_distance_threshold - config_.near_distance_threshold;
            double distance_ratio = (current_distance - config_.near_distance_threshold) / range;
            double quadratic_decay = (1.0 - distance_ratio) * (1.0 - distance_ratio);
            force_magnitude = config_.k_repulsion * 0.8 * quadratic_decay;
            
        } else {
            // 遠距離：弱い指数減衰斥力
            double range = max_repulsive_distance - config_.mid_distance_threshold;
            double decay_factor = std::exp(-(current_distance - config_.mid_distance_threshold) / range);
            force_magnitude = config_.k_repulsion * 0.3 * decay_factor;
        }
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

std::array<double, 2> TVVFGenerator::compute_time_correction(const Position& position, double /* time */,
                                                            const std::vector<DynamicObstacle>& obstacles,
                                                            const Goal& goal) const {
    std::array<double, 2> correction = {0.0, 0.0};
    std::vector<double> prediction_times = {0.5, 1.0, 2.0};
    
    for (double pred_time : prediction_times) {
        auto time_correction = compute_prediction_correction(position, obstacles, pred_time, goal);
        double time_weight = std::exp(-pred_time / 2.0);
        correction[0] += time_weight * time_correction[0];
        correction[1] += time_weight * time_correction[1];
    }
    
    return {correction[0] * 0.1, correction[1] * 0.1};  // 時間補正の影響をさらに軽減
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

std::array<double, 2> TVVFGenerator::compute_prediction_correction(const Position& position,
                                                                  const std::vector<DynamicObstacle>& obstacles,
                                                                  double prediction_time,
                                                                  const Goal& /* goal */) const {
    std::array<double, 2> correction = {0.0, 0.0};
    
    for (const auto& obstacle : obstacles) {
        Position predicted_pos = obstacle.predict_position(prediction_time);
        std::array<double, 2> predicted_relative = {position.x - predicted_pos.x,
                                                    position.y - predicted_pos.y};
        double predicted_distance = std::sqrt(predicted_relative[0] * predicted_relative[0] + 
                                             predicted_relative[1] * predicted_relative[1]);
        
        if (predicted_distance < config_.influence_radius) {
            double avoidance_strength = (config_.influence_radius - predicted_distance) / config_.influence_radius;
            auto avoidance_direction = safe_normalize(predicted_relative);
            correction[0] += avoidance_strength * avoidance_direction[0] * 0.5;
            correction[1] += avoidance_strength * avoidance_direction[1] * 0.5;
        }
    }
    
    return correction;
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
    
    // 3. 詳細な障害物危険度評価（複数段階）
    double min_obstacle_distance = std::numeric_limits<double>::max();
    double critical_distance = std::numeric_limits<double>::max();
    double warning_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : obstacles) {
        double distance = position.distance_to(obstacle.position) - obstacle.radius;
        min_obstacle_distance = std::min(min_obstacle_distance, distance);
        
        // 臨界距離：安全マージン以内
        if (distance <= config_.safety_margin) {
            critical_distance = std::min(critical_distance, distance);
        }
        // 警告距離：安全マージンの1.5倍以内
        if (distance <= config_.safety_margin * 1.5) {
            warning_distance = std::min(warning_distance, distance);
        }
    }
    
    // 4. 段階的危険度レベル計算
    double critical_level = 0.0;
    double warning_level = 0.0;
    double influence_level = 0.0;
    
    if (critical_distance < std::numeric_limits<double>::max()) {
        critical_level = std::max(0.0, 1.0 - critical_distance / config_.safety_margin);
    }
    if (warning_distance < std::numeric_limits<double>::max()) {
        warning_level = std::max(0.0, 1.0 - warning_distance / (config_.safety_margin * 1.5));
    }
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
    
    // 7. 階層的合成戦略
    std::array<double, 2> result_force;
    
    if (critical_level > 0.5) {
        // ==== 緊急回避モード（安全マージン内） ====
        // 斥力を最優先、経路は補助的
        double emergency_repulsive_weight = 4.0 + critical_level * 2.0;
        double emergency_path_weight = 0.3 * (1.0 - critical_level);
        double emergency_tangent_weight = 1.2;
        
        result_force = {
            emergency_repulsive_weight * repulsive_force[0] + 
            emergency_path_weight * path_force[0] +
            emergency_tangent_weight * tangent_force[0],
            emergency_repulsive_weight * repulsive_force[1] + 
            emergency_path_weight * path_force[1] +
            emergency_tangent_weight * tangent_force[1]
        };
        
    } else if (warning_level > 0.2) {
        // ==== 積極的回避モード（警告距離内） ====
        // 斥力と経路のバランス、接線成分で滑らかに
        double avoidance_repulsive_weight = 2.0 + warning_level * 2.0;
        double avoidance_path_weight = 1.5 * (1.0 - warning_level * 0.5);
        double avoidance_tangent_weight = 1.0;
        
        result_force = {
            avoidance_repulsive_weight * repulsive_force[0] + 
            avoidance_path_weight * path_force[0] +
            avoidance_tangent_weight * tangent_force[0],
            avoidance_repulsive_weight * repulsive_force[1] + 
            avoidance_path_weight * path_force[1] +
            avoidance_tangent_weight * tangent_force[1]
        };
        
    } else if (influence_level > 0.1) {
        // ==== 予防的回避モード（影響圏内） ====
        // 経路優先、斥力は予防的
        double preventive_path_weight = 2.5;
        double preventive_repulsive_weight = 0.8 + influence_level * 1.2;
        double preventive_tangent_weight = 0.6;
        
        result_force = {
            preventive_path_weight * path_force[0] + 
            preventive_repulsive_weight * repulsive_force[0] +
            preventive_tangent_weight * tangent_force[0],
            preventive_path_weight * path_force[1] + 
            preventive_repulsive_weight * repulsive_force[1] +
            preventive_tangent_weight * tangent_force[1]
        };
        
    } else {
        // ==== 自由航行モード（障害物なし） ====
        // 経路追従優先、ゴール引力も考慮
        double free_path_weight = 3.0;
        double free_goal_weight = (goal_magnitude > 0.1 && goal_magnitude < 2.0) ? 1.0 : 0.3;
        
        result_force = {
            free_path_weight * path_force[0] + free_goal_weight * goal_force[0],
            free_path_weight * path_force[1] + free_goal_weight * goal_force[1]
        };
    }
    
    // 8. 最終的な安全チェック（max_force制限を削除）
    
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
    
    
    // 安全距離（障害物半径 + 安全マージン）
    double safe_distance = obstacle.radius + config_.safety_margin;
    
    // 極近距離での緊急回避
    if (distance < config_.min_distance) {
        std::array<double, 2> escape_direction = safe_normalize_with_default(
            {position.x - obstacle.position.x, position.y - obstacle.position.y}, 
            1e-8, {1.0, 0.0});
        return {config_.k_repulsion * 10.0 * escape_direction[0],
                config_.k_repulsion * 10.0 * escape_direction[1]};
    }
    
    // 障害物への方向を正規化（要件：向きはロボットから障害物へ）
    std::array<double, 2> obstacle_direction = safe_normalize(to_obstacle);
    
    // 水が流れるような滑らかなベクトル場の計算
    // 障害物周りで渦のような流れを作る
    
    // 1. 接線成分：障害物の周りを流れるような成分
    std::array<double, 2> tangent_vector = {-obstacle_direction[1], obstacle_direction[0]};
    
    // 2. ゴール方向との内積で流れの向きを決定
    double goal_tangent_dot = goal_direction[0] * tangent_vector[0] + goal_direction[1] * tangent_vector[1];
    if (goal_tangent_dot < 0) {
        // ゴール方向と逆の場合は接線方向を反転
        tangent_vector[0] = -tangent_vector[0];
        tangent_vector[1] = -tangent_vector[1];
    }
    
    // 3. 距離に基づく重み計算
    double distance_factor;
    if (distance <= safe_distance) {
        // 安全距離内：強い流れ
        distance_factor = 1.0;
    } else if (distance <= safe_distance * 2.0) {
        // 中間距離：中程度の流れ
        double normalized_dist = (distance - safe_distance) / safe_distance;
        distance_factor = 0.8 * (1.0 - normalized_dist);
    } else {
        // 外側：距離に応じて指数関数的に減衰する弱い流れ
        double decay_distance = distance - safe_distance * 2.0;
        distance_factor = 0.3 * std::exp(-decay_distance * 0.5);  // 0.5は減衰係数
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

std::array<double, 2> TVVFGenerator::compute_radial_repulsive_force(const Position& position,
                                                                   const DynamicObstacle& obstacle) const {
    // 障害物への相対位置ベクトル（ロボットから障害物へ）
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x, 
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);
    
    // 影響圏外の場合はゼロベクトル
    if (distance > config_.influence_radius) {
        return {0.0, 0.0};
    }
    
    // 安全距離（障害物半径 + 安全マージン）
    double safe_distance = obstacle.radius + config_.safety_margin;
    
    // 極近距離での緊急回避
    if (distance < config_.min_distance) {
        std::array<double, 2> escape_direction = safe_normalize_with_default(
            {position.x - obstacle.position.x, position.y - obstacle.position.y}, 
            1e-8, {1.0, 0.0});
        return {config_.k_repulsion * 20.0 * escape_direction[0],
                config_.k_repulsion * 20.0 * escape_direction[1]};
    }
    
    // 障害物から離れる方向（放射状斥力）
    std::array<double, 2> repulsive_direction = safe_normalize(
        {position.x - obstacle.position.x, position.y - obstacle.position.y});
    
    // 距離に基づく斥力強度計算
    double force_magnitude;
    if (distance <= safe_distance) {
        // 安全距離内：強い斥力
        force_magnitude = config_.k_repulsion * (safe_distance / distance - 1.0) * 2.0;
    } else if (distance <= safe_distance * 2.0) {
        // 中間距離：中程度の斥力
        double normalized_dist = (distance - safe_distance) / safe_distance;
        force_magnitude = config_.k_repulsion * (1.0 - normalized_dist);
    } else {
        // 外側：弱い斥力
        double normalized_dist = (distance - safe_distance * 2.0) / (config_.influence_radius - safe_distance * 2.0);
        force_magnitude = config_.k_repulsion * 0.3 * std::exp(-normalized_dist * 2.0);
    }
    
    return {force_magnitude * repulsive_direction[0],
            force_magnitude * repulsive_direction[1]};
}

std::array<double, 2> TVVFGenerator::compute_combined_avoidance_vector(const Position& position,
                                                                      const DynamicObstacle& obstacle,
                                                                      const std::array<double, 2>& goal_direction) const {
    // 1. 放射状斥力を計算
    auto repulsive_vector = compute_radial_repulsive_force(position, obstacle);
    
    // 2. 流体ベクトル場を計算
    auto fluid_vector = compute_fluid_avoidance_vector(position, obstacle, goal_direction);
    
    // 3. 距離に基づく重み調整
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x, 
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);
    double safe_distance = obstacle.radius + config_.safety_margin;
    
    // 距離に応じて斥力と流体の重みを動的に調整
    double repulsive_weight, fluid_weight;
    if (distance <= safe_distance) {
        // 近距離：斥力を強化
        repulsive_weight = 0.8;
        fluid_weight = 0.2;
    } else if (distance <= safe_distance * 2.0) {
        // 中距離：バランス調整
        double blend_factor = (distance - safe_distance) / safe_distance;
        repulsive_weight = 0.8 - 0.4 * blend_factor;  // 0.8 -> 0.4
        fluid_weight = 0.2 + 0.4 * blend_factor;      // 0.2 -> 0.6
    } else {
        // 遠距離：流体を強化
        repulsive_weight = config_.repulsive_weight;
        fluid_weight = config_.fluid_weight;
    }
    
    // 4. 重み付き合成
    std::array<double, 2> combined_vector = {
        repulsive_weight * repulsive_vector[0] + fluid_weight * fluid_vector[0],
        repulsive_weight * repulsive_vector[1] + fluid_weight * fluid_vector[1]
    };
    
    return combined_vector;
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

std::array<double, 2> TVVFGenerator::compute_path_integrated_avoidance_vector(const Position& position,
                                                                             const DynamicObstacle& obstacle,
                                                                             const std::array<double, 2>& path_direction,
                                                                             const Path& planned_path) const {
    // 1. 従来の放射状斥力を計算
    auto repulsive_vector = compute_radial_repulsive_force(position, obstacle);
    
    // 2. 経路方向を考慮した流体ベクトル場を計算
    auto fluid_vector = compute_fluid_avoidance_vector(position, obstacle, path_direction);
    
    // 3. 経路方向成分を計算（常に未到達の経路方向を向く）
    double path_strength = config_.k_path_attraction * config_.path_direction_weight;
    std::array<double, 2> path_component = {path_strength * path_direction[0],
                                           path_strength * path_direction[1]};
    
    // 4. 距離に基づく重み調整
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x, 
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);
    double safe_distance = obstacle.radius + config_.safety_margin;
    
    // 5. 明確な距離区分に基づく統合重み計算
    double repulsive_weight, fluid_weight, path_weight;
    
    if (distance <= config_.near_distance_threshold) {
        // 近距離：安全優先（斥力中心）
        repulsive_weight = config_.near_repulsive_weight;
        fluid_weight = config_.near_fluid_weight;
        path_weight = config_.near_path_weight;
    } else if (distance <= config_.mid_distance_threshold) {
        // 中距離：バランス調整（線形補間）
        double range = config_.mid_distance_threshold - config_.near_distance_threshold;
        double blend_factor = (distance - config_.near_distance_threshold) / range;
        repulsive_weight = config_.near_repulsive_weight - 
                          (config_.near_repulsive_weight - config_.mid_repulsive_weight) * blend_factor;
        fluid_weight = config_.near_fluid_weight + 
                      (config_.mid_fluid_weight - config_.near_fluid_weight) * blend_factor;
        path_weight = config_.near_path_weight + 
                     (config_.mid_path_weight - config_.near_path_weight) * blend_factor;
    } else {
        // 遠距離：経路追従重視
        repulsive_weight = config_.repulsive_weight * 0.3;  // 設定値をさらに減衰
        fluid_weight = config_.fluid_weight * 0.5;          // 設定値を減衰
        path_weight = 0.5; // 経路方向を強化
    }
    
    // 6. 障害物との角度関係を考慮した追加調整
    // 経路方向と障害物方向の角度を計算
    std::array<double, 2> obstacle_direction = safe_normalize(to_obstacle);
    double path_obstacle_dot = path_direction[0] * obstacle_direction[0] + 
                              path_direction[1] * obstacle_direction[1];
    
    // 経路方向に障害物がある場合は回避を強化
    if (path_obstacle_dot > 0.5) { // 経路前方に障害物
        repulsive_weight *= 1.5;
        fluid_weight *= 1.3;
        path_weight *= 0.7; // 経路方向を一時的に弱める
    }
    
    // 7. 重み付き統合
    std::array<double, 2> integrated_vector = {
        repulsive_weight * repulsive_vector[0] + 
        fluid_weight * fluid_vector[0] + 
        path_weight * path_component[0],
        repulsive_weight * repulsive_vector[1] + 
        fluid_weight * fluid_vector[1] + 
        path_weight * path_component[1]
    };
    
    return integrated_vector;
}

std::array<double, 2> TVVFGenerator::compute_exponential_repulsive_force(const Position& position,
                                                                        const DynamicObstacle& obstacle) const {
    // 障害物への相対位置ベクトル（ロボットから障害物へ）
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x, 
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);
    
    // デバッグ出力: 距離と最大適用距離の比較
    static int exp_debug_counter = 0;
    exp_debug_counter++;
    if (exp_debug_counter % 50 == 0) {
        std::cout << "[EXP FORCE DEBUG] Obstacle ID: " << obstacle.id 
                  << ", Distance: " << distance 
                  << ", Max exp distance: " << config_.max_exponential_distance << std::endl;
    }
    
    // 最大指数的斥力適用距離外の場合はゼロベクトル
    if (distance > config_.max_exponential_distance) {
        if (exp_debug_counter % 50 == 0) {
            std::cout << "[EXP FORCE DEBUG] Outside max distance, returning zero force" << std::endl;
        }
        return {0.0, 0.0};
    }
    
    // 安全距離（障害物半径 + 安全マージン）
    double safe_distance = obstacle.radius + config_.safety_margin;
    
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
        
        // デバッグ出力: 指数計算の詳細
        if (exp_debug_counter % 50 == 0) {
            std::cout << "[EXP FORCE DEBUG] Exponential calculation - " 
                      << "normalized_dist: " << normalized_distance 
                      << ", exp_factor: " << exponential_factor 
                      << ", k_repulsion: " << config_.k_repulsion
                      << ", scale_factor: " << config_.exponential_scale_factor
                      << ", force_magnitude: " << force_magnitude << std::endl;
        }
        
        // 数値オーバーフロー防止
        // max_force制限を削除
    }
    
    // 距離による最小限の減衰（指数関数が主体だが、遠距離では減衰）
    if (distance > safe_distance * 2.0) {
        double decay_factor = std::exp(-(distance - safe_distance * 2.0) * 0.5);
        force_magnitude *= decay_factor;
    }
    
    std::array<double, 2> result = {force_magnitude * repulsive_direction[0],
                                    force_magnitude * repulsive_direction[1]};
    
    // デバッグ出力: 最終結果
    if (exp_debug_counter % 50 == 0) {
        std::cout << "[EXP FORCE DEBUG] Final exponential force: (" << result[0] << ", " << result[1] << ")" << std::endl;
    }
    
    return result;
}

std::array<double, 2> TVVFGenerator::compute_exponential_integrated_avoidance_vector(const Position& position,
                                                                                   const DynamicObstacle& obstacle,
                                                                                   const std::array<double, 2>& path_direction,
                                                                                   const Path& planned_path) const {
    // 1. 指数的斥力を計算
    auto exponential_repulsive_vector = compute_exponential_repulsive_force(position, obstacle);
    
    // 2. 経路方向を考慮した流体ベクトル場を計算（既存の実装を再利用）
    auto fluid_vector = compute_fluid_avoidance_vector(position, obstacle, path_direction);
    
    // 3. 経路方向成分を計算（既存の実装と同じ）
    double path_strength = config_.k_path_attraction * config_.path_direction_weight;
    std::array<double, 2> path_component = {path_strength * path_direction[0],
                                           path_strength * path_direction[1]};
    
    // 4. 距離計算
    std::array<double, 2> to_obstacle = {obstacle.position.x - position.x, 
                                        obstacle.position.y - position.y};
    double distance = std::sqrt(to_obstacle[0] * to_obstacle[0] + to_obstacle[1] * to_obstacle[1]);
    double safe_distance = obstacle.radius + config_.safety_margin;
    
    // 5. 指数的斥力に適応した重み調整
    double exponential_repulsive_weight, fluid_weight, path_weight;
    
    if (distance <= config_.near_distance_threshold) {
        // 近距離：指数的斥力を大幅に強化、他は大幅削減
        exponential_repulsive_weight = config_.near_repulsive_weight * 2.0; // 指数的斥力を強化
        fluid_weight = config_.near_fluid_weight * 0.5; // 流体を削減
        path_weight = config_.near_path_weight * 0.3;   // 経路を削減
    } else if (distance <= config_.mid_distance_threshold) {
        // 中距離：線形補間だが指数的斥力を優先
        double range = config_.mid_distance_threshold - config_.near_distance_threshold;
        double blend_factor = (distance - config_.near_distance_threshold) / range;
        
        double near_exp_weight = config_.near_repulsive_weight * 2.0;
        double mid_exp_weight = config_.mid_repulsive_weight * 1.5; // 中距離でも指数的斥力を強化
        
        exponential_repulsive_weight = near_exp_weight - (near_exp_weight - mid_exp_weight) * blend_factor;
        fluid_weight = config_.near_fluid_weight * 0.5 + 
                      (config_.mid_fluid_weight - config_.near_fluid_weight * 0.5) * blend_factor;
        path_weight = config_.near_path_weight * 0.3 + 
                     (config_.mid_path_weight - config_.near_path_weight * 0.3) * blend_factor;
    } else {
        // 遠距離：従来の重みを使用（指数的斥力は自然に減衰）
        exponential_repulsive_weight = config_.repulsive_weight * 0.8;
        fluid_weight = config_.fluid_weight * 0.6;
        path_weight = 0.6; // 経路追従を重視
    }
    
    // 6. 障害物との角度関係を考慮した追加調整（既存の実装と同じ）
    std::array<double, 2> obstacle_direction = safe_normalize(to_obstacle);
    double path_obstacle_dot = path_direction[0] * obstacle_direction[0] + 
                              path_direction[1] * obstacle_direction[1];
    
    if (path_obstacle_dot > 0.5) { // 経路前方に障害物
        exponential_repulsive_weight *= 1.3; // 指数的斥力をさらに強化
        fluid_weight *= 1.2;
        path_weight *= 0.7;
    }
    
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