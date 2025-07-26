#include "tvvf_vo_c/core/tvvf_generator.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace tvvf_vo_c {

TVVFGenerator::TVVFGenerator(const TVVFVOConfig& config) : config_(config) {}

std::array<double, 2> TVVFGenerator::compute_vector(const Position& position, double time,
                                                    const Goal& goal,
                                                    const std::vector<DynamicObstacle>& obstacles,
                                                    const std::optional<Path>& planned_path) {
    // 1. A*経路の存在チェック（必須）
    if (!planned_path.has_value() || planned_path->empty()) {
        // A*経路がない場合はゼロベクトルを返す
        return {0.0, 0.0};
    }
    
    // A*経路追従ベクトル場を使用（必須）
    auto path_force = compute_path_following_force(position, planned_path.value());
    auto goal_force = compute_adaptive_goal_force(position, goal, planned_path.value());
    
    // 2. 斥力場計算（時間依存・従来通り）
    auto repulsive_force = compute_repulsive_force(position, time, obstacles);
    
    // 3. 時間依存補正項
    auto time_correction = compute_time_correction(position, time, obstacles, goal);
    
    // 4. 合成（A*経路を優先）
    std::array<double, 2> total_force = {
        path_force[0] + goal_force[0] + repulsive_force[0] + time_correction[0],
        path_force[1] + goal_force[1] + repulsive_force[1] + time_correction[1]
    };
    
    // 5. 数値安定性とクリッピング
    total_force = clip_magnitude(total_force, config_.max_force);
    
    return total_force;
}

std::array<double, 2> TVVFGenerator::compute_attractive_force(const Position& position, const Goal& goal) {
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
                                                            const std::vector<DynamicObstacle>& obstacles) {
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
                                                                       double /* time */) {
    std::array<double, 2> current_relative_pos = {position.x - obstacle.position.x,
                                                  position.y - obstacle.position.y};
    double current_distance = std::sqrt(current_relative_pos[0] * current_relative_pos[0] + 
                                       current_relative_pos[1] * current_relative_pos[1]);
    
    if (current_distance > config_.influence_radius) {
        return {0.0, 0.0};
    }
    
    double min_distance_time = compute_minimum_distance_time(position, obstacle, config_.time_horizon);
    double time_weight = compute_time_dependent_weight(min_distance_time, current_distance, obstacle);
    
    std::array<double, 2> force_direction;
    double force_magnitude;
    
    if (current_distance < config_.min_distance) {
        force_direction = safe_normalize_with_default(current_relative_pos, 1e-8, {1.0, 0.0});
        force_magnitude = config_.k_repulsion * 100.0;
    } else {
        double effective_radius = obstacle.radius + config_.safety_margin;
        
        if (current_distance <= effective_radius) {
            force_magnitude = config_.k_repulsion * (1.0 / current_distance - 1.0 / effective_radius);
        } else {
            double decay_factor = std::exp(-(current_distance - effective_radius) / effective_radius);
            force_magnitude = config_.k_repulsion * 0.1 * decay_factor;
        }
        
        force_direction = safe_normalize(current_relative_pos);
    }
    
    return {force_magnitude * time_weight * force_direction[0],
            force_magnitude * time_weight * force_direction[1]};
}

double TVVFGenerator::compute_minimum_distance_time(const Position& position,
                                                   const DynamicObstacle& obstacle,
                                                   double horizon) {
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
                                                   const DynamicObstacle& obstacle) {
    double base_weight = 1.0;
    double time_urgency = std::exp(-min_distance_time / 1.0);
    double distance_urgency = std::exp(-current_distance / obstacle.radius);
    double total_weight = base_weight * (1.0 + time_urgency + distance_urgency);
    return std::min(total_weight, 10.0);
}

std::array<double, 2> TVVFGenerator::compute_time_correction(const Position& position, double /* time */,
                                                            const std::vector<DynamicObstacle>& obstacles,
                                                            const Goal& goal) {
    std::array<double, 2> correction = {0.0, 0.0};
    std::vector<double> prediction_times = {0.5, 1.0, 2.0};
    
    for (double pred_time : prediction_times) {
        auto time_correction = compute_prediction_correction(position, obstacles, pred_time, goal);
        double time_weight = std::exp(-pred_time / 2.0);
        correction[0] += time_weight * time_correction[0];
        correction[1] += time_weight * time_correction[1];
    }
    
    return {correction[0] * 0.2, correction[1] * 0.2};
}

std::array<double, 2> TVVFGenerator::compute_path_following_force(const Position& position,
                                                                 const Path& planned_path) {
    if (planned_path.empty()) {
        return {0.0, 0.0};
    }
    
    // 1. 現在位置に最も近い経路点を見つける
    size_t closest_point_idx = find_closest_path_point(position, planned_path);
    
    // 2. 先読み距離分だけ前の点を目標とする
    size_t target_point_idx = find_lookahead_point(position, planned_path, closest_point_idx);
    
    if (target_point_idx >= planned_path.size()) {
        target_point_idx = planned_path.size() - 1;
    }
    
    const Position& target_position = planned_path[target_point_idx].position;
    
    // 3. 目標点への方向ベクトル計算
    std::array<double, 2> direction_vector = {target_position.x - position.x,
                                             target_position.y - position.y};
    double distance = std::sqrt(direction_vector[0] * direction_vector[0] + 
                               direction_vector[1] * direction_vector[1]);
    
    if (distance < config_.min_distance) {
        return {0.0, 0.0};
    }
    
    // 4. 経路からの横方向距離に基づく補正
    auto cross_track_error = compute_cross_track_error(position, planned_path, closest_point_idx);
    
    // 5. 経路追従力の計算
    // 前進成分（経路に沿った方向）
    auto forward_force_vec = safe_normalize(direction_vector);
    std::array<double, 2> forward_force = {config_.k_path_attraction * forward_force_vec[0],
                                          config_.k_path_attraction * forward_force_vec[1]};
    
    // 横方向補正成分（経路に戻る方向）
    std::array<double, 2> lateral_correction = {-0.5 * config_.k_path_attraction * cross_track_error[0],
                                                -0.5 * config_.k_path_attraction * cross_track_error[1]};
    
    return {forward_force[0] + lateral_correction[0],
            forward_force[1] + lateral_correction[1]};
}

std::array<double, 2> TVVFGenerator::compute_adaptive_goal_force(const Position& position,
                                                                const Goal& goal,
                                                                const Path& planned_path) {
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

size_t TVVFGenerator::find_closest_path_point(const Position& position, const Path& planned_path) {
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

size_t TVVFGenerator::find_lookahead_point(const Position& position, const Path& planned_path, size_t start_idx) {
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
                                                              size_t closest_idx) {
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
                                                                  const Goal& /* goal */) {
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

std::array<double, 2> TVVFGenerator::clip_magnitude(const std::array<double, 2>& vector, double max_magnitude) {
    double magnitude = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (magnitude > max_magnitude) {
        double scale = max_magnitude / magnitude;
        return {vector[0] * scale, vector[1] * scale};
    }
    return vector;
}

std::array<double, 2> TVVFGenerator::safe_normalize(const std::array<double, 2>& vector, double min_norm) {
    double norm = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (norm < min_norm) {
        return {0.0, 0.0};
    }
    return {vector[0] / norm, vector[1] / norm};
}

std::array<double, 2> TVVFGenerator::safe_normalize_with_default(const std::array<double, 2>& vector,
                                                                double min_norm,
                                                                const std::array<double, 2>& default_value) {
    double norm = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
    if (norm < min_norm) {
        return default_value;
    }
    return {vector[0] / norm, vector[1] / norm};
}

} // namespace tvvf_vo_c