#include "tvvf_vo_c/core/velocity_selector.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace tvvf_vo_c {

FeasibleVelocitySelector::FeasibleVelocitySelector(const TVVFVOConfig& config) 
    : config_(config), vo_calc_(config) {}

Velocity FeasibleVelocitySelector::select_feasible_velocity(const std::array<double, 2>& tvvf_vector,
                                                           const std::vector<VOCone>& vo_cones,
                                                           const RobotState& robot_state) {
    // VO制約を無視してTVVFベクトルを直接使用（最大速度制限のみ適用）
    double tvvf_magnitude = vector_magnitude(tvvf_vector);
    
    if (tvvf_magnitude < 1e-8) {
        return Velocity(0.0, 0.0);
    }
    
    // 最大速度制限を適用
    double limited_magnitude = std::min(tvvf_magnitude, robot_state.max_velocity);
    auto normalized_tvvf = normalize_vector(tvvf_vector);
    
    return Velocity(normalized_tvvf[0] * limited_magnitude, 
                   normalized_tvvf[1] * limited_magnitude);
}

std::vector<std::array<double, 2>> FeasibleVelocitySelector::generate_candidate_velocities(
    const std::array<double, 2>& tvvf_vector,
    const std::vector<VOCone>& vo_cones,
    const RobotState& robot_state) {
    
    std::vector<std::array<double, 2>> candidates;
    
    // TVVF方向の速度
    double tvvf_magnitude = vector_magnitude(tvvf_vector);
    if (tvvf_magnitude > 0) {
        auto tvvf_direction = normalize_vector(tvvf_vector);
        std::vector<double> speed_ratios = {0.3, 0.6, 0.8, 1.0};
        
        for (double speed_ratio : speed_ratios) {
            double speed = std::min(robot_state.max_velocity * speed_ratio, tvvf_magnitude);
            std::array<double, 2> candidate = {tvvf_direction[0] * speed, tvvf_direction[1] * speed};
            
            if (is_velocity_feasible(candidate, vo_cones)) {
                candidates.push_back(candidate);
            }
        }
    }
    
    // 基本方向の速度
    std::vector<std::array<double, 2>> basic_directions = {
        {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0},
        {0.707, 0.707}, {-0.707, 0.707}, {0.707, -0.707}, {-0.707, -0.707}
    };
    
    for (const auto& direction : basic_directions) {
        std::vector<double> speed_ratios = {0.5, 1.0};
        for (double speed_ratio : speed_ratios) {
            double speed = robot_state.max_velocity * speed_ratio;
            std::array<double, 2> candidate = {direction[0] * speed, direction[1] * speed};
            
            if (vector_magnitude(candidate) <= robot_state.max_velocity &&
                is_velocity_feasible(candidate, vo_cones)) {
                candidates.push_back(candidate);
            }
        }
    }
    
    // 停止速度
    std::array<double, 2> stop_velocity = {0.0, 0.0};
    if (is_velocity_feasible(stop_velocity, vo_cones)) {
        candidates.push_back(stop_velocity);
    }
    
    return candidates;
}

bool FeasibleVelocitySelector::is_velocity_feasible(const std::array<double, 2>& velocity,
                                                   const std::vector<VOCone>& vo_cones) {
    for (const auto& vo_cone : vo_cones) {
        if (vo_calc_.is_velocity_in_vo(velocity, vo_cone)) {
            return false;
        }
    }
    return true;
}

std::array<double, 2> FeasibleVelocitySelector::select_optimal_velocity(
    const std::vector<std::array<double, 2>>& candidates,
    const std::array<double, 2>& tvvf_vector,
    const std::vector<VOCone>& vo_cones,
    const RobotState& robot_state) {
    
    std::array<double, 2> best_velocity = candidates[0];
    double best_score = std::numeric_limits<double>::lowest();
    
    double tvvf_magnitude = vector_magnitude(tvvf_vector);
    auto tvvf_direction = tvvf_magnitude > 0 ? normalize_vector(tvvf_vector) : std::array<double, 2>{0.0, 0.0};
    
    for (const auto& candidate : candidates) {
        double score = compute_velocity_score(candidate, tvvf_direction, tvvf_magnitude, vo_cones, robot_state);
        
        if (score > best_score) {
            best_score = score;
            best_velocity = candidate;
        }
    }
    
    return best_velocity;
}

double FeasibleVelocitySelector::compute_velocity_score(const std::array<double, 2>& velocity,
                                                       const std::array<double, 2>& tvvf_direction,
                                                       double tvvf_magnitude,
                                                       const std::vector<VOCone>& vo_cones,
                                                       const RobotState& robot_state) {
    double score = 0.0;
    
    // TVVF方向類似性
    double vel_magnitude = vector_magnitude(velocity);
    if (vel_magnitude > 0 && tvvf_magnitude > 0) {
        auto vel_direction = normalize_vector(velocity);
        double direction_similarity = vel_direction[0] * tvvf_direction[0] + vel_direction[1] * tvvf_direction[1];
        score += config_.direction_weight * direction_similarity;
    }
    
    // VO安全マージン
    double safety_score = compute_safety_score(velocity, vo_cones);
    score += config_.safety_weight * safety_score;
    
    // エネルギー効率
    double efficiency_score = 1.0 - (vel_magnitude / robot_state.max_velocity);
    score += config_.efficiency_weight * efficiency_score;
    
    // 速度連続性
    std::array<double, 2> current_vel = {robot_state.velocity.vx, robot_state.velocity.vy};
    std::array<double, 2> velocity_change = {velocity[0] - current_vel[0], velocity[1] - current_vel[1]};
    double velocity_change_magnitude = vector_magnitude(velocity_change);
    double continuity_score = 1.0 / (1.0 + velocity_change_magnitude);
    score += 0.5 * continuity_score;
    
    return score;
}

double FeasibleVelocitySelector::compute_safety_score(const std::array<double, 2>& velocity,
                                                     const std::vector<VOCone>& vo_cones) {
    if (vo_cones.empty()) {
        return 1.0;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& vo_cone : vo_cones) {
        if (vo_cone.type == VOCone::CONE) {
            std::array<double, 2> vertex = {vo_cone.cone_vertex.vx, vo_cone.cone_vertex.vy};
            std::array<double, 2> rel_vel = {velocity[0] - vertex[0], velocity[1] - vertex[1]};
            
            std::array<double, 2> left_tangent = {vo_cone.tangent_left.vx, vo_cone.tangent_left.vy};
            std::array<double, 2> right_tangent = {vo_cone.tangent_right.vx, vo_cone.tangent_right.vy};
            
            double left_distance = std::abs(cross_product_2d(left_tangent, rel_vel));
            double right_distance = std::abs(cross_product_2d(right_tangent, rel_vel));
            double boundary_distance = std::min(left_distance, right_distance);
            
            min_distance = std::min(min_distance, boundary_distance);
        }
    }
    
    return std::min(1.0, min_distance / 0.5);
}

double FeasibleVelocitySelector::vector_magnitude(const std::array<double, 2>& vector) {
    return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

std::array<double, 2> FeasibleVelocitySelector::normalize_vector(const std::array<double, 2>& vector) {
    double magnitude = vector_magnitude(vector);
    if (magnitude < 1e-8) {
        return {0.0, 0.0};
    }
    return {vector[0] / magnitude, vector[1] / magnitude};
}

double FeasibleVelocitySelector::cross_product_2d(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return a[0] * b[1] - a[1] * b[0];
}

} // namespace tvvf_vo_c