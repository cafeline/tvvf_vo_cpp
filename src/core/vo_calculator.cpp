#include "tvvf_vo_c/core/vo_calculator.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace tvvf_vo_c {

VelocityObstacleCalculator::VelocityObstacleCalculator(const TVVFVOConfig& config) : config_(config) {}

std::vector<VOCone> VelocityObstacleCalculator::compute_vo_set(const RobotState&,
                                                              const std::vector<DynamicObstacle>&,
                                                              double) {
    // VO計算を簡略化：空のリストを返す
    return std::vector<VOCone>();
}

bool VelocityObstacleCalculator::is_velocity_in_vo(const std::array<double, 2>& velocity, const VOCone& vo_cone) {
    if (vo_cone.type == VOCone::FULL_CIRCLE) {
        std::array<double, 2> relative_vel = {velocity[0] - vo_cone.center.vx,
                                             velocity[1] - vo_cone.center.vy};
        double distance = vector_magnitude(relative_vel);
        return distance <= vo_cone.radius;
    } else if (vo_cone.type == VOCone::CONE) {
        std::array<double, 2> relative_vel = {velocity[0] - vo_cone.cone_vertex.vx,
                                             velocity[1] - vo_cone.cone_vertex.vy};
        
        std::array<double, 2> tangent_left = {vo_cone.tangent_left.vx, vo_cone.tangent_left.vy};
        std::array<double, 2> tangent_right = {vo_cone.tangent_right.vx, vo_cone.tangent_right.vy};
        
        double cross_left = cross_product_2d(tangent_left, relative_vel);
        double cross_right = cross_product_2d(relative_vel, tangent_right);
        
        return cross_left >= 0 && cross_right >= 0;
    }
    return false;
}

std::vector<std::array<double, 2>> VelocityObstacleCalculator::get_vo_free_velocities(
    const RobotState& robot_state,
    const std::vector<VOCone>& vo_cones,
    double resolution) {
    
    if (resolution <= 0.0) {
        resolution = config_.vo_resolution;
    }
    
    std::vector<std::array<double, 2>> feasible_velocities;
    double max_vel = robot_state.max_velocity;
    
    // 速度空間をサンプリング
    for (double vx = -max_vel; vx <= max_vel; vx += resolution) {
        for (double vy = -max_vel; vy <= max_vel; vy += resolution) {
            std::array<double, 2> velocity = {vx, vy};
            
            // 最大速度制限チェック
            if (vector_magnitude(velocity) > max_vel) {
                continue;
            }
            
            // VO制約チェック
            bool is_feasible = true;
            for (const auto& vo_cone : vo_cones) {
                if (is_velocity_in_vo(velocity, vo_cone)) {
                    is_feasible = false;
                    break;
                }
            }
            
            if (is_feasible) {
                feasible_velocities.push_back(velocity);
            }
        }
    }
    
    return feasible_velocities;
}

std::optional<VOCone> VelocityObstacleCalculator::compute_single_vo(const RobotState& robot_state,
                                                                   const DynamicObstacle& obstacle,
                                                                   double time_horizon) {
    std::array<double, 2> rel_pos = {obstacle.position.x - robot_state.position.x,
                                     obstacle.position.y - robot_state.position.y};
    std::array<double, 2> rel_vel = {obstacle.velocity.vx, obstacle.velocity.vy};
    double rel_distance = vector_magnitude(rel_pos);
    
    double max_relative_speed = robot_state.max_velocity + vector_magnitude(rel_vel);
    double max_influence_distance = max_relative_speed * time_horizon;
    
    if (rel_distance > max_influence_distance) {
        return std::nullopt;
    }
    
    double expanded_radius = obstacle.radius + robot_state.radius + config_.safety_margin;
    Velocity cone_vertex(rel_vel[0], rel_vel[1]);
    
    VOCone vo_cone;
    vo_cone.obstacle_id = obstacle.id;
    vo_cone.cone_vertex = cone_vertex;
    
    // 障害物が非常に近い場合は全円
    if (rel_distance <= expanded_radius) {
        vo_cone.type = VOCone::FULL_CIRCLE;
        vo_cone.center = cone_vertex;
        vo_cone.radius = robot_state.max_velocity;
        return vo_cone;
    }
    
    // 円錐VO計算
    vo_cone.type = VOCone::CONE;
    
    double sin_theta = expanded_radius / rel_distance;
    double cos_theta = std::sqrt(1.0 - sin_theta * sin_theta);
    
    auto rel_pos_norm = normalize_vector(rel_pos);
    
    // 接線ベクトル計算
    std::array<double, 2> tangent_right = {
        cos_theta * rel_pos_norm[0] - sin_theta * rel_pos_norm[1],
        sin_theta * rel_pos_norm[0] + cos_theta * rel_pos_norm[1]
    };
    
    std::array<double, 2> tangent_left = {
        cos_theta * rel_pos_norm[0] + sin_theta * rel_pos_norm[1],
        -sin_theta * rel_pos_norm[0] + cos_theta * rel_pos_norm[1]
    };
    
    vo_cone.tangent_left = Velocity(tangent_left[0], tangent_left[1]);
    vo_cone.tangent_right = Velocity(tangent_right[0], tangent_right[1]);
    
    // 円錐の境界点
    std::array<double, 2> cone_right = {cone_vertex.vx + tangent_right[0] * robot_state.max_velocity,
                                       cone_vertex.vy + tangent_right[1] * robot_state.max_velocity};
    std::array<double, 2> cone_left = {cone_vertex.vx + tangent_left[0] * robot_state.max_velocity,
                                      cone_vertex.vy + tangent_left[1] * robot_state.max_velocity};
    
    vo_cone.cone_left = Velocity(cone_left[0], cone_left[1]);
    vo_cone.cone_right = Velocity(cone_right[0], cone_right[1]);
    
    return vo_cone;
}

double VelocityObstacleCalculator::vector_magnitude(const std::array<double, 2>& vector) {
    return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1]);
}

std::array<double, 2> VelocityObstacleCalculator::normalize_vector(const std::array<double, 2>& vector) {
    double magnitude = vector_magnitude(vector);
    if (magnitude < 1e-8) {
        return {0.0, 0.0};
    }
    return {vector[0] / magnitude, vector[1] / magnitude};
}

double VelocityObstacleCalculator::cross_product_2d(const std::array<double, 2>& a, const std::array<double, 2>& b) {
    return a[0] * b[1] - a[1] * b[0];
}

} // namespace tvvf_vo_c