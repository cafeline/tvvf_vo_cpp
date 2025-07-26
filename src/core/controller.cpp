#include "tvvf_vo_c/core/controller.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace tvvf_vo_c {

TVVFVOController::TVVFVOController(const TVVFVOConfig& config)
    : config_(config),
      tvvf_generator_(config),
      vo_calculator_(config),
      velocity_selector_(config) {
    
    // 統計情報の初期化
    stats_["computation_time"] = 0.0;
    stats_["tvvf_time"] = 0.0;
    stats_["vo_time"] = 0.0;
    stats_["selection_time"] = 0.0;
    stats_["num_vo_cones"] = 0.0;
    stats_["num_candidates"] = 0.0;
    stats_["safety_margin"] = 0.0;
}

ControlOutput TVVFVOController::update(const RobotState& robot_state,
                                      const std::vector<DynamicObstacle>& obstacles,
                                      const Goal& goal,
                                      const std::optional<Path>& planned_path) {
    double start_time = get_current_time();
    
    try {
        // A*経路の存在チェック
        if (!planned_path.has_value() || planned_path->empty()) {
            // 経路がない場合は停止コマンドを返す
            return ControlOutput(
                Velocity(0.0, 0.0),
                0.0,
                0.01,
                0.0
            );
        }
        
        // TVVF計算（A*経路統合版）
        double tvvf_start = get_current_time();
        auto tvvf_vector = tvvf_generator_.compute_vector(
            robot_state.position, start_time, goal, obstacles, planned_path
        );
        stats_["tvvf_time"] = (get_current_time() - tvvf_start) * 1000;
        
        // VO制約計算
        double vo_start = get_current_time();
        auto vo_cones = vo_calculator_.compute_vo_set(
            robot_state, obstacles, config_.time_horizon
        );
        stats_["vo_time"] = (get_current_time() - vo_start) * 1000;
        stats_["num_vo_cones"] = static_cast<double>(vo_cones.size());
        
        // 実行可能速度選択
        double selection_start = get_current_time();
        Velocity selected_velocity = velocity_selector_.select_feasible_velocity(
            tvvf_vector, vo_cones, robot_state
        );
        stats_["selection_time"] = (get_current_time() - selection_start) * 1000;
        
        // 安全性評価
        double safety_margin = compute_safety_margin(robot_state, obstacles);
        stats_["safety_margin"] = safety_margin;
        
        // 制御コマンド生成
        ControlOutput control_output = generate_control_output(
            selected_velocity, robot_state, safety_margin
        );
        
        stats_["computation_time"] = (get_current_time() - start_time) * 1000;
        
        return control_output;
        
    } catch (const std::exception& e) {
        std::cerr << "制御計算エラー: " << e.what() << std::endl;
        return ControlOutput(
            Velocity(0.0, 0.0),
            0.0,
            config_.max_computation_time,
            0.0
        );
    }
}

std::unordered_map<std::string, double> TVVFVOController::get_stats() const {
    return stats_;
}

double TVVFVOController::compute_safety_margin(const RobotState& robot_state,
                                              const std::vector<DynamicObstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& obstacle : obstacles) {
        double distance = robot_state.position.distance_to(obstacle.position);
        double effective_distance = distance - robot_state.radius - obstacle.radius;
        min_distance = std::min(min_distance, effective_distance);
    }
    
    return std::max(0.0, min_distance);
}

ControlOutput TVVFVOController::generate_control_output(const Velocity& selected_velocity,
                                                       const RobotState& robot_state,
                                                       double safety_margin) {
    double target_angle = std::atan2(selected_velocity.vy, selected_velocity.vx);
    double angle_diff = target_angle - robot_state.orientation;
    
    // 角度差の正規化
    angle_diff = normalize_angle(angle_diff);
    
    double angular_velocity = angle_diff * 2.0;
    
    return ControlOutput(
        selected_velocity,
        angular_velocity,
        config_.max_computation_time,
        safety_margin
    );
}

double TVVFVOController::normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double TVVFVOController::get_current_time() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

} // namespace tvvf_vo_c