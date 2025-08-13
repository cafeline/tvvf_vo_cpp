#include "tvvf_vo_c/core/controller.hpp"
#include "tvvf_vo_c/utils/time_utils.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace tvvf_vo_c {

TVVFVOController::TVVFVOController(const TVVFVOConfig& config)
    : config_(config) {
    
    // 統計情報の初期化
    stats_["computation_time"] = 0.0;
}

ControlOutput TVVFVOController::update(const RobotState& robot_state,
                                      const std::vector<DynamicObstacle>& obstacles,
                                      const Goal& goal,
                                      const std::optional<Path>& planned_path) {
    // この関数は互換性のために残すが、使用されない
    // GlobalFieldGeneratorが実際の制御を行う
    return ControlOutput(
        Velocity(0.0, 0.0),
        0.0,
        0.01,
        0.0
    );
}

std::unordered_map<std::string, double> TVVFVOController::get_stats() const {
    return stats_;
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