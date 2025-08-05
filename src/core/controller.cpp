#include "tvvf_vo_c/core/controller.hpp"
#include "tvvf_vo_c/utils/time_utils.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace tvvf_vo_c {

TVVFVOController::TVVFVOController(const TVVFVOConfig& config)
    : config_(config),
      tvvf_generator_(config) {
    
    // 統計情報の初期化
    stats_["computation_time"] = 0.0;
    stats_["tvvf_time"] = 0.0;
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
        std::array<double, 2> tvvf_vector;
        {
            PROFILE_MODULE(internal_profiler_, "TVVF_Generation");
            double tvvf_start = get_current_time();
            tvvf_vector = tvvf_generator_.compute_vector(
                robot_state.position, start_time, goal, obstacles, planned_path
            );
            stats_["tvvf_time"] = (get_current_time() - tvvf_start) * 1000;
        }
        
        // TVVFベクトルをそのまま速度として出力（差動駆動変換はノード側で実行）
        Velocity selected_velocity;
        {
            // TVVFベクトルをロボットの速度制限に合わせて正規化
            double magnitude = std::sqrt(tvvf_vector[0] * tvvf_vector[0] + tvvf_vector[1] * tvvf_vector[1]);
            
            if (magnitude > 0.001) {
                // 最大速度で制限
                if (magnitude > config_.max_linear_velocity) {
                    tvvf_vector[0] = (tvvf_vector[0] / magnitude) * config_.max_linear_velocity;
                    tvvf_vector[1] = (tvvf_vector[1] / magnitude) * config_.max_linear_velocity;
                }
                
                // TVVFベクトルをそのまま2D速度として設定
                selected_velocity = Velocity(tvvf_vector[0], tvvf_vector[1]);
            } else {
                selected_velocity = Velocity(0.0, 0.0);
            }
        }
        
        // 制御コマンド生成
        double total_time = (get_current_time() - start_time) * 1000;
        stats_["computation_time"] = total_time;
        
        return ControlOutput(
            selected_velocity, 
            total_time,
            0.01,  // 固定のdt
            1.0    // 固定の信頼度
        );
        
    } catch (const std::exception& e) {
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

const TVVFGenerator& TVVFVOController::get_tvvf_generator() const {
    return tvvf_generator_;
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