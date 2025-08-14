// src/core/global_field_generator.cpp
// 高速化版

#include "tvvf_vo_c/core/global_field_generator.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace tvvf_vo_c {

// 定数定義
namespace {
    // 斥力計算パラメータ
    constexpr double REPULSIVE_GAIN = 1.0;            // 斥力ゲイン
    constexpr double INFLUENCE_RADIUS = 3.0;          // 影響半径[m]
    constexpr double MIN_DISTANCE = 0.1;              // 最小距離[m]（ゼロ除算防止）
    
    constexpr int SEARCH_RADIUS = 10;                 // 障害物検索半径[グリッド]
    
    constexpr double MIN_MAGNITUDE_THRESHOLD = 0.01;  // ベクトル大きさの最小閾値
    
    // パフォーマンス最適化
    constexpr double INFLUENCE_RADIUS_SQ = INFLUENCE_RADIUS * INFLUENCE_RADIUS;
    constexpr double MIN_DISTANCE_SQ = MIN_DISTANCE * MIN_DISTANCE;
    constexpr double MIN_MAGNITUDE_THRESHOLD_SQ = MIN_MAGNITUDE_THRESHOLD * MIN_MAGNITUDE_THRESHOLD;
}

// コンストラクタ
GlobalFieldGenerator::GlobalFieldGenerator() 
    : last_computation_time_(0.0), static_field_computed_(false) {
    fast_marching_ = std::make_unique<FastMarching>();
}

// 静的場の事前計算
void GlobalFieldGenerator::precomputeStaticField(const nav_msgs::msg::OccupancyGrid& map,
                                                  const Position& goal) {
    // ゴール位置を保存
    goal_position_ = goal;
    
    // Fast Marching Methodを使用して静的場を計算
    fast_marching_->initializeFromOccupancyGrid(map);
    fast_marching_->computeDistanceField(goal);
    fast_marching_->generateVectorField();
    
    // 静的場を保存
    static_field_ = fast_marching_->getField();
    static_field_computed_ = true;
}

// 計算時間の取得
double GlobalFieldGenerator::getLastComputationTime() const {
    return last_computation_time_;
}

// Private helper functions

// 全障害物からの斥力の合計を計算
std::array<double, 2> GlobalFieldGenerator::computeTotalRepulsiveForce(
    const Position& position,
    const std::vector<DynamicObstacle>& obstacles) const {
    
    std::array<double, 2> total_force = {0.0, 0.0};
    
    for (const auto& obstacle : obstacles) {
        auto force = computeRepulsiveForceFromObstacle(position, obstacle);
        total_force[0] += force[0];
        total_force[1] += force[1];
    }
    
    return total_force;
}

// 単一障害物からの斥力を計算（高速化版）
std::array<double, 2> GlobalFieldGenerator::computeRepulsiveForceFromObstacle(
    const Position& position,
    const DynamicObstacle& obstacle) const {
    
    // 障害物への相対位置
    double dx = position.x - obstacle.position.x;
    double dy = position.y - obstacle.position.y;
    double distance_sq = dx * dx + dy * dy;
    
    // 影響範囲外の早期リターン（sqrtを避ける）
    if (distance_sq > INFLUENCE_RADIUS_SQ || distance_sq < MIN_DISTANCE_SQ) {
        return {0.0, 0.0};
    }
    
    // 距離の逆数を直接計算（高速逆平方根の近似）
    double inv_distance = 1.0 / std::sqrt(distance_sq);
    
    // 斥力の大きさ
    double force_magnitude = REPULSIVE_GAIN * (inv_distance - 1.0 / INFLUENCE_RADIUS);
    
    // 正規化された方向ベクトル × 力の大きさ
    return {
        force_magnitude * dx * inv_distance,
        force_magnitude * dy * inv_distance
    };
}

// ベクトルが有意な大きさを持つかチェック（高速化版）
bool GlobalFieldGenerator::hasSignificantMagnitude(
    const std::array<double, 2>& vector) const {
    
    // 2乗で比較（sqrtを避ける）
    return (vector[0] * vector[0] + vector[1] * vector[1]) > MIN_MAGNITUDE_THRESHOLD_SQ;
}

// ロボット位置での速度ベクトル取得（高速化版）
std::array<double, 2> GlobalFieldGenerator::getVelocityAt(const Position& position,
                                                          const std::vector<DynamicObstacle>& obstacles) {
    if (!static_field_computed_) {
        return {0.0, 0.0};
    }
    
    // 静的場からベクトル取得
    auto [grid_x, grid_y] = static_field_.worldToGrid(position);
    
    // 範囲チェック
    if (grid_x < 0 || grid_x >= static_field_.width || 
        grid_y < 0 || grid_y >= static_field_.height) {
        return {0.0, 0.0};
    }
    
    // 引力成分（静的場または直接計算）
    std::array<double, 2> attraction_force = static_field_.vectors[grid_y][grid_x];
    
    // 静的場がNaNの場合は直接ゴールへの引力を計算
    if (std::isnan(attraction_force[0]) || std::isnan(attraction_force[1])) {
        double dx_goal = goal_position_.x - position.x;
        double dy_goal = goal_position_.y - position.y;
        double dist_sq = dx_goal*dx_goal + dy_goal*dy_goal;
        
        if (dist_sq > 0.0001) {
            double inv_dist = 1.0 / std::sqrt(dist_sq);
            attraction_force = {dx_goal * inv_dist, dy_goal * inv_dist};
        } else {
            attraction_force = {0.0, 0.0};
        }
    }
    
    // マップ端からの斥力（高速化版）
    auto map_repulsion = computeMapBoundaryRepulsionFast(position);
    
    // 動的障害物からの斥力
    std::array<double, 2> dynamic_repulsion = {0.0, 0.0};
    if (!obstacles.empty()) {
        dynamic_repulsion = computeTotalRepulsiveForce(position, obstacles);
    }
    
    // 全ての力を合成
    std::array<double, 2> total_force = {
        attraction_force[0] + map_repulsion[0] + dynamic_repulsion[0],
        attraction_force[1] + map_repulsion[1] + dynamic_repulsion[1]
    };
    
    // 正規化（高速版）
    double mag_sq = total_force[0]*total_force[0] + total_force[1]*total_force[1];
    if (mag_sq > MIN_MAGNITUDE_THRESHOLD_SQ) {
        double inv_mag = 1.0 / std::sqrt(mag_sq);
        return {total_force[0] * inv_mag, total_force[1] * inv_mag};
    }
    
    return {0.0, 0.0};
}

// マップ端からの斥力計算（高速化版）
std::array<double, 2> GlobalFieldGenerator::computeMapBoundaryRepulsionFast(
    const Position& position) const {
    
    // 現在のグリッド位置
    auto [grid_x, grid_y] = static_field_.worldToGrid(position);
    
    // 範囲チェック
    if (grid_x < 0 || grid_x >= static_field_.width || 
        grid_y < 0 || grid_y >= static_field_.height) {
        return {0.0, 0.0};
    }
    
    // 周囲の障害物を探索して斥力方向を計算
    double repulsion_x = 0.0;
    double repulsion_y = 0.0;
    double max_range_sq = (map_repulsion_range_ / static_field_.resolution);
    max_range_sq *= max_range_sq;
    
    bool has_obstacle = false;
    
    // 検索範囲を動的に調整（近い障害物を優先）
    for (int radius = 1; radius <= SEARCH_RADIUS; ++radius) {
        bool found_at_radius = false;
        
        // 各半径での境界を探索
        for (int i = -radius; i <= radius; ++i) {
            // 上下の境界
            int dx = i;
            for (int dy : {-radius, radius}) {
                if (dx*dx + dy*dy > max_range_sq) continue;
                
                int nx = grid_x + dx;
                int ny = grid_y + dy;
                
                if (nx < 0 || nx >= static_field_.width || 
                    ny < 0 || ny >= static_field_.height || 
                    static_field_.grid[ny][nx].is_obstacle) {
                    
                    // 距離の2乗に反比例した重み
                    double dist_sq = dx*dx + dy*dy;
                    if (dist_sq < 1.0) dist_sq = 1.0;
                    double weight = 1.0 / dist_sq;
                    
                    repulsion_x -= dx * weight;
                    repulsion_y -= dy * weight;
                    has_obstacle = true;
                    found_at_radius = true;
                }
            }
            
            // 左右の境界（角は除く）
            if (i != -radius && i != radius) {
                int dy = i;
                for (int dx : {-radius, radius}) {
                    if (dx*dx + dy*dy > max_range_sq) continue;
                    
                    int nx = grid_x + dx;
                    int ny = grid_y + dy;
                    
                    if (nx < 0 || nx >= static_field_.width || 
                        ny < 0 || ny >= static_field_.height || 
                        static_field_.grid[ny][nx].is_obstacle) {
                        
                        double dist_sq = dx*dx + dy*dy;
                        if (dist_sq < 1.0) dist_sq = 1.0;
                        double weight = 1.0 / dist_sq;
                        
                        repulsion_x -= dx * weight;
                        repulsion_y -= dy * weight;
                        has_obstacle = true;
                        found_at_radius = true;
                    }
                }
            }
        }
        
        // 近い距離で十分な障害物が見つかったら終了
        if (found_at_radius && radius > 3) break;
    }
    
    if (!has_obstacle) {
        return {0.0, 0.0};
    }
    
    // 正規化（高速版）
    double mag_sq = repulsion_x * repulsion_x + repulsion_y * repulsion_y;
    if (mag_sq < MIN_MAGNITUDE_THRESHOLD_SQ) {
        return {0.0, 0.0};
    }
    
    double inv_mag = 1.0 / std::sqrt(mag_sq);
    double strength = map_repulsion_gain_;
    
    return {
        repulsion_x * inv_mag * strength,
        repulsion_y * inv_mag * strength
    };
}

// 互換性のための既存関数（使用しない）
std::array<double, 2> GlobalFieldGenerator::computeMapBoundaryRepulsion(
    const Position& position) const {
    return computeMapBoundaryRepulsionFast(position);
}

double GlobalFieldGenerator::getDistanceToNearestObstacle(
    const Position& position) const {
    // この関数は高速化版では使用しない
    return 0.0;
}

}  // namespace tvvf_vo_c