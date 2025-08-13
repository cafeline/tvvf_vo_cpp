// src/core/global_field_generator.cpp

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
    
    // ブレンディングパラメータ
    constexpr double DEFAULT_BLEND_WEIGHT = 0.5;      // デフォルトのブレンド比率
    constexpr double MIN_MAGNITUDE_THRESHOLD = 0.01;  // ベクトル大きさの最小閾値
    
    // パフォーマンス最適化
    constexpr double INFLUENCE_RADIUS_SQ = INFLUENCE_RADIUS * INFLUENCE_RADIUS;
    constexpr double MIN_DISTANCE_SQ = MIN_DISTANCE * MIN_DISTANCE;
}

// コンストラクタ
GlobalFieldGenerator::GlobalFieldGenerator() 
    : last_computation_time_(0.0), static_field_computed_(false) {
    wavefront_expander_ = std::make_unique<WavefrontExpander>();
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

// 動的障害物を考慮したフィールド生成
VectorField GlobalFieldGenerator::generateField(const std::vector<DynamicObstacle>& obstacles) {
    auto start = std::chrono::high_resolution_clock::now();
    
    if (!static_field_computed_) {
        // 静的場が計算されていない場合は空のフィールドを返す
        return VectorField();
    }
    
    // 障害物がない場合は静的場をそのまま返す（最適化）
    if (obstacles.empty()) {
        auto end = std::chrono::high_resolution_clock::now();
        last_computation_time_ = std::chrono::duration<double>(end - start).count();
        return static_field_;
    }
    
    // 動的障害物の影響をブレンド
    VectorField result = blendWithDynamicObstacles(static_field_, obstacles);
    
    auto end = std::chrono::high_resolution_clock::now();
    last_computation_time_ = std::chrono::duration<double>(end - start).count();
    
    return result;
}

// 動的障害物の影響をブレンド
VectorField GlobalFieldGenerator::blendWithDynamicObstacles(const VectorField& static_field,
                                                            const std::vector<DynamicObstacle>& obstacles) {
    // フィールドをコピー
    VectorField blended_field = static_field;
    
    // 各グリッドセルに対して動的障害物の影響を計算
    for (int y = 0; y < blended_field.height; ++y) {
        for (int x = 0; x < blended_field.width; ++x) {
            Position cell_pos = blended_field.gridToWorld(x, y);
            
            // このセルに対する全障害物からの斥力を計算
            auto total_repulsive_force = computeTotalRepulsiveForce(cell_pos, obstacles);
            
            // 斥力が有意な場合のみブレンド
            if (hasSignificantMagnitude(total_repulsive_force)) {
                blended_field.vectors[y][x] = blendAndNormalizeVectors(
                    static_field.vectors[y][x],
                    total_repulsive_force,
                    DEFAULT_BLEND_WEIGHT
                );
            }
        }
    }
    
    return blended_field;
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

// 単一障害物からの斥力を計算
std::array<double, 2> GlobalFieldGenerator::computeRepulsiveForceFromObstacle(
    const Position& position,
    const DynamicObstacle& obstacle) const {
    
    // 障害物への相対位置
    double dx = position.x - obstacle.position.x;
    double dy = position.y - obstacle.position.y;
    double distance_sq = dx * dx + dy * dy;
    
    // 影響範囲外の早期リターン（平方根計算を避ける）
    if (distance_sq > INFLUENCE_RADIUS_SQ || distance_sq < MIN_DISTANCE_SQ) {
        return {0.0, 0.0};
    }
    
    double distance = std::sqrt(distance_sq);
    
    // 斥力の大きさ（Coulomb型ポテンシャル）
    double force_magnitude = REPULSIVE_GAIN * (1.0 / distance - 1.0 / INFLUENCE_RADIUS);
    
    // 正規化された方向ベクトル × 力の大きさ
    return {
        force_magnitude * (dx / distance),
        force_magnitude * (dy / distance)
    };
}

// ベクトルのブレンドと正規化
std::array<double, 2> GlobalFieldGenerator::blendAndNormalizeVectors(
    const std::array<double, 2>& static_vector,
    const std::array<double, 2>& dynamic_vector,
    double blend_weight) const {
    
    // 加算的なブレンド（斥力を直接加算）
    std::array<double, 2> blended = {
        static_vector[0] + blend_weight * dynamic_vector[0],
        static_vector[1] + blend_weight * dynamic_vector[1]
    };
    
    // 正規化
    return normalizeVector(blended);
}

// ベクトルの正規化
std::array<double, 2> GlobalFieldGenerator::normalizeVector(
    const std::array<double, 2>& vector) const {
    
    double magnitude_sq = vector[0] * vector[0] + vector[1] * vector[1];
    
    // ゼロベクトルのチェック
    if (magnitude_sq < MIN_MAGNITUDE_THRESHOLD * MIN_MAGNITUDE_THRESHOLD) {
        return {0.0, 0.0};
    }
    
    double inv_magnitude = 1.0 / std::sqrt(magnitude_sq);
    return {
        vector[0] * inv_magnitude,
        vector[1] * inv_magnitude
    };
}

// ベクトルが有意な大きさを持つかチェック
bool GlobalFieldGenerator::hasSignificantMagnitude(
    const std::array<double, 2>& vector) const {
    
    return std::abs(vector[0]) > MIN_MAGNITUDE_THRESHOLD || 
           std::abs(vector[1]) > MIN_MAGNITUDE_THRESHOLD;
}

// ロボット位置での速度ベクトル取得
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
    
    // 静的場のベクトル
    std::array<double, 2> result_vector = static_field_.vectors[grid_y][grid_x];
    
    // NaNチェック
    if (std::isnan(result_vector[0]) || std::isnan(result_vector[1])) {
        // 静的場自体がNaNの場合はゼロベクトルを返す
        result_vector = {0.0, 0.0};
    }
    
    // マップ端（占有領域）からの斥力
    // ただし、静的場がNaNの場合は斥力のみを使用
    if (std::isnan(result_vector[0]) || std::isnan(result_vector[1])) {
        auto map_repulsion = computeMapBoundaryRepulsion(position);
        if (!std::isnan(map_repulsion[0]) && !std::isnan(map_repulsion[1])) {
            // 静的場がNaNなので斥力のみを正規化して返す
            double mag = std::sqrt(map_repulsion[0]*map_repulsion[0] + 
                                 map_repulsion[1]*map_repulsion[1]);
            if (mag > 0.01) {
                result_vector = {map_repulsion[0]/mag, map_repulsion[1]/mag};
            } else {
                // ゴールへの単純な引力
                double dx = goal_position_.x - position.x;
                double dy = goal_position_.y - position.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > 0.01) {
                    result_vector = {dx/dist, dy/dist};
                } else {
                    result_vector = {0.0, 0.0};
                }
            }
        }
    } else {
        // 通常の斥力統合
        auto map_repulsion = computeMapBoundaryRepulsion(position);
        if (hasSignificantMagnitude(map_repulsion)) {
            if (!std::isnan(map_repulsion[0]) && !std::isnan(map_repulsion[1])) {
                // 斥力の強さに基づいて動的にブレンド比率を決定
                double repulsion_magnitude = std::sqrt(map_repulsion[0]*map_repulsion[0] + 
                                                      map_repulsion[1]*map_repulsion[1]);
                // 斥力が強いほどブレンド比率を上げる（最大0.9）
                double dynamic_blend_weight = std::min(0.9, repulsion_magnitude * map_repulsion_gain_ / 10.0);
                
                result_vector = blendAndNormalizeVectors(
                    result_vector,
                    map_repulsion,
                    dynamic_blend_weight
                );
            }
        }
    }
    
    // 動的障害物からの斥力
    if (!obstacles.empty()) {
        auto repulsive_force = computeTotalRepulsiveForce(position, obstacles);
        
        // 斥力が有意な場合のみブレンド
        if (hasSignificantMagnitude(repulsive_force)) {
            result_vector = blendAndNormalizeVectors(
                result_vector,
                repulsive_force,
                DEFAULT_BLEND_WEIGHT
            );
        }
    }
    
    return result_vector;
}

// マップ端からの斥力計算
std::array<double, 2> GlobalFieldGenerator::computeMapBoundaryRepulsion(
    const Position& position) const {
    
    // 最近傍障害物までの距離を取得
    double distance_to_obstacle = getDistanceToNearestObstacle(position);
    
    // デバッグ: 距離を出力
    // std::cout << "Distance to obstacle at (" << position.x << ", " << position.y << "): " << distance_to_obstacle << std::endl;
    
    // 影響範囲外なら斥力なし
    if (distance_to_obstacle > map_repulsion_range_) {
        return {0.0, 0.0};
    }
    
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
    int obstacle_count = 0;
    
    for (int dy = -SEARCH_RADIUS; dy <= SEARCH_RADIUS; ++dy) {
        for (int dx = -SEARCH_RADIUS; dx <= SEARCH_RADIUS; ++dx) {
            if (dx == 0 && dy == 0) continue;  // 自分自身はスキップ
            
            int nx = grid_x + dx;
            int ny = grid_y + dy;
            
            bool is_obstacle = false;
            
            // 範囲外も障害物として扱う
            if (nx < 0 || nx >= static_field_.width || 
                ny < 0 || ny >= static_field_.height) {
                is_obstacle = true;
            } else if (static_field_.grid[ny][nx].is_obstacle) {
                is_obstacle = true;
            }
            
            // 障害物の場合
            if (is_obstacle) {
                // 障害物からの方向ベクトル（障害物から遠ざかる）
                double dist = std::sqrt(dx * dx + dy * dy) * static_field_.resolution;
                if (dist < MIN_DISTANCE) dist = MIN_DISTANCE;
                
                if (dist <= map_repulsion_range_) {
                    // 距離に反比例した斥力
                    double weight = 1.0 / dist;
                    // 障害物から遠ざかる方向（-dx, -dyの方向）
                    repulsion_x -= (double)dx * weight;
                    repulsion_y -= (double)dy * weight;
                    obstacle_count++;
                }
            }
        }
    }
    
    // 障害物がない場合
    if (obstacle_count == 0) {
        return {0.0, 0.0};
    }
    
    // 正規化
    double mag = std::sqrt(repulsion_x * repulsion_x + repulsion_y * repulsion_y);
    if (mag < MIN_MAGNITUDE_THRESHOLD) {
        return {0.0, 0.0};
    }
    
    // 線形斥力（距離に反比例）
    double repulsion_strength = map_repulsion_gain_ * (1.0 - distance_to_obstacle / map_repulsion_range_);
    
    return {
        (repulsion_x / mag) * repulsion_strength,
        (repulsion_y / mag) * repulsion_strength
    };
}

// 最近傍障害物までの距離を取得
double GlobalFieldGenerator::getDistanceToNearestObstacle(
    const Position& position) const {
    
    auto [grid_x, grid_y] = static_field_.worldToGrid(position);
    
    // 範囲チェック
    if (grid_x < 0 || grid_x >= static_field_.width || 
        grid_y < 0 || grid_y >= static_field_.height) {
        return 0.0;
    }
    
    // 現在位置が障害物の場合
    if (static_field_.grid[grid_y][grid_x].is_obstacle) {
        return 0.0;
    }
    
    // 単純な距離計算（Fast Marchingの距離場が正しく初期化されていない可能性）
    // 周囲の障害物を探索して最短距離を計算
    double min_distance = map_repulsion_range_ + 1.0;
    
    for (int dy = -SEARCH_RADIUS; dy <= SEARCH_RADIUS; ++dy) {
        for (int dx = -SEARCH_RADIUS; dx <= SEARCH_RADIUS; ++dx) {
            int nx = grid_x + dx;
            int ny = grid_y + dy;
            
            if (nx < 0 || nx >= static_field_.width || 
                ny < 0 || ny >= static_field_.height) {
                // マップ外も障害物として扱う
                double dist = std::sqrt(dx * dx + dy * dy) * static_field_.resolution;
                min_distance = std::min(min_distance, dist);
                continue;
            }
            
            if (static_field_.grid[ny][nx].is_obstacle) {
                double dist = std::sqrt(dx * dx + dy * dy) * static_field_.resolution;
                min_distance = std::min(min_distance, dist);
            }
        }
    }
    
    double distance = min_distance;
    
    // 距離が無限大の場合は最大値を返す
    if (std::isinf(distance)) {
        return map_repulsion_range_ + 1.0;
    }
    
    return distance;
}

}  // namespace tvvf_vo_c