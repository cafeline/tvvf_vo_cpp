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
    
    // 線形補間によるブレンド
    std::array<double, 2> blended = {
        (1.0 - blend_weight) * static_vector[0] + blend_weight * dynamic_vector[0],
        (1.0 - blend_weight) * static_vector[1] + blend_weight * dynamic_vector[1]
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

}  // namespace tvvf_vo_c