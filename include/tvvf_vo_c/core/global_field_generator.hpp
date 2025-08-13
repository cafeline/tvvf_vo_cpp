// include/tvvf_vo_c/core/global_field_generator.hpp

#ifndef TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_
#define TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_

#include "tvvf_vo_c/core/field_types.hpp"
#include "tvvf_vo_c/core/wavefront_expander.hpp"
#include "tvvf_vo_c/core/fast_marching.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <vector>

namespace tvvf_vo_c {

class GlobalFieldGenerator {
public:
    GlobalFieldGenerator();
    ~GlobalFieldGenerator() = default;
    
    // 静的ベクトル場の事前計算
    void precomputeStaticField(const nav_msgs::msg::OccupancyGrid& map,
                                const Position& goal);
    
    // 動的障害物を考慮したベクトル場生成
    VectorField generateField(const std::vector<DynamicObstacle>& obstacles);
    
    // 動的障害物の影響を計算
    VectorField blendWithDynamicObstacles(const VectorField& static_field,
                                           const std::vector<DynamicObstacle>& obstacles);
    
    // パフォーマンス計測用
    double getLastComputationTime() const;
    
    // ロボット位置での速度ベクトル取得
    std::array<double, 2> getVelocityAt(const Position& position,
                                         const std::vector<DynamicObstacle>& obstacles);
    
    // 静的場が計算済みかチェック
    bool isStaticFieldReady() const { return static_field_computed_; }
    
private:
    std::unique_ptr<WavefrontExpander> wavefront_expander_;
    std::unique_ptr<FastMarching> fast_marching_;
    VectorField static_field_;
    double last_computation_time_;
    bool static_field_computed_;
    
    // ヘルパー関数
    std::array<double, 2> computeTotalRepulsiveForce(
        const Position& position,
        const std::vector<DynamicObstacle>& obstacles) const;
    
    std::array<double, 2> computeRepulsiveForceFromObstacle(
        const Position& position,
        const DynamicObstacle& obstacle) const;
    
    std::array<double, 2> blendAndNormalizeVectors(
        const std::array<double, 2>& static_vector,
        const std::array<double, 2>& dynamic_vector,
        double blend_weight) const;
    
    std::array<double, 2> normalizeVector(
        const std::array<double, 2>& vector) const;
    
    bool hasSignificantMagnitude(
        const std::array<double, 2>& vector) const;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_GLOBAL_FIELD_GENERATOR_HPP_