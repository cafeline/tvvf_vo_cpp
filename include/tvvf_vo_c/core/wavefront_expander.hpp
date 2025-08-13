// include/tvvf_vo_c/core/wavefront_expander.hpp

#ifndef TVVF_VO_C_CORE_WAVEFRONT_EXPANDER_HPP_
#define TVVF_VO_C_CORE_WAVEFRONT_EXPANDER_HPP_

#include "tvvf_vo_c/core/field_types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <vector>
#include <array>

namespace tvvf_vo_c {

class WavefrontExpander {
private:
    VectorField field_;
    std::queue<GridCell*> wave_queue_;
    
public:
    WavefrontExpander();
    
    // マップを初期化
    void initializeFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map);
    
    // ゴールからの距離場を計算
    void computeDistanceField(const Position& goal);
    
    // 距離場からベクトル場を生成
    void generateVectorField();
    
    // 指定位置のベクトルを取得
    std::array<double, 2> getVectorAt(const Position& pos) const;
    
    // フィールドを取得（テスト用）
    const VectorField& getField() const { return field_; }
    
private:
    // 8近傍の隣接セルを取得
    std::vector<GridCell*> getNeighbors(int x, int y);
    
    // BFS波面展開の実装
    void expandWavefront();
    
    // ヘルパー関数
    void resetDistanceField();
    bool isValidGridPosition(int x, int y) const;
    std::array<double, 2> computeCellVector(int x, int y) const;
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_WAVEFRONT_EXPANDER_HPP_