// include/tvvf_vo_c/core/field_types.hpp

#ifndef TVVF_VO_C_CORE_FIELD_TYPES_HPP_
#define TVVF_VO_C_CORE_FIELD_TYPES_HPP_

#include <vector>
#include <array>
#include <limits>
#include "tvvf_vo_c/core/types.hpp"  // Position, Velocity, DynamicObstacleをtypes.hppから使用

namespace tvvf_vo_c {

// グリッドマップ上の単一セル
struct GridCell {
    int x, y;                    // グリッド座標
    double distance;             // ゴールまでの距離
    double potential;            // ポテンシャル値
    bool is_obstacle;            // 障害物フラグ
    bool is_visited;             // 訪問済みフラグ
    
    GridCell() : x(0), y(0), distance(std::numeric_limits<double>::infinity()), 
                 potential(0.0), is_obstacle(false), is_visited(false) {}
};

// ベクトル場全体
class VectorField {
public:
    int width, height;           // グリッドサイズ
    double resolution;           // セル解像度[m]
    Position origin;             // 原点位置
    std::vector<std::vector<GridCell>> grid;  // 2Dグリッド
    std::vector<std::vector<std::array<double, 2>>> vectors;  // ベクトル場
    
    VectorField() : width(0), height(0), resolution(1.0) {}
    
    // グリッド座標⇔ワールド座標変換
    GridCell& at(int x, int y) {
        return grid[y][x];
    }
    
    const GridCell& at(int x, int y) const {
        return grid[y][x];
    }
    
    Position gridToWorld(int x, int y) const {
        return Position(
            origin.x + x * resolution,
            origin.y + y * resolution
        );
    }
    
    std::pair<int, int> worldToGrid(const Position& pos) const {
        int x = static_cast<int>((pos.x - origin.x) / resolution);
        int y = static_cast<int>((pos.y - origin.y) / resolution);
        return {x, y};
    }
    
    std::array<double, 2> getVector(const Position& pos) const {
        auto [x, y] = worldToGrid(pos);
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return vectors[y][x];
        }
        return {0.0, 0.0};
    }
};

}  // namespace tvvf_vo_c

#endif  // TVVF_VO_C_CORE_FIELD_TYPES_HPP_