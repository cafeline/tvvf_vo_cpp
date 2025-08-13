// src/core/wavefront_expander.cpp

#include "tvvf_vo_c/core/wavefront_expander.hpp"
#include <cmath>
#include <limits>

namespace tvvf_vo_c {

// 定数定義
namespace {
    constexpr double ORTHOGONAL_COST = 1.0;     // 直交移動のコスト
    constexpr double DIAGONAL_COST = 1.414;     // 対角移動のコスト（√2）
    constexpr int8_t OBSTACLE_THRESHOLD = 50;   // 障害物判定の閾値
    constexpr int8_t UNKNOWN_CELL = -1;         // 不明なセルの値
    
    // 8近傍の方向ベクトル
    constexpr int DX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    constexpr int DY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    constexpr double COSTS[8] = {
        DIAGONAL_COST, ORTHOGONAL_COST, DIAGONAL_COST,
        ORTHOGONAL_COST, ORTHOGONAL_COST,
        DIAGONAL_COST, ORTHOGONAL_COST, DIAGONAL_COST
    };
}

WavefrontExpander::WavefrontExpander() {}

void WavefrontExpander::initializeFromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map) {
    // フィールドメタデータの設定
    field_.width = map.info.width;
    field_.height = map.info.height;
    field_.resolution = map.info.resolution;
    field_.origin = Position(map.info.origin.position.x, 
                            map.info.origin.position.y);
    
    // グリッドとベクトル場のメモリ確保
    field_.grid.resize(field_.height);
    field_.vectors.resize(field_.height);
    
    // 各セルの初期化
    for (int y = 0; y < field_.height; ++y) {
        field_.grid[y].resize(field_.width);
        field_.vectors[y].resize(field_.width, {0.0, 0.0});
        
        for (int x = 0; x < field_.width; ++x) {
            const int index = y * field_.width + x;
            GridCell& cell = field_.grid[y][x];
            
            // グリッド座標を設定
            cell.x = x;
            cell.y = y;
            
            // 障害物判定（占有率ベース）
            const int8_t occupancy = map.data[index];
            cell.is_obstacle = (occupancy > OBSTACLE_THRESHOLD || occupancy == UNKNOWN_CELL);
            cell.distance = std::numeric_limits<double>::infinity();
            cell.is_visited = false;
        }
    }
}

void WavefrontExpander::computeDistanceField(const Position& goal) {
    // キューをクリア（前回の計算結果を削除）
    while (!wave_queue_.empty()) {
        wave_queue_.pop();
    }
    
    // 全セルの距離と訪問フラグをリセット
    resetDistanceField();
    
    // ゴール位置をグリッド座標に変換
    const auto [gx, gy] = field_.worldToGrid(goal);
    
    // 境界チェックとゴールセルの初期化
    if (isValidGridPosition(gx, gy)) {
        GridCell& goal_cell = field_.grid[gy][gx];
        if (!goal_cell.is_obstacle) {
            goal_cell.distance = 0.0;
            goal_cell.is_visited = true;
            wave_queue_.push(&goal_cell);
            
            // BFS波面展開を実行
            expandWavefront();
        }
    }
}

void WavefrontExpander::resetDistanceField() {
    for (auto& row : field_.grid) {
        for (auto& cell : row) {
            if (!cell.is_obstacle) {
                cell.distance = std::numeric_limits<double>::infinity();
                cell.is_visited = false;
            }
        }
    }
}

bool WavefrontExpander::isValidGridPosition(int x, int y) const {
    return x >= 0 && x < field_.width && y >= 0 && y < field_.height;
}

void WavefrontExpander::generateVectorField() {
    for (int y = 0; y < field_.height; ++y) {
        for (int x = 0; x < field_.width; ++x) {
            field_.vectors[y][x] = computeCellVector(x, y);
        }
    }
}

std::array<double, 2> WavefrontExpander::computeCellVector(int x, int y) const {
    const GridCell& cell = field_.grid[y][x];
    
    // 障害物または到達不可能なセルはゼロベクトル
    if (cell.is_obstacle || std::isinf(cell.distance)) {
        return {0.0, 0.0};
    }
    
    // 最小距離の隣接セルを探索
    double min_distance = cell.distance;
    int best_dx = 0;
    int best_dy = 0;
    
    // 8近傍を探索
    for (int i = 0; i < 8; ++i) {
        const int nx = x + DX[i];
        const int ny = y + DY[i];
        
        if (!isValidGridPosition(nx, ny)) continue;
        
        const GridCell& neighbor = field_.grid[ny][nx];
        if (neighbor.is_obstacle) continue;
        
        if (neighbor.distance < min_distance) {
            min_distance = neighbor.distance;
            best_dx = DX[i];
            best_dy = DY[i];
        }
    }
    
    // ベクトル正規化
    const double norm = std::sqrt(best_dx * best_dx + best_dy * best_dy);
    if (norm > 0.0) {
        return {best_dx / norm, best_dy / norm};
    }
    
    return {0.0, 0.0};
}

std::array<double, 2> WavefrontExpander::getVectorAt(const Position& pos) const {
    return field_.getVector(pos);
}

std::vector<GridCell*> WavefrontExpander::getNeighbors(int x, int y) {
    std::vector<GridCell*> neighbors;
    
    // 8近傍をチェック
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < field_.width && 
                ny >= 0 && ny < field_.height) {
                neighbors.push_back(&field_.grid[ny][nx]);
            }
        }
    }
    
    return neighbors;
}

void WavefrontExpander::expandWavefront() {
    while (!wave_queue_.empty()) {
        GridCell* current = wave_queue_.front();
        wave_queue_.pop();
        
        // 8近傍を探索
        for (int i = 0; i < 8; ++i) {
            const int nx = current->x + DX[i];
            const int ny = current->y + DY[i];
            
            // 境界チェック
            if (!isValidGridPosition(nx, ny)) continue;
            
            GridCell& neighbor = field_.grid[ny][nx];
            
            // 障害物または訪問済みはスキップ
            if (neighbor.is_obstacle || neighbor.is_visited) continue;
            
            // 距離更新
            neighbor.distance = current->distance + COSTS[i];
            neighbor.is_visited = true;
            wave_queue_.push(&neighbor);
        }
    }
}

}  // namespace tvvf_vo_c