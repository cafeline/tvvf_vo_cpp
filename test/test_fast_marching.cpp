// test/test_fast_marching.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/fast_marching.hpp"
#include "tvvf_vo_c/core/wavefront_expander.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <cmath>

using namespace tvvf_vo_c;

class FastMarchingTest : public ::testing::Test {
protected:
    FastMarching fmm;
    nav_msgs::msg::OccupancyGrid test_map;
    
    void SetUp() override {
        // 10x10のテストマップ（より精密なテスト用）
        test_map.info.width = 10;
        test_map.info.height = 10;
        test_map.info.resolution = 0.5;
        test_map.info.origin.position.x = 0.0;
        test_map.info.origin.position.y = 0.0;
        test_map.data.resize(100, 0);
    }
};

// TEST 1: Eikonalソルバーの単体テスト
TEST_F(FastMarchingTest, SolveEikonalSimple) {
    // Given: 隣接セルの到達時間が既知
    fmm.initializeFromOccupancyGrid(test_map);
    
    // When: Eikonal方程式を解く
    double time = fmm.solveEikonalWithKnownNeighbors(
        1.0,  // 左隣の時間
        2.0,  // 右隣の時間
        1.5,  // 上隣の時間
        2.5   // 下隣の時間
    );
    
    // Then: 最小時間+速度場の逆数
    EXPECT_GT(time, 1.0);  // 最小隣接時間より大きい
    EXPECT_LT(time, 2.0);  // 妥当な範囲内
}

// TEST 2: 距離場の精度テスト（直線）
TEST_F(FastMarchingTest, DistanceFieldAccuracyStraightLine) {
    // Given: 障害物なしのマップ
    fmm.initializeFromOccupancyGrid(test_map);
    Position goal(0.0, 0.0);  // 左上隅をゴール
    
    // When: FMMで距離場を計算
    fmm.computeDistanceField(goal);
    
    // Then: 直線距離が正確に計算される
    auto field = fmm.getField();
    // (5,0)の点：x方向に5セル分 = 2.5m
    EXPECT_NEAR(field.grid[0][5].distance, 2.5, 0.1);
    // (0,5)の点：y方向に5セル分 = 2.5m
    EXPECT_NEAR(field.grid[5][0].distance, 2.5, 0.1);
}

// TEST 3: 距離場の精度テスト（対角線）
TEST_F(FastMarchingTest, DistanceFieldAccuracyDiagonal) {
    // Given: 障害物なしのマップ
    fmm.initializeFromOccupancyGrid(test_map);
    Position goal(0.0, 0.0);
    
    // When: FMMで距離場を計算
    fmm.computeDistanceField(goal);
    
    // Then: 対角線距離が正確（ユークリッド距離）
    auto field = fmm.getField();
    // (5,5)の点：sqrt(2.5^2 + 2.5^2) ≈ 3.536
    double expected = std::sqrt(2.5 * 2.5 + 2.5 * 2.5);
    // FMMの離散化誤差を考慮して許容誤差を0.4に設定
    EXPECT_NEAR(field.grid[5][5].distance, expected, 0.4);
}

// TEST 4: 障害物迂回の精度テスト
TEST_F(FastMarchingTest, ObstacleBypassAccuracy) {
    // Given: 垂直な壁がある
    for (int y = 2; y < 8; y++) {
        test_map.data[y * 10 + 5] = 100;  // x=5に壁
    }
    fmm.initializeFromOccupancyGrid(test_map);
    Position goal(0.0, 2.5);  // 左側中央をゴール
    
    // When: FMMで距離場を計算
    fmm.computeDistanceField(goal);
    
    // Then: 壁の反対側への最短経路が計算される
    auto field = fmm.getField();
    // (9,5)への経路：壁を迂回
    EXPECT_GT(field.grid[5][9].distance, 4.5);  // 直線より長い
    EXPECT_LT(field.grid[5][9].distance, 10.0); // 妥当な範囲
}

// TEST 5: Wavefrontとの比較テスト
TEST_F(FastMarchingTest, CompareWithWavefront) {
    // Given: 同じマップで両方のアルゴリズムを実行
    WavefrontExpander wavefront;
    wavefront.initializeFromOccupancyGrid(test_map);
    fmm.initializeFromOccupancyGrid(test_map);
    Position goal(2.5, 2.5);  // グリッド中央（5,5）に対応
    
    // When: 両方で距離場を計算
    wavefront.computeDistanceField(goal);
    fmm.computeDistanceField(goal);
    
    // Then: FMMの方が精度が高い（特に対角線）
    auto wf_field = wavefront.getField();
    auto fmm_field = fmm.getField();
    
    // 対角線上の点(0,0)で比較
    double wf_dist = wf_field.grid[0][0].distance;
    double fmm_dist = fmm_field.grid[0][0].distance;
    double true_dist = std::sqrt(2.5*2.5 + 2.5*2.5);  // ワールド座標での距離
    
    // 両方とも有効な距離を持っているか確認
    ASSERT_FALSE(std::isinf(wf_dist)) << "Wavefront distance is infinite";
    ASSERT_FALSE(std::isinf(fmm_dist)) << "FMM distance is infinite";
    
    // FMMの方が真の距離に近い
    EXPECT_LT(std::abs(fmm_dist - true_dist), 
              std::abs(wf_dist - true_dist));
}

// TEST 6: パフォーマンステスト
TEST_F(FastMarchingTest, PerformanceTest) {
    // Given: 大きめのマップ（20x20）
    nav_msgs::msg::OccupancyGrid large_map;
    large_map.info.width = 20;
    large_map.info.height = 20;
    large_map.info.resolution = 0.5;
    large_map.info.origin.position.x = 0.0;
    large_map.info.origin.position.y = 0.0;
    large_map.data.resize(400, 0);
    
    fmm.initializeFromOccupancyGrid(large_map);
    Position goal(5.0, 5.0);
    
    // When: 距離場計算の時間を測定
    auto start = std::chrono::high_resolution_clock::now();
    fmm.computeDistanceField(goal);
    auto end = std::chrono::high_resolution_clock::now();
    
    // Then: 50ms以内に完了
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_LT(duration.count(), 50);
}

// メイン関数
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}