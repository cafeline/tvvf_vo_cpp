// test/test_wavefront_expander.cpp

#include <gtest/gtest.h>
#include "tvvf_vo_c/core/wavefront_expander.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>

using namespace tvvf_vo_c;

class WavefrontExpanderTest : public ::testing::Test {
protected:
    WavefrontExpander expander;
    nav_msgs::msg::OccupancyGrid test_map;
    
    void SetUp() override {
        // 5x5のテストマップを作成
        test_map.info.width = 5;
        test_map.info.height = 5;
        test_map.info.resolution = 1.0;
        test_map.info.origin.position.x = 0.0;
        test_map.info.origin.position.y = 0.0;
        test_map.data.resize(25, 0);  // 全て自由空間
    }
};

// TEST 1: マップ初期化のテスト
TEST_F(WavefrontExpanderTest, InitializeFromOccupancyGrid) {
    // Given: 5x5のマップ
    // When: マップを初期化
    expander.initializeFromOccupancyGrid(test_map);
    
    // Then: フィールドサイズが正しく設定される
    auto field = expander.getField();
    EXPECT_EQ(field.width, 5);
    EXPECT_EQ(field.height, 5);
    EXPECT_DOUBLE_EQ(field.resolution, 1.0);
}

// TEST 2: 距離場計算のテスト（シンプルケース）
TEST_F(WavefrontExpanderTest, ComputeDistanceFieldSimple) {
    // Given: 障害物なしのマップとゴール位置
    expander.initializeFromOccupancyGrid(test_map);
    Position goal(2.0, 2.0);  // 中央をゴールに設定
    
    // When: 距離場を計算
    expander.computeDistanceField(goal);
    
    // Then: ゴール位置の距離が0、隣接セルの距離が正しい
    auto field = expander.getField();
    EXPECT_DOUBLE_EQ(field.grid[2][2].distance, 0.0);  // ゴール
    EXPECT_DOUBLE_EQ(field.grid[2][1].distance, 1.0);  // 左隣
    EXPECT_DOUBLE_EQ(field.grid[2][3].distance, 1.0);  // 右隣
    EXPECT_DOUBLE_EQ(field.grid[1][2].distance, 1.0);  // 上隣
    EXPECT_DOUBLE_EQ(field.grid[3][2].distance, 1.0);  // 下隣
    EXPECT_NEAR(field.grid[1][1].distance, 1.414, 0.01); // 対角
}

// TEST 3: 障害物回避のテスト
TEST_F(WavefrontExpanderTest, AvoidObstacles) {
    // Given: 中央に障害物があるマップ
    test_map.data[12] = 100;  // (2,2)を障害物に
    expander.initializeFromOccupancyGrid(test_map);
    Position goal(4.0, 4.0);  // 右下隅をゴールに
    
    // When: 距離場を計算
    expander.computeDistanceField(goal);
    
    // Then: 障害物の距離は無限大、その周りは迂回経路
    auto field = expander.getField();
    EXPECT_EQ(field.grid[2][2].distance, INFINITY);  // 障害物
    EXPECT_GT(field.grid[0][0].distance, 0);  // 左上隅から到達可能
}

// TEST 4: ベクトル場生成のテスト
TEST_F(WavefrontExpanderTest, GenerateVectorField) {
    // Given: 距離場が計算済み
    expander.initializeFromOccupancyGrid(test_map);
    Position goal(4.0, 4.0);
    expander.computeDistanceField(goal);
    
    // When: ベクトル場を生成
    expander.generateVectorField();
    
    // Then: (0,0)のベクトルが右下方向を向いている
    auto vec = expander.getVectorAt(Position(0.0, 0.0));
    EXPECT_GT(vec[0], 0);  // x成分が正（右向き）
    EXPECT_GT(vec[1], 0);  // y成分が正（下向き）
    
    // ベクトルが正規化されている
    double norm = std::sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
    EXPECT_NEAR(norm, 1.0, 0.01);
}

// TEST 5: 到達不可能な場合のテスト
TEST_F(WavefrontExpanderTest, UnreachableGoal) {
    // Given: ゴールが壁で囲まれている
    for (int i = 1; i <= 3; i++) {
        for (int j = 1; j <= 3; j++) {
            if (i != 2 || j != 2) {
                test_map.data[i * 5 + j] = 100;  // 中央以外を障害物に
            }
        }
    }
    expander.initializeFromOccupancyGrid(test_map);
    Position goal(2.0, 2.0);  // 囲まれた中央をゴールに
    
    // When: 距離場を計算
    expander.computeDistanceField(goal);
    
    // Then: 外側のセルは到達不可能（距離無限大）
    auto field = expander.getField();
    EXPECT_EQ(field.grid[0][0].distance, INFINITY);
    EXPECT_EQ(field.grid[4][4].distance, INFINITY);
}

// メイン関数
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}