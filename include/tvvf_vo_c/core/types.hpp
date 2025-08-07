#ifndef TVVF_VO_C_CORE_TYPES_HPP_
#define TVVF_VO_C_CORE_TYPES_HPP_

#include <vector>
#include <optional>
#include <cmath>
#include <memory>

namespace tvvf_vo_c {

/**
 * @brief 2D位置クラス
 */
struct Position {
    double x, y;
    
    Position() : x(0.0), y(0.0) {}
    Position(double x, double y) : x(x), y(y) {}
    
    double distance_to(const Position& other) const;
    Position operator+(const Position& other) const;
    Position operator-(const Position& other) const;
    Position operator*(double scalar) const;
    bool operator==(const Position& other) const;
};

/**
 * @brief 2D速度クラス
 */
struct Velocity {
    double vx, vy;
    
    Velocity() : vx(0.0), vy(0.0) {}
    Velocity(double vx, double vy) : vx(vx), vy(vy) {}
    
    double magnitude() const;
    Velocity normalize() const;
    Velocity operator+(const Velocity& other) const;
    Velocity operator-(const Velocity& other) const;
    Velocity operator*(double scalar) const;
};

/**
 * @brief 2D力ベクトルクラス
 */
struct Force {
    double fx, fy;
    
    Force() : fx(0.0), fy(0.0) {}
    Force(double fx, double fy) : fx(fx), fy(fy) {}
    
    double magnitude() const;
    Force normalize() const;
    Force operator+(const Force& other) const;
    Force operator-(const Force& other) const;
    Force operator*(double scalar) const;
};

/**
 * @brief ロボット状態クラス
 */
struct RobotState {
    Position position;
    Velocity velocity;
    double orientation;          // [rad]
    double max_velocity;         // [m/s]
    double max_acceleration;     // [m/s²]
    double radius;               // [m]
    
    RobotState() 
        : orientation(0.0), max_velocity(2.0), max_acceleration(1.0), radius(0.3) {}
    
    RobotState(const Position& pos, const Velocity& vel, double orient,
               double max_vel = 2.0, double max_acc = 1.0, double r = 0.3)
        : position(pos), velocity(vel), orientation(orient), 
          max_velocity(max_vel), max_acceleration(max_acc), radius(r) {}
};

/**
 * @brief 動的障害物クラス
 */
struct DynamicObstacle {
    int id;
    Position position;
    Velocity velocity;
    double radius;
    double prediction_horizon;   // [s]
    double uncertainty;          // [m]
    
    DynamicObstacle() 
        : id(0), radius(0.1), prediction_horizon(3.0), uncertainty(0.1) {}
    
    DynamicObstacle(int id, const Position& pos, const Velocity& vel, double r,
                     double pred_horizon = 3.0, double uncert = 0.1)
        : id(id), position(pos), velocity(vel), radius(r), 
          prediction_horizon(pred_horizon), uncertainty(uncert) {}
    
    Position predict_position(double time_delta) const;
};

/**
 * @brief 目標クラス
 */
struct Goal {
    Position position;
    double tolerance;            // [m]
    
    Goal() : tolerance(0.1) {}
    Goal(const Position& pos, double tol = 0.1) : position(pos), tolerance(tol) {}
};

/**
 * @brief 経路上の点クラス
 */
struct PathPoint {
    Position position;
    double cost;
    
    PathPoint() : cost(0.0) {}
    PathPoint(const Position& pos, double c = 0.0) : position(pos), cost(c) {}
};

/**
 * @brief 経路クラス
 */
class Path {
public:
    std::vector<PathPoint> points;
    double total_cost;
    
    Path() : total_cost(0.0) {}
    
    void add_point(const Position& position, double cost = 0.0);
    void clear();
    bool empty() const;
    size_t size() const;
    const PathPoint& operator[](size_t index) const;
    PathPoint& operator[](size_t index);
};

/**
 * @brief 制御出力クラス
 */
struct ControlOutput {
    Velocity velocity_command;
    double angular_velocity;     // [rad/s]
    double execution_time;       // [s]
    double safety_margin;        // [m]
    
    ControlOutput() 
        : angular_velocity(0.0), execution_time(0.05), safety_margin(0.0) {}
    
    ControlOutput(const Velocity& vel_cmd, double ang_vel = 0.0, 
                  double exec_time = 0.05, double safety = 0.0)
        : velocity_command(vel_cmd), angular_velocity(ang_vel), 
          execution_time(exec_time), safety_margin(safety) {}
};

/**
 * @brief TVVF-VO統合システム設定
 */
struct TVVFVOConfig {
    // TVVF関連
    double k_attraction;
    double k_repulsion;
    double influence_radius;
    
    // 経路統合関連（ベクトル場統合用）
    double k_path_attraction;
    double path_influence_radius;
    double lookahead_distance;
    
    // 障害物回避関連
    double safety_margin;
    
    // ロボット仕様パラメータ
    double max_linear_velocity;
    double max_angular_velocity;
    
    // 流体ベクトル場専用パラメータ
    double fluid_strength_factor;     // 流体効果の強度調整係数
    double repulsive_weight;          // 斥力成分の重み
    double fluid_weight;              // 流体成分の重み
    double path_direction_weight;     // 経路方向成分の重み
    
    // 明確な距離区分パラメータ [m]
    double near_distance_threshold;   // 近距離の上限 [m]
    double mid_distance_threshold;    // 中距離の上限 [m]
    
    // 距離別重み調整パラメータ
    double near_repulsive_weight;     // 近距離での斥力重み
    double near_fluid_weight;         // 近距離での流体重み
    double near_path_weight;          // 近距離での経路重み
    double mid_repulsive_weight;      // 中距離での斥力重み
    double mid_fluid_weight;          // 中距離での流体重み
    double mid_path_weight;           // 中距離での経路重み
    
    // 指数的斥力パラメータ（新機能）
    bool enable_exponential_repulsion;    // 指数的斥力の有効化
    double exponential_base;              // 指数関数の底
    double exponential_scale_factor;      // 指数関数のスケーリング係数
    double max_exponential_distance;      // 指数関数を適用する最大距離
    double exponential_smoothing_threshold; // 指数関数の滑らか化閾値
    
    // 数値安定性
    double min_distance;
    
    // 性能関連
    double max_computation_time;
    
    TVVFVOConfig() 
        : k_attraction(1.0), k_repulsion(2.0), influence_radius(3.0),
          k_path_attraction(2.0), path_influence_radius(2.0), lookahead_distance(1.5),
          safety_margin(0.2), max_linear_velocity(2.0), max_angular_velocity(2.0),
          fluid_strength_factor(1.0),
          repulsive_weight(0.4), fluid_weight(0.6), path_direction_weight(1.0),
          near_distance_threshold(0.5), mid_distance_threshold(1.5),
          near_repulsive_weight(0.7), near_fluid_weight(0.2), near_path_weight(0.1),
          mid_repulsive_weight(0.4), mid_fluid_weight(0.4), mid_path_weight(0.2),
          enable_exponential_repulsion(false), exponential_base(2.0), exponential_scale_factor(1.5),
          max_exponential_distance(1.5), exponential_smoothing_threshold(0.1),
          min_distance(1e-6), max_computation_time(0.05) {}
};


/**
 * @brief A*アルゴリズム用ノードクラス
 */
struct AStarNode {
    std::pair<int, int> position;  // (x, y) グリッド座標
    double g_cost;                 // スタートからのコスト
    double h_cost;                 // ゴールまでのヒューリスティックコスト
    double f_cost;                 // 総コスト
    std::shared_ptr<AStarNode> parent;
    
    AStarNode() : position({0, 0}), g_cost(0.0), h_cost(0.0), f_cost(0.0) {}
    
    AStarNode(const std::pair<int, int>& pos, double g = 0.0, double h = 0.0,
              std::shared_ptr<AStarNode> p = nullptr)
        : position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
    
    // Python版のheapqと同じ動作になるよう修正
    bool operator<(const AStarNode& other) const {
        // f_costで比較（小さい値が優先）
        if (std::abs(f_cost - other.f_cost) < 1e-6) {
            // f_costが同じ場合はh_costで比較（小さい値が優先）
            return h_cost < other.h_cost;
        }
        return f_cost < other.f_cost;
    }
    
    // ヒープ用の比較（大きい値を優先するため逆転）
    bool operator>(const AStarNode& other) const {
        if (std::abs(f_cost - other.f_cost) < 1e-6) {
            return h_cost > other.h_cost;
        }
        return f_cost > other.f_cost;
    }
    
    bool operator==(const AStarNode& other) const {
        return position == other.position;
    }
};

} // namespace tvvf_vo_c

#endif // TVVF_VO_C_CORE_TYPES_HPP_