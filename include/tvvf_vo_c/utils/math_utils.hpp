#ifndef TVVF_VO_C_UTILS_MATH_UTILS_HPP_
#define TVVF_VO_C_UTILS_MATH_UTILS_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

namespace tvvf_vo_c {
namespace math_utils {

/**
 * @brief 数学定数
 */
constexpr double PI = M_PI;
constexpr double TWO_PI = 2.0 * M_PI;
constexpr double HALF_PI = M_PI / 2.0;
constexpr double EPSILON = 1e-9;

/**
 * @brief 角度を-π～πの範囲に正規化
 * @param angle 角度（ラジアン）
 * @return 正規化された角度
 */
double normalize_angle(double angle);

/**
 * @brief 2つの角度の差を計算（-π～πの範囲）
 * @param angle1 角度1（ラジアン）
 * @param angle2 角度2（ラジアン）
 * @return 角度差
 */
double angle_difference(double angle1, double angle2);

/**
 * @brief 2点間の距離を計算
 * @param x1, y1 点1の座標
 * @param x2, y2 点2の座標
 * @return 距離
 */
double distance(double x1, double y1, double x2, double y2);

/**
 * @brief 2点間の角度を計算
 * @param x1, y1 点1の座標
 * @param x2, y2 点2の座標
 * @return 角度（ラジアン）
 */
double angle_between_points(double x1, double y1, double x2, double y2);

/**
 * @brief 値を指定範囲にクランプ
 * @param value 値
 * @param min_val 最小値
 * @param max_val 最大値
 * @return クランプされた値
 */
template<typename T>
T clamp(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(max_val, value));
}

/**
 * @brief 線形補間
 * @param a 開始値
 * @param b 終了値
 * @param t 補間パラメータ（0.0-1.0）
 * @return 補間された値
 */
double lerp(double a, double b, double t);

/**
 * @brief ガウシアン関数
 * @param x 入力値
 * @param mu 平均
 * @param sigma 標準偏差
 * @return ガウシアン値
 */
double gaussian(double x, double mu = 0.0, double sigma = 1.0);

/**
 * @brief シグモイド関数
 * @param x 入力値
 * @param k 傾き係数
 * @return シグモイド値（0-1）
 */
double sigmoid(double x, double k = 1.0);

/**
 * @brief 2次元ベクトルの内積
 * @param ax, ay ベクトルA
 * @param bx, by ベクトルB
 * @return 内積
 */
double dot_product(double ax, double ay, double bx, double by);

/**
 * @brief 2次元ベクトルの外積（スカラー値）
 * @param ax, ay ベクトルA
 * @param bx, by ベクトルB
 * @return 外積のz成分
 */
double cross_product(double ax, double ay, double bx, double by);

/**
 * @brief ベクトルの正規化
 * @param x, y ベクトル成分
 * @return 正規化されたベクトル（std::pair<double, double>）
 */
std::pair<double, double> normalize_vector(double x, double y);

/**
 * @brief 点から線分までの最短距離
 * @param px, py 点の座標
 * @param x1, y1 線分の始点
 * @param x2, y2 線分の終点
 * @return 最短距離
 */
double point_to_line_distance(double px, double py, 
                             double x1, double y1, 
                             double x2, double y2);

/**
 * @brief 点が多角形内部にあるかチェック（Ray Casting Algorithm）
 * @param px, py 点の座標
 * @param polygon_x, polygon_y 多角形の頂点座標リスト
 * @return 内部にある場合true
 */
bool point_in_polygon(double px, double py,
                     const std::vector<double>& polygon_x,
                     const std::vector<double>& polygon_y);

/**
 * @brief 2つの線分の交点を計算
 * @param x1, y1 線分1の始点
 * @param x2, y2 線分1の終点
 * @param x3, y3 線分2の始点
 * @param x4, y4 線分2の終点
 * @param intersection_x, intersection_y 交点座標（出力）
 * @return 交点が存在する場合true
 */
bool line_intersection(double x1, double y1, double x2, double y2,
                      double x3, double y3, double x4, double y4,
                      double& intersection_x, double& intersection_y);

/**
 * @brief 統計計算用クラス
 */
class Statistics {
public:
    void add_sample(double value);
    void clear();
    
    double mean() const;
    double variance() const;
    double standard_deviation() const;
    double min() const;
    double max() const;
    size_t count() const;

private:
    std::vector<double> samples_;
    double sum_ = 0.0;
    double sum_squared_ = 0.0;
    double min_val_ = std::numeric_limits<double>::max();
    double max_val_ = std::numeric_limits<double>::lowest();
};

/**
 * @brief 移動平均フィルタ
 */
class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t window_size);
    
    double update(double value);
    double get_average() const;
    void clear();

private:
    std::vector<double> buffer_;
    size_t window_size_;
    size_t current_index_;
    bool buffer_full_;
    double sum_;
};

} // namespace math_utils
} // namespace tvvf_vo_c

#endif // TVVF_VO_C_UTILS_MATH_UTILS_HPP_