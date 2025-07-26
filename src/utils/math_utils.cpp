#include "tvvf_vo_c/utils/math_utils.hpp"
#include <cmath>
#include <numeric>

namespace tvvf_vo_c {
namespace math_utils {

double normalize_angle(double angle) {
    while (angle > PI) {
        angle -= TWO_PI;
    }
    while (angle < -PI) {
        angle += TWO_PI;
    }
    return angle;
}

double angle_difference(double angle1, double angle2) {
    return normalize_angle(angle1 - angle2);
}

double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double angle_between_points(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

double gaussian(double x, double mu, double sigma) {
    double diff = x - mu;
    return std::exp(-(diff * diff) / (2.0 * sigma * sigma)) / (sigma * std::sqrt(TWO_PI));
}

double sigmoid(double x, double k) {
    return 1.0 / (1.0 + std::exp(-k * x));
}

double dot_product(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

double cross_product(double ax, double ay, double bx, double by) {
    return ax * by - ay * bx;
}

std::pair<double, double> normalize_vector(double x, double y) {
    double magnitude = std::sqrt(x * x + y * y);
    if (magnitude < EPSILON) {
        return {0.0, 0.0};
    }
    return {x / magnitude, y / magnitude};
}

double point_to_line_distance(double px, double py, 
                             double x1, double y1, 
                             double x2, double y2) {
    double line_length_squared = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    
    if (line_length_squared < EPSILON) {
        // 線分が点の場合
        return distance(px, py, x1, y1);
    }
    
    // 点から線分への投影の位置パラメータを計算
    double t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_length_squared;
    
    if (t < 0.0) {
        // 線分の始点側への投影
        return distance(px, py, x1, y1);
    } else if (t > 1.0) {
        // 線分の終点側への投影
        return distance(px, py, x2, y2);
    } else {
        // 線分上への投影
        double projection_x = x1 + t * (x2 - x1);
        double projection_y = y1 + t * (y2 - y1);
        return distance(px, py, projection_x, projection_y);
    }
}

bool point_in_polygon(double px, double py,
                     const std::vector<double>& polygon_x,
                     const std::vector<double>& polygon_y) {
    if (polygon_x.size() != polygon_y.size() || polygon_x.size() < 3) {
        return false;
    }
    
    bool inside = false;
    size_t n = polygon_x.size();
    
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon_y[i] > py) != (polygon_y[j] > py)) &&
            (px < (polygon_x[j] - polygon_x[i]) * (py - polygon_y[i]) / 
             (polygon_y[j] - polygon_y[i]) + polygon_x[i])) {
            inside = !inside;
        }
    }
    
    return inside;
}

bool line_intersection(double x1, double y1, double x2, double y2,
                      double x3, double y3, double x4, double y4,
                      double& intersection_x, double& intersection_y) {
    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    
    if (std::abs(denom) < EPSILON) {
        // 線分が平行
        return false;
    }
    
    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        intersection_x = x1 + t * (x2 - x1);
        intersection_y = y1 + t * (y2 - y1);
        return true;
    }
    
    return false;
}

// Statistics class implementation
void Statistics::add_sample(double value) {
    samples_.push_back(value);
    sum_ += value;
    sum_squared_ += value * value;
    min_val_ = std::min(min_val_, value);
    max_val_ = std::max(max_val_, value);
}

void Statistics::clear() {
    samples_.clear();
    sum_ = 0.0;
    sum_squared_ = 0.0;
    min_val_ = std::numeric_limits<double>::max();
    max_val_ = std::numeric_limits<double>::lowest();
}

double Statistics::mean() const {
    if (samples_.empty()) return 0.0;
    return sum_ / samples_.size();
}

double Statistics::variance() const {
    if (samples_.size() < 2) return 0.0;
    double m = mean();
    return (sum_squared_ - samples_.size() * m * m) / (samples_.size() - 1);
}

double Statistics::standard_deviation() const {
    return std::sqrt(variance());
}

double Statistics::min() const {
    return samples_.empty() ? 0.0 : min_val_;
}

double Statistics::max() const {
    return samples_.empty() ? 0.0 : max_val_;
}

size_t Statistics::count() const {
    return samples_.size();
}

// MovingAverageFilter class implementation
MovingAverageFilter::MovingAverageFilter(size_t window_size) 
    : buffer_(window_size, 0.0), window_size_(window_size), 
      current_index_(0), buffer_full_(false), sum_(0.0) {
}

double MovingAverageFilter::update(double value) {
    if (buffer_full_) {
        sum_ -= buffer_[current_index_];
    }
    
    buffer_[current_index_] = value;
    sum_ += value;
    
    current_index_ = (current_index_ + 1) % window_size_;
    
    if (!buffer_full_ && current_index_ == 0) {
        buffer_full_ = true;
    }
    
    return get_average();
}

double MovingAverageFilter::get_average() const {
    if (!buffer_full_ && current_index_ == 0) {
        return 0.0;
    }
    
    size_t count = buffer_full_ ? window_size_ : current_index_;
    return sum_ / count;
}

void MovingAverageFilter::clear() {
    std::fill(buffer_.begin(), buffer_.end(), 0.0);
    current_index_ = 0;
    buffer_full_ = false;
    sum_ = 0.0;
}

} // namespace math_utils
} // namespace tvvf_vo_c