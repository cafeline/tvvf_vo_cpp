#include "tvvf_vo_c/utils/time_utils.hpp"
#include <iomanip>
#include <sstream>
#include <iostream>
#include <map>
#include <algorithm>
#include <numeric>

namespace tvvf_vo_c {
namespace time_utils {

double get_current_time() {
    auto now = Clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

TimePoint get_current_time_point() {
    return Clock::now();
}

double get_elapsed_time(const TimePoint& start, const TimePoint& end) {
    auto duration = end - start;
    return std::chrono::duration<double>(duration).count();
}

double get_elapsed_time_since(const TimePoint& start) {
    return get_elapsed_time(start, Clock::now());
}

std::string format_duration(double seconds) {
    std::ostringstream oss;
    
    if (seconds < 1e-6) {
        oss << std::fixed << std::setprecision(2) << (seconds * 1e9) << " ns";
    } else if (seconds < 1e-3) {
        oss << std::fixed << std::setprecision(2) << (seconds * 1e6) << " μs";
    } else if (seconds < 1.0) {
        oss << std::fixed << std::setprecision(2) << (seconds * 1e3) << " ms";
    } else if (seconds < 60.0) {
        oss << std::fixed << std::setprecision(2) << seconds << " s";
    } else if (seconds < 3600.0) {
        int minutes = static_cast<int>(seconds / 60);
        double remaining_seconds = seconds - minutes * 60;
        oss << minutes << "m " << std::fixed << std::setprecision(1) << remaining_seconds << "s";
    } else {
        int hours = static_cast<int>(seconds / 3600);
        int minutes = static_cast<int>((seconds - hours * 3600) / 60);
        double remaining_seconds = seconds - hours * 3600 - minutes * 60;
        oss << hours << "h " << minutes << "m " << std::fixed << std::setprecision(1) << remaining_seconds << "s";
    }
    
    return oss.str();
}

std::string get_timestamp_string() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count();
    
    return oss.str();
}

// Timer class implementation
Timer::Timer() : running_(false) {}

void Timer::start() {
    start_time_ = Clock::now();
    running_ = true;
}

double Timer::stop() {
    if (!running_) {
        return 0.0;
    }
    
    auto end_time = Clock::now();
    double elapsed = get_elapsed_time(start_time_, end_time);
    running_ = false;
    
    return elapsed;
}

void Timer::reset() {
    running_ = false;
}

double Timer::elapsed() const {
    if (!running_) {
        return 0.0;
    }
    
    return get_elapsed_time_since(start_time_);
}

bool Timer::is_running() const {
    return running_;
}

// PerformanceTimer class implementation
PerformanceTimer::PerformanceTimer() {}

void PerformanceTimer::start(const std::string& label) {
    auto& data = measurements_[label];
    data.start_time = Clock::now();
    data.is_measuring = true;
}

double PerformanceTimer::end(const std::string& label) {
    auto end_time = Clock::now();
    
    auto it = measurements_.find(label);
    if (it == measurements_.end() || !it->second.is_measuring) {
        return 0.0;
    }
    
    double duration_ms = get_elapsed_time(it->second.start_time, end_time) * 1000.0;
    it->second.durations_ms.push_back(duration_ms);
    it->second.is_measuring = false;
    
    return duration_ms;
}

void PerformanceTimer::clear() {
    measurements_.clear();
}

std::string PerformanceTimer::get_summary() const {
    std::ostringstream oss;
    oss << "Performance Timer Summary:\n";
    oss << "==========================\n";
    
    for (const auto& [label, data] : measurements_) {
        if (data.durations_ms.empty()) continue;
        
        auto stats = get_statistics(label);
        oss << label << ":\n";
        oss << "  Count: " << stats.count << "\n";
        oss << "  Mean:  " << std::fixed << std::setprecision(3) << stats.mean_ms << " ms\n";
        oss << "  Min:   " << std::fixed << std::setprecision(3) << stats.min_ms << " ms\n";
        oss << "  Max:   " << std::fixed << std::setprecision(3) << stats.max_ms << " ms\n";
        oss << "\n";
    }
    
    return oss.str();
}

PerformanceTimer::Statistics PerformanceTimer::get_statistics(const std::string& label) const {
    Statistics stats = {0.0, 0.0, 0.0, 0};
    
    auto it = measurements_.find(label);
    if (it == measurements_.end() || it->second.durations_ms.empty()) {
        return stats;
    }
    
    const auto& durations = it->second.durations_ms;
    stats.count = durations.size();
    
    stats.mean_ms = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
    stats.min_ms = *std::min_element(durations.begin(), durations.end());
    stats.max_ms = *std::max_element(durations.begin(), durations.end());
    
    return stats;
}

// RateLimiter class implementation
RateLimiter::RateLimiter(double max_rate_hz) 
    : interval_seconds_(1.0 / max_rate_hz), first_execution_(true) {}

bool RateLimiter::can_execute() {
    auto now = Clock::now();
    
    if (first_execution_) {
        last_execution_ = now;
        first_execution_ = false;
        return true;
    }
    
    double elapsed = get_elapsed_time(last_execution_, now);
    if (elapsed >= interval_seconds_) {
        last_execution_ = now;
        return true;
    }
    
    return false;
}

double RateLimiter::time_until_next() const {
    if (first_execution_) {
        return 0.0;
    }
    
    auto now = Clock::now();
    double elapsed = get_elapsed_time(last_execution_, now);
    return std::max(0.0, interval_seconds_ - elapsed);
}

void RateLimiter::set_rate(double new_rate_hz) {
    interval_seconds_ = 1.0 / new_rate_hz;
}

// ScopedTimer class implementation
ScopedTimer::ScopedTimer(const std::string& name, bool auto_print) 
    : name_(name), auto_print_(auto_print) {
    timer_.start();
}

ScopedTimer::~ScopedTimer() {
    double elapsed = timer_.stop() * 1000.0; // ms
    
    if (auto_print_) {
        std::cout << "[" << name_ << "] " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms" << std::endl;
    }
}

double ScopedTimer::elapsed_ms() const {
    return timer_.elapsed() * 1000.0;
}

} // namespace time_utils
} // namespace tvvf_vo_c