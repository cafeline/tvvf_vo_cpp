#ifndef TVVF_VO_C_UTILS_TIME_UTILS_HPP_
#define TVVF_VO_C_UTILS_TIME_UTILS_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <map>

namespace tvvf_vo_c {
namespace time_utils {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

/**
 * @brief 現在時刻を取得
 * @return 現在時刻（秒）
 */
double get_current_time();

/**
 * @brief 現在のタイムポイントを取得
 * @return タイムポイント
 */
TimePoint get_current_time_point();

/**
 * @brief 2つのタイムポイント間の経過時間を計算
 * @param start 開始時刻
 * @param end 終了時刻
 * @return 経過時間（秒）
 */
double get_elapsed_time(const TimePoint& start, const TimePoint& end);

/**
 * @brief 現在時刻までの経過時間を計算
 * @param start 開始時刻
 * @return 経過時間（秒）
 */
double get_elapsed_time_since(const TimePoint& start);

/**
 * @brief 時間をフォーマットされた文字列に変換
 * @param seconds 時間（秒）
 * @return フォーマット済み文字列
 */
std::string format_duration(double seconds);

/**
 * @brief 現在時刻をタイムスタンプ文字列に変換
 * @return タイムスタンプ文字列
 */
std::string get_timestamp_string();

/**
 * @brief 高精度タイマークラス
 */
class Timer {
public:
    Timer();
    
    /**
     * @brief タイマーを開始
     */
    void start();
    
    /**
     * @brief タイマーを停止して経過時間を取得
     * @return 経過時間（秒）
     */
    double stop();
    
    /**
     * @brief タイマーをリセット
     */
    void reset();
    
    /**
     * @brief 現在の経過時間を取得（タイマーは継続）
     * @return 経過時間（秒）
     */
    double elapsed() const;
    
    /**
     * @brief タイマーが動作中かチェック
     * @return 動作中の場合true
     */
    bool is_running() const;

private:
    TimePoint start_time_;
    bool running_;
};

/**
 * @brief パフォーマンス測定用タイマークラス
 */
class PerformanceTimer {
public:
    PerformanceTimer();
    
    /**
     * @brief 測定を開始
     * @param label 測定ラベル
     */
    void start(const std::string& label);
    
    /**
     * @brief 測定を終了
     * @param label 測定ラベル
     * @return 経過時間（ミリ秒）
     */
    double end(const std::string& label);
    
    /**
     * @brief すべての測定結果をクリア
     */
    void clear();
    
    /**
     * @brief 測定結果のサマリーを取得
     * @return サマリー文字列
     */
    std::string get_summary() const;
    
    /**
     * @brief 特定ラベルの統計情報を取得
     * @param label 測定ラベル
     * @return 統計情報（平均、最小、最大、回数）
     */
    struct Statistics {
        double mean_ms;
        double min_ms;
        double max_ms;
        size_t count;
    };
    
    Statistics get_statistics(const std::string& label) const;

private:
    struct MeasurementData {
        TimePoint start_time;
        std::vector<double> durations_ms;
        bool is_measuring = false;
    };
    
    std::map<std::string, MeasurementData> measurements_;
};

/**
 * @brief レート制限クラス
 */
class RateLimiter {
public:
    /**
     * @brief コンストラクタ
     * @param max_rate_hz 最大レート（Hz）
     */
    explicit RateLimiter(double max_rate_hz);
    
    /**
     * @brief 実行可能かチェック
     * @return 実行可能な場合true
     */
    bool can_execute();
    
    /**
     * @brief 次回実行可能時刻までの待機時間を取得
     * @return 待機時間（秒）
     */
    double time_until_next() const;
    
    /**
     * @brief レートを変更
     * @param new_rate_hz 新しいレート（Hz）
     */
    void set_rate(double new_rate_hz);

private:
    double interval_seconds_;
    TimePoint last_execution_;
    bool first_execution_;
};

/**
 * @brief 自動タイマー（RAII）
 * スコープを抜ける際に自動的に時間を測定
 */
class ScopedTimer {
public:
    /**
     * @brief コンストラクタ
     * @param name タイマー名
     * @param auto_print スコープ終了時に自動出力するかどうか
     */
    explicit ScopedTimer(const std::string& name, bool auto_print = true);
    
    /**
     * @brief デストラクタ（自動測定）
     */
    ~ScopedTimer();
    
    /**
     * @brief 現在の経過時間を取得
     * @return 経過時間（ミリ秒）
     */
    double elapsed_ms() const;

private:
    std::string name_;
    bool auto_print_;
    Timer timer_;
};

} // namespace time_utils
} // namespace tvvf_vo_c

// 便利なマクロ
#define SCOPED_TIMER(name) tvvf_vo_c::time_utils::ScopedTimer _timer(name)
#define SCOPED_TIMER_SILENT(name) tvvf_vo_c::time_utils::ScopedTimer _timer(name, false)

#endif // TVVF_VO_C_UTILS_TIME_UTILS_HPP_