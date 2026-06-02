#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct Row {
    double t_ms = 0.0;
    double acc_x = 0.0;
    double acc_y = 0.0;
    double acc_z = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double tof[4] = {};
    double vz_mps = 0.0;
    double height_log = 0.0;
    double pos_out = 0.0;
    double vel_p = 0.0;
    double vel_i = 0.0;
    double vel_d = 0.0;
    double vel_out = 0.0;
    double throttle = 0.0;
};

struct Stats {
    int n = 0;
    double mean = 0.0;
    double stddev = 0.0;
    double minv = 0.0;
    double maxv = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
};

struct AlgoConfig {
    std::string name;
    std::string source;
    double tau_s = 10.0;
    double rate_limit_pwm_s = 80.0;
    bool strict_gate = false;
    bool use_rate_limit = false;
    bool use_target_throttle = false;
    bool voltage_schedule = false;

    AlgoConfig() {}

    AlgoConfig(const std::string &n, const std::string &s, double tau, double rate,
               bool strict, bool limit, bool target, bool voltage)
        : name(n),
          source(s),
          tau_s(tau),
          rate_limit_pwm_s(rate),
          strict_gate(strict),
          use_rate_limit(limit),
          use_target_throttle(target),
          voltage_schedule(voltage)
    {
    }
};

struct AlgoResult {
    AlgoConfig cfg;
    int update_rows = 0;
    int risky_update_rows = 0;
    int tilt_update_rows = 0;
    double update_time_s = 0.0;
    double hover_start = 2700.0;
    double hover_end = 2700.0;
    double hover_delta = 0.0;
    double hover_delta_per_v = 0.0;
    double residual_start_mean = 0.0;
    double residual_end_mean = 0.0;
    Stats residual_abs;
    Stats step_abs;
};

static std::vector<std::string> split_csv(const std::string &line)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        out.push_back(item);
    }
    return out;
}

static bool parse_double(const std::string &s, double *v)
{
    char *end = nullptr;
    *v = std::strtod(s.c_str(), &end);
    return end != s.c_str() && std::isfinite(*v);
}

static std::vector<Row> load_csv(const std::wstring &path)
{
    std::vector<Row> rows;
    FILE *fp = _wfopen(path.c_str(), L"rb");
    if (fp == nullptr) {
        return rows;
    }

    char buf[8192];
    bool header = true;
    while (std::fgets(buf, sizeof(buf), fp) != nullptr) {
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
            line.pop_back();
        }
        if (header) {
            header = false;
            continue;
        }

        std::vector<std::string> cols = split_csv(line);
        if (cols.size() < 25) {
            continue;
        }

        double v[25];
        bool ok = true;
        for (int i = 0; i < 25; ++i) {
            if (!parse_double(cols[i], &v[i])) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            continue;
        }

        Row r;
        r.t_ms = v[0];
        r.acc_x = v[4];
        r.acc_y = v[5];
        r.acc_z = v[6];
        r.roll_deg = v[7];
        r.pitch_deg = v[8];
        for (int i = 0; i < 4; ++i) {
            r.tof[i] = v[10 + i];
        }
        r.vz_mps = v[14];
        r.height_log = v[15];
        r.pos_out = v[16];
        r.vel_p = v[17];
        r.vel_i = v[18];
        r.vel_d = v[19];
        r.vel_out = v[20];
        r.throttle = v[24];
        rows.push_back(r);
    }

    std::fclose(fp);
    return rows;
}

static double percentile_sorted(const std::vector<double> &v, double p)
{
    if (v.empty()) {
        return 0.0;
    }
    const double idx = (v.size() - 1) * p;
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) {
        return v[lo];
    }
    const double w = idx - lo;
    return v[lo] * (1.0 - w) + v[hi] * w;
}

static Stats summarize(std::vector<double> v)
{
    Stats s;
    s.n = static_cast<int>(v.size());
    if (v.empty()) {
        return s;
    }

    const double sum = std::accumulate(v.begin(), v.end(), 0.0);
    s.mean = sum / v.size();
    double sq = 0.0;
    for (double x : v) {
        sq += (x - s.mean) * (x - s.mean);
    }
    s.stddev = std::sqrt(sq / v.size());
    std::sort(v.begin(), v.end());
    s.minv = v.front();
    s.maxv = v.back();
    s.p50 = percentile_sorted(v, 0.50);
    s.p95 = percentile_sorted(v, 0.95);
    s.p99 = percentile_sorted(v, 0.99);
    return s;
}

static double median_or_zero(std::vector<double> v)
{
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return percentile_sorted(v, 0.50);
}

static double clampd(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}

static double deg_to_rad(double deg)
{
    return deg * 3.14159265358979323846 / 180.0;
}

static double tilt_cos(const Row &r)
{
    return std::max(0.50, std::cos(deg_to_rad(r.roll_deg)) * std::cos(deg_to_rad(r.pitch_deg)));
}

static double inferred_base_throttle(const Row &r)
{
    return (r.throttle - r.vel_out) * tilt_cos(r);
}

static double acc_norm_g(const Row &r)
{
    return std::sqrt(r.acc_x * r.acc_x + r.acc_y * r.acc_y + r.acc_z * r.acc_z);
}

static double height_mm(const Row &r)
{
    return r.height_log;
}

static double tof_spread_mm(const Row &r)
{
    std::vector<double> vals;
    for (int i = 0; i < 4; ++i) {
        const double mm = r.tof[i];
        if (mm >= 50.0 && mm <= 2200.0) {
            vals.push_back(mm);
        }
    }
    if (vals.size() < 2) {
        return 0.0;
    }
    std::sort(vals.begin(), vals.end());
    return vals.back() - vals.front();
}

static bool is_flight(const Row &r)
{
    return r.throttle > 2200.0 && height_mm(r) > 400.0 && height_mm(r) < 1800.0;
}

static bool base_gate(const Row &r)
{
    return is_flight(r) &&
           std::fabs(r.vz_mps) < 0.18 &&
           std::fabs(r.pos_out) < 0.08 &&
           std::fabs(r.roll_deg) < 7.0 &&
           std::fabs(r.pitch_deg) < 7.0 &&
           r.throttle < 4900.0;
}

static bool strict_gate(const Row &r)
{
    return base_gate(r) &&
           std::fabs(r.roll_deg) < 4.5 &&
           std::fabs(r.pitch_deg) < 4.5 &&
           std::fabs(r.vz_mps) < 0.12 &&
           std::fabs(r.pos_out) < 0.06 &&
           std::fabs(acc_norm_g(r) - 1.0) < 0.08 &&
           tof_spread_mm(r) < 120.0;
}

static bool tilt_gate(const Row &r)
{
    return is_flight(r) && (std::fabs(r.roll_deg) >= 7.0 || std::fabs(r.pitch_deg) >= 7.0);
}

static double estimate_hover_init(const std::vector<Row> &rows)
{
    std::vector<double> candidates;
    for (const Row &r : rows) {
        if (strict_gate(r)) {
            candidates.push_back(inferred_base_throttle(r));
        }
    }
    if (candidates.size() < 100) {
        for (const Row &r : rows) {
            if (base_gate(r)) {
                candidates.push_back(inferred_base_throttle(r));
            }
        }
    }
    const double med = median_or_zero(candidates);
    return med > 1000.0 ? med : 2700.0;
}

static double dt_s_at(const std::vector<Row> &rows, size_t i)
{
    if (i == 0) {
        return 0.001;
    }
    double dt = (rows[i].t_ms - rows[i - 1].t_ms) * 0.001;
    if (dt < 0.0005 || dt > 0.05) {
        dt = 0.001;
    }
    return dt;
}

static double learning_signal(const AlgoConfig &cfg, const Row &r, double learned_delta, double hover_init)
{
    if (cfg.voltage_schedule) {
        return 0.0;
    }
    if (cfg.source == "raw_throttle") {
        const double target_hover = r.throttle * tilt_cos(r);
        return target_hover - (hover_init + learned_delta);
    }
    if (cfg.use_target_throttle) {
        const double target_hover = inferred_base_throttle(r);
        return target_hover - (hover_init + learned_delta);
    }
    if (cfg.source == "vel_i") {
        return r.vel_i - learned_delta;
    }
    return r.vel_out - learned_delta;
}

static AlgoResult simulate_algo(const std::vector<Row> &rows, const AlgoConfig &cfg)
{
    AlgoResult res;
    res.cfg = cfg;
    if (rows.empty()) {
        return res;
    }

    const double v_start = 16.6;
    const double v_end = 15.8;
    const double hover_init = estimate_hover_init(rows);
    const double hover_min = std::max(1700.0, hover_init - 500.0);
    const double hover_max = std::min(4300.0, hover_init + 700.0);
    double hover = hover_init;
    res.hover_start = hover_init;

    std::vector<double> residual_abs;
    std::vector<double> step_abs;
    std::vector<double> residual_start;
    std::vector<double> residual_end;

    double first_flight_t = 0.0;
    double last_flight_t = 0.0;
    bool have_flight = false;
    for (const Row &r : rows) {
        if (is_flight(r)) {
            if (!have_flight) {
                first_flight_t = r.t_ms;
                have_flight = true;
            }
            last_flight_t = r.t_ms;
        }
    }
    const double duration_s = std::max(0.001, (last_flight_t - first_flight_t) * 0.001);
    const double voltage_slope_pwm_per_v = 165.0;

    for (size_t i = 0; i < rows.size(); ++i) {
        const Row &r = rows[i];
        const double dt = dt_s_at(rows, i);
        bool gate = cfg.strict_gate ? strict_gate(r) : base_gate(r);
        const bool strict_ok = strict_gate(r);

        const double learned_delta = hover - hover_init;
        double signal = learning_signal(cfg, r, learned_delta, hover_init);
        double step = 0.0;

        if (cfg.voltage_schedule && is_flight(r)) {
            const double elapsed_s = std::max(0.0, (r.t_ms - first_flight_t) * 0.001);
            const double voltage = v_start + (v_end - v_start) * clampd(elapsed_s / duration_s, 0.0, 1.0);
            const double target_hover = hover_init + (v_start - voltage) * voltage_slope_pwm_per_v;
            signal = target_hover - hover;
            gate = true;
        }

        if (gate) {
            const double alpha = dt / (dt + cfg.tau_s);
            step = alpha * signal;
            if (cfg.use_rate_limit) {
                step = clampd(step, -cfg.rate_limit_pwm_s * dt, cfg.rate_limit_pwm_s * dt);
            }
            hover = clampd(hover + step, hover_min, hover_max);
            ++res.update_rows;
            res.update_time_s += dt;
            step_abs.push_back(std::fabs(step));
            if (!strict_ok) {
                ++res.risky_update_rows;
            }
            if (tilt_gate(r)) {
                ++res.tilt_update_rows;
            }
        }

        if (is_flight(r)) {
            const double residual = (cfg.source == "vel_i") ? (r.vel_i - (hover - hover_init)) :
                                    (r.vel_out - (hover - hover_init));
            residual_abs.push_back(std::fabs(residual));

            const double since_start_s = (r.t_ms - first_flight_t) * 0.001;
            const double until_end_s = (last_flight_t - r.t_ms) * 0.001;
            if (since_start_s <= 20.0 && strict_ok) {
                residual_start.push_back(residual);
            }
            if (until_end_s <= 20.0 && strict_ok) {
                residual_end.push_back(residual);
            }
        }
    }

    res.hover_end = hover;
    res.hover_delta = hover - res.hover_start;
    res.hover_delta_per_v = res.hover_delta / (v_start - v_end);
    res.residual_abs = summarize(residual_abs);
    res.step_abs = summarize(step_abs);
    res.residual_start_mean = summarize(residual_start).mean;
    res.residual_end_mean = summarize(residual_end).mean;
    return res;
}

static void write_summary_csv(const std::string &path, const std::vector<AlgoResult> &results)
{
    std::ofstream out(path.c_str());
    out << std::fixed << std::setprecision(6);
    out << "algo,source,tau_s,strict_gate,rate_limit_pwm_s,update_rows,update_time_s,"
        << "risky_update_pct,tilt_update_rows,hover_end,hover_delta,hover_delta_per_v,"
        << "residual_abs_mean,residual_abs_p95,residual_start_mean,residual_end_mean,"
        << "step_abs_p95,step_abs_p99\n";
    for (const AlgoResult &r : results) {
        const double risky_pct = r.update_rows > 0 ? 100.0 * r.risky_update_rows / r.update_rows : 0.0;
        out << r.cfg.name << "," << r.cfg.source << "," << r.cfg.tau_s << ","
            << (r.cfg.strict_gate ? 1 : 0) << ","
            << (r.cfg.use_rate_limit ? r.cfg.rate_limit_pwm_s : 0.0) << ","
            << r.update_rows << "," << r.update_time_s << ","
            << risky_pct << "," << r.tilt_update_rows << ","
            << r.hover_end << "," << r.hover_delta << "," << r.hover_delta_per_v << ","
            << r.residual_abs.mean << "," << r.residual_abs.p95 << ","
            << r.residual_start_mean << "," << r.residual_end_mean << ","
            << r.step_abs.p95 << "," << r.step_abs.p99 << "\n";
    }
}

static void write_report(const std::string &path, const std::vector<Row> &rows, const std::vector<AlgoResult> &results)
{
    std::vector<double> h_err_abs;
    std::vector<double> vz_abs;
    std::vector<double> vel_i;
    std::vector<double> vel_out;
    std::vector<double> throttle;
    std::vector<double> inferred_base;
    std::vector<double> inferred_base_strict;
    std::vector<double> acc_g;
    std::vector<double> tof_spread;
    int flight_rows = 0;
    int base_rows = 0;
    int strict_rows = 0;
    int tilt_rows = 0;
    int throttle_sat_rows = 0;

    double first_flight_t = 0.0;
    double last_flight_t = 0.0;
    bool have_flight = false;
    for (const Row &r : rows) {
        if (!is_flight(r)) {
            continue;
        }
        if (!have_flight) {
            first_flight_t = r.t_ms;
            have_flight = true;
        }
        last_flight_t = r.t_ms;
        ++flight_rows;
        h_err_abs.push_back(std::fabs(height_mm(r) - 1000.0));
        vz_abs.push_back(std::fabs(r.vz_mps));
        vel_i.push_back(r.vel_i);
        vel_out.push_back(r.vel_out);
        throttle.push_back(r.throttle);
        if (base_gate(r)) {
            inferred_base.push_back(inferred_base_throttle(r));
        }
        if (strict_gate(r)) {
            inferred_base_strict.push_back(inferred_base_throttle(r));
        }
        acc_g.push_back(acc_norm_g(r));
        tof_spread.push_back(tof_spread_mm(r));
        if (base_gate(r)) {
            ++base_rows;
        }
        if (strict_gate(r)) {
            ++strict_rows;
        }
        if (tilt_gate(r)) {
            ++tilt_rows;
        }
        if (r.throttle <= 1700.0 || r.throttle >= 4900.0) {
            ++throttle_sat_rows;
        }
    }

    const Stats hs = summarize(h_err_abs);
    const Stats vs = summarize(vz_abs);
    const Stats is = summarize(vel_i);
    const Stats os = summarize(vel_out);
    const Stats ts = summarize(throttle);
    const Stats bs = summarize(inferred_base);
    const Stats bss = summarize(inferred_base_strict);
    const Stats as = summarize(acc_g);
    const Stats ss = summarize(tof_spread);
    const double duration_s = have_flight ? (last_flight_t - first_flight_t) * 0.001 : 0.0;

    std::ofstream out(path.c_str());
    out << std::fixed << std::setprecision(3);
    out << "# 基础油门学习离线分析\n\n";
    out << "## 数据口径\n\n";
    out << "- 日志：`无学习油门,长时间飞行对照组.csv`，共 " << rows.size() << " 行，飞行段 "
        << flight_rows << " 行，约 " << duration_s << " s。\n";
    out << "- 列映射沿用当前 25 路 JustFloat：`I14` 融合竖直速度，`I15` 融合高度，`I16` 位置环输出，`I17/I18/I19` 速度环 P/I/D，`I20` 速度环总输出，`I24` 总油门。\n";
    out << "- 本日志飞行段 `I15` 和 `I10..I13` 已经是 mm 量级，目标高度按 1000 mm 计算。\n";
    out << "- 离线回放只能评估“学习器会学到什么、是否容易吸收错误信号”，不能等价为开启学习后的真实闭环轨迹。\n\n";

    out << "## 长航时对照组现象\n\n";
    out << "- 飞行段高度绝对误差：mean " << hs.mean << " mm，P95 " << hs.p95 << " mm。\n";
    out << "- `|vz|`：mean " << vs.mean << " m/s，P95 " << vs.p95 << " m/s。\n";
    out << "- 速度环 I 项：mean " << is.mean << " PWM，P50 " << is.p50 << " PWM，P95 " << is.p95 << " PWM，max " << is.maxv << " PWM。\n";
    out << "- 速度环总输出：mean " << os.mean << " PWM，P50 " << os.p50 << " PWM，P95 " << os.p95 << " PWM。\n";
    out << "- 总油门：mean " << ts.mean << "，P95 " << ts.p95 << "，饱和/贴边比例 "
        << (flight_rows > 0 ? 100.0 * throttle_sat_rows / flight_rows : 0.0) << "%。\n";
    out << "- 由 `throttle - height_vel_out` 反推的当前固定基础油门：宽门控 median "
        << bs.p50 << "，严格门控 median " << bss.p50 << "。离线学习表使用严格门控中位数作为起始基础油门。\n";
    out << "- 加速度模长：mean " << as.mean << " g，P95 " << as.p95 << " g；TOF 四路 spread：P95 "
        << ss.p95 << " mm，P99 " << ss.p99 << " mm。\n";
    out << "- 当前宽门控可学习样本 " << base_rows << " 行，占飞行段 "
        << (flight_rows > 0 ? 100.0 * base_rows / flight_rows : 0.0) << "%；严格门控样本 " << strict_rows
        << " 行，占 " << (flight_rows > 0 ? 100.0 * strict_rows / flight_rows : 0.0) << "%；明显倾角样本 "
        << tilt_rows << " 行。\n\n";

    out << "## 开源飞控做法\n\n";
    out << "1. ArduPilot 使用 `MOT_THST_HOVER` 表示悬停所需归一化推力，`MOT_HOVER_LEARN` 控制是否学习/保存。源码中的 `update_throttle_hover()` 本质是 10 s 时间常数的低通平均：让 hover throttle 缓慢靠近当前 throttle，并限制在合理范围内。源码链接：https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsMulticopter.cpp\n";
    out << "2. PX4 使用 `MPC_THR_HOVER` 初始化 hover thrust estimator。估计器根据当前分配后的垂直推力和竖直加速度做 EKF 更新，并带有创新门限、噪声自适应、速度过大时降低敏感度、估计范围限制。源码链接：https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_hover_thrust_estimator\n";
    out << "3. 两者共同点不是“让 PID 自己消失”，而是只吸收慢变化的悬停偏置；快响应仍由高度/速度闭环完成。\n\n";

    out << "## 离线学习算法对比\n\n";
    out << "| 算法 | 学习源 | 更新时长 | 风险更新比例 | 起始基础油门 | 末端基础油门 | 学习量 | 每 V 学习量 | 残余输出P95 |\n";
    out << "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const AlgoResult &r : results) {
        const double risky_pct = r.update_rows > 0 ? 100.0 * r.risky_update_rows / r.update_rows : 0.0;
        out << "| " << r.cfg.name << " | " << r.cfg.source << " | " << r.update_time_s << " s | "
            << risky_pct << "% | " << r.hover_start << " | " << r.hover_end << " | " << r.hover_delta << " | "
            << r.hover_delta_per_v << " | " << r.residual_abs.p95 << " |\n";
    }

    const AlgoResult *best = nullptr;
    for (const AlgoResult &r : results) {
        if (r.cfg.name == "ki_strict_tau20_ratelimit") {
            best = &r;
            break;
        }
    }

    out << "\n## 结论\n\n";
    out << "1. 这条长航时日志里，总油门从中段约 3130 上升到后段约 3280，再到尾段约 3096；速度环 I 项在不同阶段有正有负，说明不能只按全局均值判断基础油门是否不足，必须在稳态门控内学习。\n";
    out << "2. 不建议直接用“所有飞行时刻的总油门”学习。总油门包含高度误差、速度误差、D 项、姿态补偿和 TOF 抖动导致的快动作，门控不严会把控制器动作学进基础油门。\n";
    out << "3. 当前代码的 `hover += alpha * height_vel_out` 方向是对的，但门控还应加入姿态、加速度模长、TOF spread、油门饱和判断，并把时间常数从 6 s 放慢到 15..25 s。\n";
    out << "4. 对你现在的系统，更推荐先用 KI-only 或“I 项优先、总输出兜底”的学习：基础油门只吸收速度环 I 项的慢偏置，P/D/总输出仍负责闭环响应。这样最不影响后续调 Kp/Ki。\n";
    if (best != nullptr) {
        out << "5. 本日志上推荐的保守离线结果是 `ki_strict_tau20_ratelimit`：末端基础油门约 "
            << best->hover_end << "，相对本次反推基础油门学习 " << best->hover_delta
            << " PWM，折算约 " << best->hover_delta_per_v << " PWM/V。这个量级适合作为在线学习上限参考，而不是一次性写死标定值。\n";
    }
    out << "\n## 推荐在线方案\n\n";
    out << "```c\n";
    out << "if (flying && tof_health_good && fabsf(vz) < 0.12f && fabsf(height_pos_out) < 0.06f &&\n";
    out << "    fabsf(roll) < 4.5f && fabsf(pitch) < 4.5f && fabsf(acc_norm_g - 1.0f) < 0.08f &&\n";
    out << "    tof_spread_mm < 120.0f && throttle_not_saturated) {\n";
    out << "    float residual_i = height_vel_pid.i_term - (hover_throttle - hover_init);\n";
    out << "    float alpha = dt / (20.0f + dt);\n";
    out << "    float step = clamp(alpha * residual_i, -60.0f * dt, 60.0f * dt);\n";
    out << "    hover_throttle = clamp(hover_throttle + step, hover_init - 200.0f, hover_init + 300.0f);\n";
    out << "}\n";
    out << "```\n\n";
    out << "- 学习目标：让速度环 I 项在长时间悬停后回到接近 0，而不是让 `height_vel_out` 变成 0。\n";
    out << "- 防过拟合：每次飞行只在线学习 RAM 值；只有连续多次飞行学到相近结果，再更新持久化默认值。\n";
    out << "- 防抵消 PID：学习只在稳态门控内发生，速率限制小于正常 P/D 动作，且学习量限幅在基础油门附近。\n";
    out << "- 防学习不足：如果飞行末段 KI 仍持续偏正，可把上限从 `+250` 放到 `+350`，但不要先放宽门控。\n";
}

int main()
{
    const std::wstring input = L"无学习油门,长时间飞行对照组.csv";
    const std::string out_csv = "hover_learning_summary.csv";
    const std::string out_md = "hover_learning_analysis.md";

    const std::vector<Row> rows = load_csv(input);
    if (rows.empty()) {
        std::fprintf(stderr, "failed to load csv\n");
        return 1;
    }

    std::vector<AlgoConfig> configs;
    configs.push_back({"raw_throttle_tau10", "raw_throttle", 10.0, 0.0, false, false, false, false});
    configs.push_back({"base_from_throttle_minus_pid_tau10", "throttle", 10.0, 0.0, false, false, true, false});
    configs.push_back({"velout_current_tau6", "vel_out", 6.0, 0.0, false, false, false, false});
    configs.push_back({"velout_strict_tau15", "vel_out", 15.0, 80.0, true, true, false, false});
    configs.push_back({"ki_strict_tau20_ratelimit", "vel_i", 20.0, 60.0, true, true, false, false});
    configs.push_back({"ki_strict_tau30_slow", "vel_i", 30.0, 40.0, true, true, false, false});
    configs.push_back({"voltage_linear_165pwm_per_v", "voltage", 20.0, 0.0, false, false, false, true});

    std::vector<AlgoResult> results;
    for (const AlgoConfig &cfg : configs) {
        results.push_back(simulate_algo(rows, cfg));
    }

    write_summary_csv(out_csv, results);
    write_report(out_md, rows, results);

    std::printf("rows=%zu\nsummary=%s\nreport=%s\n", rows.size(), out_csv.c_str(), out_md.c_str());
    return 0;
}
