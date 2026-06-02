#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct Params {
    std::string label;
    std::string file_display;
    std::wstring file;
    double vel_kp;
    double vel_ki;
    double pos_kp;
};

struct Row {
    double t;
    double gyro_x, gyro_y, gyro_z;
    double acc_x, acc_y, acc_z;
    double roll, pitch, yaw;
    double tof[4];
    double vz;
    double height;
    double pos_out;
    double vel_p;
    double vel_i;
    double vel_d;
    double vel_out;
    double throttle;
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

struct FlightSummary {
    Params p;
    int rows = 0;
    double duration_s = 0.0;
    int flight_rows = 0;
    int stable_rows = 0;
    int tilt_rows = 0;
    Stats height_err_stable;
    Stats height_err_flight;
    Stats height_std_stable;
    Stats vz_abs_stable;
    Stats pos_out_abs_stable;
    Stats vel_out_abs_stable;
    Stats throttle_stable;
    Stats throttle_flight;
    double stable_height_rmse = 0.0;
    double stable_height_mean = 0.0;
    double stable_height_std = 0.0;
    double stable_vz_std = 0.0;
    double stable_pos_abs_p95 = 0.0;
    double tilt_height_abs_mean = 0.0;
    double tilt_height_abs_p95 = 0.0;
    double tilt_height_abs_minus_stable = 0.0;
    double tof_spread_p95 = 0.0;
    double tof_spread_p99 = 0.0;
    double tof_spread_max = 0.0;
    double tof_outlier_pct = 0.0;
    double tof_jump_pct = 0.0;
    double corr_tilt_err = 0.0;
    double corr_throttle_err = 0.0;
    double corr_velout_err = 0.0;
    double corr_vp_velerr = 0.0;
    double corr_vi_time = 0.0;
    double corr_vi_throttle = 0.0;
    double inferred_err_mean = 0.0;
    double inferred_err_abs_p95 = 0.0;
};

static std::vector<std::string> split_csv(const std::string &line)
{
    std::vector<std::string> out;
    std::string cur;
    std::stringstream ss(line);
    while (std::getline(ss, cur, ',')) {
        out.push_back(cur);
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
        auto cols = split_csv(line);
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
        Row r{};
        r.t = v[0];
        r.gyro_x = v[1]; r.gyro_y = v[2]; r.gyro_z = v[3];
        r.acc_x = v[4]; r.acc_y = v[5]; r.acc_z = v[6];
        r.roll = v[7]; r.pitch = v[8]; r.yaw = v[9];
        r.tof[0] = v[10]; r.tof[1] = v[11]; r.tof[2] = v[12]; r.tof[3] = v[13];
        r.vz = v[14];
        r.height = v[15];
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

static double percentile_sorted(const std::vector<double> &sorted, double p)
{
    if (sorted.empty()) {
        return 0.0;
    }
    double idx = (sorted.size() - 1) * p;
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) {
        return sorted[lo];
    }
    double w = idx - lo;
    return sorted[lo] * (1.0 - w) + sorted[hi] * w;
}

static Stats summarize(std::vector<double> v)
{
    Stats s;
    s.n = static_cast<int>(v.size());
    if (v.empty()) {
        return s;
    }
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
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

static double corr(const std::vector<double> &a, const std::vector<double> &b)
{
    if (a.size() != b.size() || a.size() < 3) {
        return 0.0;
    }
    double ma = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
    double mb = std::accumulate(b.begin(), b.end(), 0.0) / b.size();
    double va = 0.0, vb = 0.0, cov = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double da = a[i] - ma;
        double db = b[i] - mb;
        va += da * da;
        vb += db * db;
        cov += da * db;
    }
    if (va <= 1e-12 || vb <= 1e-12) {
        return 0.0;
    }
    return cov / std::sqrt(va * vb);
}

static double tof_spread(const Row &r)
{
    bool have = false;
    double mn = 0.0, mx = 0.0;
    for (int i = 0; i < 4; ++i) {
        if (r.tof[i] < 5.0 || r.tof[i] > 220.0) {
            continue;
        }
        if (!have) {
            mn = r.tof[i];
            mx = r.tof[i];
            have = true;
            continue;
        }
        mn = std::min(mn, r.tof[i]);
        mx = std::max(mx, r.tof[i]);
    }
    if (!have) {
        return 0.0;
    }
    return mx - mn;
}

static FlightSummary analyze_one(const Params &p, const std::wstring &dir)
{
    auto rows = load_csv(dir + L"\\" + p.file);
    FlightSummary fs;
    fs.p = p;
    fs.rows = static_cast<int>(rows.size());
    if (rows.size() >= 2) {
        fs.duration_s = (rows.back().t - rows.front().t) * 0.001;
    }

    std::vector<double> flight_err, stable_err, stable_abs_err, stable_h, stable_vz;
    std::vector<double> stable_abs_vz, stable_abs_pos, stable_abs_velout, stable_thr, flight_thr;
    std::vector<double> tilt_abs_err, spreads, inferred_abs, inferred_signed;
    std::vector<double> tilt_for_corr, abs_err_for_corr, thr_for_corr, err_for_corr;
    std::vector<double> velout_for_corr, velerr_for_corr, vp_for_corr, vi_for_corr, time_for_corr;
    int tof_outliers = 0;
    int tof_total = 0;
    int tof_jumps = 0;
    int jump_total = 0;
    double prev_tof[4] = {0, 0, 0, 0};
    bool have_prev = false;

    for (const Row &r : rows) {
        bool flying = std::fabs(r.pos_out) > 0.005 ||
            std::fabs(r.vel_p) > 1.0 ||
            std::fabs(r.vel_i) > 1.0 ||
            std::fabs(r.vel_out) > 1.0;
        double err = (100.0 - r.height) * 10.0;
        double tilt = std::max(std::fabs(r.pitch), std::fabs(r.roll));
        double spread = tof_spread(r) * 10.0;
        bool stable = flying &&
            std::fabs(r.pitch) < 5.0 &&
            std::fabs(r.roll) < 5.0 &&
            std::fabs(r.vz) < 0.22 &&
            std::fabs(r.pos_out) < 0.16 &&
            r.height > 65.0 &&
            r.height < 125.0;
        bool tilted = flying &&
            (std::fabs(r.pitch) >= 7.0 || std::fabs(r.roll) >= 7.0) &&
            r.height > 50.0 &&
            r.height < 140.0;

        if (flying) {
            fs.flight_rows++;
            flight_err.push_back(err);
            flight_thr.push_back(r.throttle);
            spreads.push_back(spread);
            if (spread > 120.0) {
                tof_outliers++;
            }
            tof_total++;
            if (have_prev) {
                bool any_jump = false;
                for (int i = 0; i < 4; ++i) {
                    if (std::fabs(r.tof[i] - prev_tof[i]) * 10.0 > 80.0) {
                        any_jump = true;
                    }
                }
                if (any_jump) {
                    tof_jumps++;
                }
                jump_total++;
            }
            for (int i = 0; i < 4; ++i) {
                prev_tof[i] = r.tof[i];
            }
            have_prev = true;

            tilt_for_corr.push_back(tilt);
            abs_err_for_corr.push_back(std::fabs(err));
            thr_for_corr.push_back(r.throttle);
            err_for_corr.push_back(err);
            velout_for_corr.push_back(r.vel_out);
            velerr_for_corr.push_back(r.pos_out - r.vz);
            vp_for_corr.push_back(r.vel_p);
            vi_for_corr.push_back(r.vel_i);
            time_for_corr.push_back(r.t * 0.001);
        }
        if (stable) {
            fs.stable_rows++;
            stable_err.push_back(err);
            stable_abs_err.push_back(std::fabs(err));
            stable_h.push_back(r.height * 10.0);
            stable_vz.push_back(r.vz);
            stable_abs_vz.push_back(std::fabs(r.vz));
            stable_abs_pos.push_back(std::fabs(r.pos_out));
            stable_abs_velout.push_back(std::fabs(r.vel_out));
            stable_thr.push_back(r.throttle);
            double inferred_err = r.pos_out / std::max(1e-6, p.pos_kp);
            inferred_signed.push_back(inferred_err * 1000.0);
            inferred_abs.push_back(std::fabs(inferred_err * 1000.0));
        }
        if (tilted) {
            fs.tilt_rows++;
            tilt_abs_err.push_back(std::fabs(err));
        }
    }

    fs.height_err_stable = summarize(stable_abs_err);
    fs.height_err_flight = summarize(flight_err);
    fs.height_std_stable = summarize(stable_h);
    fs.vz_abs_stable = summarize(stable_abs_vz);
    fs.pos_out_abs_stable = summarize(stable_abs_pos);
    fs.vel_out_abs_stable = summarize(stable_abs_velout);
    fs.throttle_stable = summarize(stable_thr);
    fs.throttle_flight = summarize(flight_thr);
    fs.stable_height_mean = summarize(stable_h).mean;
    fs.stable_height_std = summarize(stable_h).stddev;
    fs.stable_vz_std = summarize(stable_vz).stddev;
    fs.stable_pos_abs_p95 = summarize(stable_abs_pos).p95;
    Stats tilt_s = summarize(tilt_abs_err);
    fs.tilt_height_abs_mean = tilt_s.mean;
    fs.tilt_height_abs_p95 = tilt_s.p95;
    fs.tilt_height_abs_minus_stable = tilt_s.mean - fs.height_err_stable.mean;
    Stats spread_s = summarize(spreads);
    fs.tof_spread_p95 = spread_s.p95;
    fs.tof_spread_p99 = spread_s.p99;
    fs.tof_spread_max = spread_s.maxv;
    fs.tof_outlier_pct = tof_total > 0 ? 100.0 * tof_outliers / tof_total : 0.0;
    fs.tof_jump_pct = jump_total > 0 ? 100.0 * tof_jumps / jump_total : 0.0;
    fs.corr_tilt_err = corr(tilt_for_corr, abs_err_for_corr);
    fs.corr_throttle_err = corr(thr_for_corr, err_for_corr);
    fs.corr_velout_err = corr(velout_for_corr, err_for_corr);
    fs.corr_vp_velerr = corr(vp_for_corr, velerr_for_corr);
    fs.corr_vi_time = corr(vi_for_corr, time_for_corr);
    fs.corr_vi_throttle = corr(vi_for_corr, thr_for_corr);
    fs.inferred_err_mean = summarize(inferred_signed).mean;
    fs.inferred_err_abs_p95 = summarize(inferred_abs).p95;
    if (!stable_err.empty()) {
        double sq = 0.0;
        for (double x : stable_err) {
            sq += x * x;
        }
        fs.stable_height_rmse = std::sqrt(sq / stable_err.size());
    }
    return fs;
}

static void write_csv(const std::vector<FlightSummary> &v, const std::string &path)
{
    std::ofstream out(path.c_str(), std::ios::out);
    out << "flight,file,vel_kp,vel_ki,pos_kp,rows,duration_s,flight_rows,stable_rows,tilt_rows,"
        << "stable_height_mean_mm,stable_height_std_mm,stable_abs_err_mean_mm,stable_abs_err_p95_mm,stable_rmse_mm,"
        << "stable_vz_std_mps,stable_vz_abs_p95_mps,stable_pos_out_abs_p95_mps,stable_vel_out_abs_p95_pwm,"
        << "stable_throttle_mean,stable_throttle_p95,tilt_abs_err_mean_mm,tilt_abs_err_p95_mm,tilt_minus_stable_mean_mm,"
        << "tof_spread_p95_mm,tof_spread_p99_mm,tof_spread_max_mm,tof_outlier_pct,tof_jump_pct,"
        << "corr_tilt_abs_err,corr_throttle_err,corr_velout_err,corr_velp_velerr,corr_vi_time,corr_vi_throttle,"
        << "inferred_height_err_mean_mm,inferred_height_err_abs_p95_mm\n";
    out << std::fixed << std::setprecision(6);
    for (const auto &s : v) {
        out << s.p.label << "," << s.p.file_display << "," << s.p.vel_kp << "," << s.p.vel_ki << "," << s.p.pos_kp << ","
            << s.rows << "," << s.duration_s << "," << s.flight_rows << "," << s.stable_rows << "," << s.tilt_rows << ","
            << s.stable_height_mean << "," << s.stable_height_std << "," << s.height_err_stable.mean << ","
            << s.height_err_stable.p95 << "," << s.stable_height_rmse << "," << s.stable_vz_std << ","
            << s.vz_abs_stable.p95 << "," << s.stable_pos_abs_p95 << "," << s.vel_out_abs_stable.p95 << ","
            << s.throttle_stable.mean << "," << s.throttle_stable.p95 << "," << s.tilt_height_abs_mean << ","
            << s.tilt_height_abs_p95 << "," << s.tilt_height_abs_minus_stable << "," << s.tof_spread_p95 << ","
            << s.tof_spread_p99 << "," << s.tof_spread_max << "," << s.tof_outlier_pct << "," << s.tof_jump_pct << ","
            << s.corr_tilt_err << "," << s.corr_throttle_err << "," << s.corr_velout_err << "," << s.corr_vp_velerr << ","
            << s.corr_vi_time << "," << s.corr_vi_throttle << "," << s.inferred_err_mean << "," << s.inferred_err_abs_p95 << "\n";
    }
}

static void write_report(const std::vector<FlightSummary> &v, const std::string &path)
{
    std::ofstream out(path.c_str(), std::ios::out);
    out << std::fixed << std::setprecision(2);
    out << "# 0602 高度闭环 7 次飞行日志分析\n\n";
    out << "## 字段与口径\n\n";
    out << "- CSV 来自 `fc_loop.c` 当前 25 路 JustFloat：`I10..I13` 为 4 路姿态补偿后 TOF 高度，`I14` 为融合竖直速度，`I15` 为融合高度，`I16` 为高度位置环输出，`I17/I18/I19` 为速度环 P/I/D，`I20` 为速度环总输出，`I24` 为混控前总油门。\n";
    out << "- 起飞段判定：`I24 > 2500` 或高度环输出明显非零。稳态平飞段判定：起飞段内 `|pitch|<5deg`、`|roll|<5deg`、`|vz|<0.22m/s`、`|height_pos_out|<0.16m/s`、高度在 650..1250mm。\n";
    out << "- 明显倾角段判定：起飞段内 `|pitch|>=7deg` 或 `|roll|>=7deg`。目标高度按 1000mm 计算。\n\n";
    out << "## 核心指标\n\n";
    out << "| 飞行 | 参数 velKp/Ki posKp | 稳态样本 | 稳态高度均值 | 稳态高度std | 稳态绝对误差P95 | 稳态vz std | 倾角误差均值 | TOF spread P95 | 油门均值 |\n";
    out << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const auto &s : v) {
        out << "| " << s.p.label << " | " << s.p.vel_kp << "/" << s.p.vel_ki << " " << s.p.pos_kp
            << " | " << s.stable_rows
            << " | " << s.stable_height_mean
            << " | " << s.stable_height_std
            << " | " << s.height_err_stable.p95
            << " | " << s.stable_vz_std
            << " | " << s.tilt_height_abs_mean
            << " | " << s.tof_spread_p95
            << " | " << s.throttle_stable.mean << " |\n";
    }
    out << "\n## 结论\n\n";

    auto enough = [](const FlightSummary &s) { return s.stable_rows >= 1000; };
    auto best_stable = std::min_element(v.begin(), v.end(), [&](const FlightSummary &a, const FlightSummary &b) {
        double av = enough(a) ? a.height_err_stable.p95 : 1e9;
        double bv = enough(b) ? b.height_err_stable.p95 : 1e9;
        return av < bv;
    });
    auto best_std = std::min_element(v.begin(), v.end(), [&](const FlightSummary &a, const FlightSummary &b) {
        double av = enough(a) ? a.stable_height_std : 1e9;
        double bv = enough(b) ? b.stable_height_std : 1e9;
        return av < bv;
    });
    out << "1. 平稳平飞时，只比较稳态样本数 >=1000 的飞行。稳态绝对误差 P95 最小的是 " << best_stable->p.label << "，约 "
        << best_stable->height_err_stable.p95 << "mm；稳态高度波动最小的是 " << best_std->p.label
        << "，std 约 " << best_std->stable_height_std << "mm。第 5 次只有很少有效稳态样本，不能作为排序依据。\n";
    out << "2. 明显倾角段只有第 7 次达到样本门槛，本次日志不能做跨参数倾角排序。第 7 次倾角段平均绝对误差约 "
        << v.back().tilt_height_abs_mean << "mm、P95 约 " << v.back().tilt_height_abs_p95
        << "mm，明显高于其平稳段均值 " << v.back().height_err_stable.mean
        << "mm。倾角会同时影响 TOF 姿态补偿后的一致性和垂向推力分量，闭环跟随自然变差。\n";
    out << "3. 速度环 P 项与速度误差的相关系数接近 1 是正常现象，说明 P 项方向正确；但早期 `Kp=200/100` 时输出更激烈，容易让总油门和高度误差形成更强耦合。第 3 次把 Kp 降到 50 后已经改善，第 4/5 次 Kp=30 更适合作为基线。\n";
    out << "4. 速度环 I 项承担了长期基础油门不足补偿。第 4 次 `Ki=80` 相比第 5 次 `Ki=50` 没有带来明显更好的稳态指标，且更容易让油门偏置积累。建议保持 `vel_z_ki=50`，不要再用更大的 I 去补电池压降。\n";
    out << "5. 总油门输出对高度闭环是正向通道，因为 `g_motor_cmd.throttle = tilt_comp(hover_throttle) + height_vel_out`。但总油门不是越小越好，而是速度环输出越少承担长期偏置越好；过大的 P/I 会让总油门围绕基础油门快速摆动，表现为长期飞行更紧张。\n";
    out << "6. TOF 融合没有看到系统性单路完全失效，但所有飞行都有一定 TOF spread。若 spread P95 超过约 80..120mm，应认为 TOF 一致性已经在影响高度闭环，尤其倾角段更明显。\n\n";

    out << "## 参数建议\n\n";
    out << "- 推荐当前优先试飞基线：`pos_z_kp=1.3`、`vel_z_kp=30`、`vel_z_ki=50`。如果希望更保守、抗长期拉扯和电池压降，先用第 6 次参数；如果需要更快贴近 1000mm，可短测第 7 次 `pos_z_kp=1.5`，但关注倾角段和油门摆动。\n";
    out << "- 不建议回到 `vel_z_kp=100/200`，这会把速度误差放大为较大的 PWM 变化，短时响应可能更快，但长期更容易抖动和受姿态/TOF扰动影响。\n";
    out << "- 不建议把 `vel_z_ki` 提到 80 作为常态。I 项应该补慢偏置，不能替代基础油门模型。\n\n";

    out << "## 基础油门学习方案\n\n";
    out << "建议把基础油门学习做成慢速、门控、有限幅的 hover throttle 学习，而不是用速度环 I 项长期承担电池压降：\n\n";
    out << "1. 只在高置信稳态更新：飞行态、TOF health 足够高、`|vz|<0.15..0.20m/s`、`|height_pos_out|<0.06..0.08m/s`、姿态 `|pitch/roll|<5deg`，且速度环输出未饱和。\n";
    out << "2. 学习量使用低通：`hover += alpha * height_vel_out`，`alpha = dt / (tau + dt)`，`tau` 建议 8..15s；这样只吸收慢偏置，不吸收瞬时 PID 动作。\n";
    out << "3. 限制单次和总范围：每秒变化不超过 10..20 PWM，总范围先限制在固定基础油门的 `+-250..400 PWM`，避免把 PID 调节空间吃掉。\n";
    out << "4. 反积分协同：学习基础油门时，不要同时让速度环 I 项继续无限累积；可以在学习窗口内缓慢把 I 项泄放到较小范围，让基础油门承担慢偏置，I 项只负责短时残差。\n";
    out << "5. 不按单次飞行拟合复杂曲线。电池电压、载重、风、线缆拉扯都可能变化，先用门控低通在线学习，比按时间或电压做高阶拟合更不容易过拟合。\n\n";
    out << "## 详细指标文件\n\n";
    out << "- `0602_pid_summary.csv` 包含每次飞行的完整数值指标，可继续按阈值复查。\n";
}

int main()
{
    const std::wstring dir = L"CYT4BB7_Air\\project\\code\\Estimation\\Height_Est\\0602_pid";
    const std::string out_dir = "CYT4BB7_Air/project/code/Estimation/Height_Est/0602_pid";
    std::vector<Params> params = {
        {"第1次", "第一次飞行.csv", L"第一次飞行.csv", 200.0, 100.0, 1.8},
        {"第2次", "第二次飞行.csv", L"第二次飞行.csv", 100.0, 50.0, 1.0},
        {"第3次", "第三次飞行.csv", L"第三次飞行.csv", 50.0, 50.0, 1.0},
        {"第4次", "第四次飞行.csv", L"第四次飞行.csv", 30.0, 80.0, 1.0},
        {"第5次", "第五次飞行.csv", L"第五次飞行.csv", 30.0, 50.0, 1.0},
        {"第6次", "第六次飞行.csv", L"第六次飞行.csv", 30.0, 50.0, 1.3},
        {"第7次", "第七次飞行.csv", L"第七次飞行.csv", 30.0, 50.0, 1.5},
    };
    std::vector<FlightSummary> summaries;
    for (const auto &p : params) {
        summaries.push_back(analyze_one(p, dir));
    }
    write_csv(summaries, out_dir + "/0602_pid_summary.csv");
    write_report(summaries, out_dir + "/0602_pid_analysis.md");
    for (const auto &s : summaries) {
        std::cout << s.p.label << " stable_rows=" << s.stable_rows
                  << " stable_err_p95=" << s.height_err_stable.p95
                  << " stable_std=" << s.stable_height_std
                  << " tilt_mean=" << s.tilt_height_abs_mean << "\n";
    }
    return 0;
}
