#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static constexpr double PI = 3.14159265358979323846;
static constexpr double DEG_TO_RAD = PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / PI;

struct Sample {
    double t_ms;
    double gx;
    double gy;
    double gz;
    double ax;
    double ay;
    double az;
    double online_roll;
    double online_pitch;
};

struct Quat {
    double w;
    double x;
    double y;
    double z;
};

struct Variant {
    std::string name;
    double kp;
    double ki;
    double min_g;
    double max_g;
    double band_g;
};

struct Series {
    std::vector<double> roll;
    std::vector<double> pitch;
    double acc_used_pct = 0.0;
    double acc_weight_mean = 0.0;
};

struct Metric {
    int flight;
    std::string variant;
    double kp;
    double ki;
    double min_g;
    double max_g;
    double band_g;
    std::string ref;
    std::string axis;
    double rms;
    double p95;
    double mean;
    double final_diff;
    double acc_used_pct;
    double acc_weight_mean;
};

static double clampd(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

static double wrap180(double value)
{
    while (value > 180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
}

static Quat qnorm(Quat q)
{
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1.0e-12) return {1.0, 0.0, 0.0, 0.0};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

static Quat qmul(const Quat &a, const Quat &b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

static Quat qconj(const Quat &q)
{
    return {q.w, -q.x, -q.y, -q.z};
}

static std::array<double, 3> qrotate(const Quat &q, const std::array<double, 3> &v)
{
    Quat out = qmul(qmul(q, {0.0, v[0], v[1], v[2]}), qconj(q));
    return {out.x, out.y, out.z};
}

static Quat small_angle_quat(const std::array<double, 3> &delta)
{
    double angle = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    if (angle < 1.0e-12) {
        return qnorm({1.0, 0.5 * delta[0], 0.5 * delta[1], 0.5 * delta[2]});
    }
    double half = 0.5 * angle;
    double scale = std::sin(half) / angle;
    return {std::cos(half), delta[0] * scale, delta[1] * scale, delta[2] * scale};
}

static Quat quat_from_gravity(double gx, double gy, double gz)
{
    double n = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (n < 1.0e-12) return {1.0, 0.0, 0.0, 0.0};
    return qnorm({gz + n, gy, -gx, 0.0});
}

static Quat integrate_quat(Quat q, double wx, double wy, double wz, double dt)
{
    double tx = wx * 0.5 * dt;
    double ty = wy * 0.5 * dt;
    double tz = wz * 0.5 * dt;
    double mag2 = tx * tx + ty * ty + tz * tz;
    if (mag2 < 1.0e-20) return q;
    Quat dq;
    if (mag2 < 0.00489898) {
        double scale = 1.0 - mag2 / 6.0;
        dq = {1.0 - mag2 / 2.0, tx * scale, ty * scale, tz * scale};
    } else {
        double mag = std::sqrt(mag2);
        double scale = std::sin(mag) / mag;
        dq = {std::cos(mag), tx * scale, ty * scale, tz * scale};
    }
    return qnorm(qmul(q, dq));
}

static std::array<double, 3> measured_gravity(double ax, double ay, double az, double *mag_out)
{
    double mag = std::sqrt(ax * ax + ay * ay + az * az);
    if (mag_out) *mag_out = mag;
    if (mag < 1.0e-12) return {0.0, 0.0, 1.0};
    return {-ax / mag, -ay / mag, -az / mag};
}

static std::array<double, 3> estimated_gravity(const Quat &q)
{
    return {
        2.0 * (q.x * q.z - q.w * q.y),
        2.0 * (q.w * q.x + q.y * q.z),
        q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
    };
}

static std::array<double, 2> roll_pitch(const Quat &q)
{
    double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                             1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * RAD_TO_DEG;
    double sp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = std::asin(clampd(sp, -1.0, 1.0)) * RAD_TO_DEG;
    return {roll, pitch};
}

static double pt1(double state, double value, double cutoff_hz, double dt)
{
    double rc = 1.0 / (2.0 * PI * cutoff_hz);
    double alpha = clampd(dt / (rc + dt), 0.0, 1.0);
    return state + alpha * (value - state);
}

static bool parse_csv_line(const std::string &line, std::array<double, 41> *out)
{
    std::stringstream ss(line);
    std::string cell;
    int idx = 0;
    while (std::getline(ss, cell, ',') && idx < 41) {
        (*out)[idx++] = std::strtod(cell.c_str(), nullptr);
    }
    return idx >= 15;
}

static std::vector<Sample> read_csv(const std::string &path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::string line;
    std::getline(in, line);
    std::vector<Sample> samples;
    std::array<double, 41> row{};
    while (std::getline(in, line)) {
        if (!parse_csv_line(line, &row)) continue;
        samples.push_back({row[0], row[7], row[8], row[9], row[10], row[11], row[12], row[13], row[14]});
    }
    return samples;
}

static Series run_mahony(const std::vector<Sample> &s, const Variant &v)
{
    Series out;
    out.roll.resize(s.size());
    out.pitch.resize(s.size());
    double mag0 = 1.0;
    auto g0 = measured_gravity(s[0].ax, s[0].ay, s[0].az, &mag0);
    Quat q = quat_from_gravity(g0[0], g0[1], g0[2]);
    std::array<double, 3> gyro_lpf{0.0, 0.0, 0.0};
    bool lpf_ready = false;
    double ix = 0.0;
    double iy = 0.0;
    int static_count = 0;
    double acc_used = 0.0;
    double weight_sum = 0.0;
    for (size_t i = 0; i < s.size(); ++i) {
        double dt = (i == 0) ? 0.001 : std::max((s[i].t_ms - s[i - 1].t_ms) * 0.001, 1.0e-6);
        double wx = s[i].gx * DEG_TO_RAD;
        double wy = s[i].gy * DEG_TO_RAD;
        double wz = s[i].gz * DEG_TO_RAD;
        double amag = 1.0;
        auto g = measured_gravity(s[i].ax, s[i].ay, s[i].az, &amag);
        double gyro_abs = std::sqrt(s[i].gx * s[i].gx + s[i].gy * s[i].gy + s[i].gz * s[i].gz);
        if (gyro_abs < 1.5 && amag > v.min_g && amag < v.max_g && std::fabs(amag - 1.0) < 0.08) {
            static_count = std::min(static_count + 1, 65535);
        } else {
            static_count = 0;
        }
        bool is_static = static_count >= 100;
        if (!lpf_ready) {
            gyro_lpf = {wx, wy, wz};
            lpf_ready = true;
        } else {
            gyro_lpf[0] = pt1(gyro_lpf[0], wx, 3.0, dt);
            gyro_lpf[1] = pt1(gyro_lpf[1], wy, 3.0, dt);
            gyro_lpf[2] = pt1(gyro_lpf[2], wz, 3.0, dt);
        }
        double weight = 0.0;
        if (amag > v.min_g && amag < v.max_g) {
            double near = clampd(1.0 - std::fabs(amag - 1.0) / v.band_g, 0.0, 1.0);
            double rate_dps = std::sqrt(gyro_lpf[0] * gyro_lpf[0] + gyro_lpf[1] * gyro_lpf[1] + gyro_lpf[2] * gyro_lpf[2]) * RAD_TO_DEG;
            double rate = 1.0;
            if (rate_dps > 12.0) {
                rate = (rate_dps >= 30.0) ? 0.0 : 1.0 - (rate_dps - 12.0) / 18.0;
            }
            weight = near * rate;
        }
        double cx = 0.0, cy = 0.0, cz = 0.0;
        if (weight > 0.0) {
            auto eg = estimated_gravity(q);
            double ex = g[1] * eg[2] - g[2] * eg[1];
            double ey = g[2] * eg[0] - g[0] * eg[2];
            double ez = g[0] * eg[1] - g[1] * eg[0];
            if (v.ki > 0.0 && is_static) {
                double lim = 2.0 * DEG_TO_RAD;
                ix = clampd(ix + v.ki * ex * weight * dt, -lim, lim);
                iy = clampd(iy + v.ki * ey * weight * dt, -lim, lim);
            }
            cx = v.kp * weight * ex;
            cy = v.kp * weight * ey;
            cz = v.kp * weight * ez;
            acc_used += 1.0;
        }
        weight_sum += weight;
        q = integrate_quat(q, wx + ix + cx, wy + iy + cy, wz + cz, dt);
        auto rp = roll_pitch(q);
        out.roll[i] = rp[0];
        out.pitch[i] = rp[1];
    }
    out.acc_used_pct = 100.0 * acc_used / std::max<size_t>(1, s.size());
    out.acc_weight_mean = weight_sum / std::max<size_t>(1, s.size());
    return out;
}

static Series run_madgwick(const std::vector<Sample> &s, double beta)
{
    Series out;
    out.roll.resize(s.size());
    out.pitch.resize(s.size());
    double mag0 = 1.0;
    auto g0 = measured_gravity(s[0].ax, s[0].ay, s[0].az, &mag0);
    Quat q = quat_from_gravity(g0[0], g0[1], g0[2]);
    for (size_t i = 0; i < s.size(); ++i) {
        double dt = (i == 0) ? 0.001 : std::max((s[i].t_ms - s[i - 1].t_ms) * 0.001, 1.0e-6);
        double gx = s[i].gx * DEG_TO_RAD;
        double gy = s[i].gy * DEG_TO_RAD;
        double gz = s[i].gz * DEG_TO_RAD;
        double amag = 1.0;
        auto g = measured_gravity(s[i].ax, s[i].ay, s[i].az, &amag);
        Quat qdot = qmul(q, {0.0, gx, gy, gz});
        qdot = {0.5 * qdot.w, 0.5 * qdot.x, 0.5 * qdot.y, 0.5 * qdot.z};
        if (amag > 1.0e-12) {
            double q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;
            double f1 = 2.0 * (q2 * q4 - q1 * q3) - g[0];
            double f2 = 2.0 * (q1 * q2 + q3 * q4) - g[1];
            double f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - g[2];
            std::array<double, 4> step{
                -2.0 * q3 * f1 + 2.0 * q2 * f2,
                2.0 * q4 * f1 + 2.0 * q1 * f2 - 4.0 * q2 * f3,
                -2.0 * q1 * f1 + 2.0 * q4 * f2 - 4.0 * q3 * f3,
                2.0 * q2 * f1 + 2.0 * q3 * f2,
            };
            double sn = std::sqrt(step[0] * step[0] + step[1] * step[1] + step[2] * step[2] + step[3] * step[3]);
            if (sn > 1.0e-12) {
                qdot.w -= beta * step[0] / sn;
                qdot.x -= beta * step[1] / sn;
                qdot.y -= beta * step[2] / sn;
                qdot.z -= beta * step[3] / sn;
            }
        }
        q = qnorm({q.w + qdot.w * dt, q.x + qdot.x * dt, q.y + qdot.y * dt, q.z + qdot.z * dt});
        auto rp = roll_pitch(q);
        out.roll[i] = rp[0];
        out.pitch[i] = rp[1];
    }
    return out;
}

static Series run_quat_ekf(const std::vector<Sample> &s)
{
    Series out;
    out.roll.resize(s.size());
    out.pitch.resize(s.size());
    double amag0 = 1.0;
    auto g0 = measured_gravity(s[0].ax, s[0].ay, s[0].az, &amag0);
    Quat q = quat_from_gravity(g0[0], g0[1], g0[2]);
    std::array<std::array<double, 3>, 3> p{};
    for (int i = 0; i < 3; ++i) p[i][i] = (2.0 * DEG_TO_RAD) * (2.0 * DEG_TO_RAD);
    double used = 0.0;
    double wsum = 0.0;
    for (size_t i = 0; i < s.size(); ++i) {
        double dt = (i == 0) ? 0.001 : std::max((s[i].t_ms - s[i - 1].t_ms) * 0.001, 1.0e-6);
        double wx = s[i].gx * DEG_TO_RAD;
        double wy = s[i].gy * DEG_TO_RAD;
        double wz = s[i].gz * DEG_TO_RAD;
        q = integrate_quat(q, wx, wy, wz, dt);
        double qn = (0.020 * dt) * (0.020 * dt);
        for (int j = 0; j < 3; ++j) p[j][j] += qn;
        double amag = 1.0;
        auto z = measured_gravity(s[i].ax, s[i].ay, s[i].az, &amag);
        double acc_err = std::fabs(amag - 1.0);
        double weight = 0.0;
        if (acc_err <= 0.70) {
            weight = (acc_err <= 0.35) ? 1.0 : 1.0 - (acc_err - 0.35) / 0.35;
            weight = clampd(weight, 0.0, 1.0);
            auto h = qrotate(q, {0.0, 0.0, 1.0});
            std::array<double, 3> innov{z[0] - h[0], z[1] - h[1], z[2] - h[2]};
            // Diagonal approximation of the error-state EKF update. This preserves the
            // quaternion innovation direction while keeping the grid search fast.
            double r = (0.060 / std::max(weight, 0.05)) * (0.060 / std::max(weight, 0.05));
            std::array<double, 3> delta{
                p[0][0] / (p[0][0] + r) * (h[1] * innov[2] - h[2] * innov[1]),
                p[1][1] / (p[1][1] + r) * (h[2] * innov[0] - h[0] * innov[2]),
                p[2][2] / (p[2][2] + r) * (h[0] * innov[1] - h[1] * innov[0]),
            };
            q = qnorm(qmul(small_angle_quat(delta), q));
            for (int j = 0; j < 3; ++j) p[j][j] = (1.0 - p[j][j] / (p[j][j] + r)) * p[j][j];
            used += 1.0;
        }
        wsum += weight;
        auto rp = roll_pitch(q);
        out.roll[i] = rp[0];
        out.pitch[i] = rp[1];
    }
    out.acc_used_pct = 100.0 * used / std::max<size_t>(1, s.size());
    out.acc_weight_mean = wsum / std::max<size_t>(1, s.size());
    return out;
}

static double median_offset(const std::vector<double> &ref, const std::vector<double> &value, const std::vector<Sample> &s)
{
    std::vector<double> diff;
    double end = s.front().t_ms + 500.0;
    for (size_t i = 0; i < s.size() && s[i].t_ms <= end; ++i) {
        diff.push_back(ref[i] - value[i]);
    }
    if (diff.empty()) return 0.0;
    std::nth_element(diff.begin(), diff.begin() + diff.size() / 2, diff.end());
    return diff[diff.size() / 2];
}

static std::array<double, 4> compare_axis(const std::vector<double> &online, const std::vector<double> &a,
                                          const std::vector<double> &b, const std::vector<Sample> &s)
{
    double off_a = median_offset(online, a, s);
    double off_b = median_offset(online, b, s);
    std::vector<double> absdiff;
    double sum = 0.0;
    double sumsq = 0.0;
    double last = 0.0;
    int n = 0;
    double start = s.front().t_ms + 1000.0;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i].t_ms < start) continue;
        double d = (a[i] + off_a) - (b[i] + off_b);
        last = d;
        sum += d;
        sumsq += d * d;
        absdiff.push_back(std::fabs(d));
        ++n;
    }
    if (n == 0) return {0.0, 0.0, 0.0, 0.0};
    size_t pidx = std::min(absdiff.size() - 1, static_cast<size_t>(std::floor(0.95 * (absdiff.size() - 1))));
    std::nth_element(absdiff.begin(), absdiff.begin() + pidx, absdiff.end());
    return {std::sqrt(sumsq / n), absdiff[pidx], sum / n, last};
}

static std::vector<Variant> make_variants()
{
    std::vector<Variant> out;
    std::vector<double> kps{0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.40, 1.60, 2.00};
    std::vector<double> kis{0.0, 0.01, 0.02, 0.05};
    for (double kp : kps) {
        for (double ki : kis) {
            char name[96];
            std::snprintf(name, sizeof(name), "mahony_kp%.2f_ki%.2f_band0.35", kp, ki);
            out.push_back({name, kp, ki, 0.30, 3.00, 0.35});
        }
    }
    for (double band : {0.20, 0.30, 0.35, 0.50, 0.70}) {
        for (double kp : {0.40, 0.50, 0.60, 0.80, 1.00}) {
            char name[96];
            std::snprintf(name, sizeof(name), "mahony_kp%.2f_ki0.02_band%.2f", kp, band);
            out.push_back({name, kp, 0.02, 0.30, 3.00, band});
        }
    }
    out.push_back({"bf_gate_kp0.25_ki0.00_band0.10", 0.25, 0.0, 0.90, 1.10, 0.10});
    return out;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "usage: mahony_ekf_grid_search out.csv flight1.csv ...\n";
        return 2;
    }
    std::ofstream out(argv[1]);
    out << "flight,variant,kp,ki,min_g,max_g,band_g,ref,axis,rms_diff_deg,p95_abs_diff_deg,mean_diff_deg,final_diff_deg,acc_used_pct,acc_weight_mean\n";
    auto variants = make_variants();
    for (int arg = 2; arg < argc; ++arg) {
        int flight = arg - 1;
        auto samples = read_csv(argv[arg]);
        Series ekf = run_quat_ekf(samples);
        Series mad = run_madgwick(samples, 0.04);
        std::vector<double> online_roll(samples.size()), online_pitch(samples.size());
        for (size_t i = 0; i < samples.size(); ++i) {
            online_roll[i] = samples[i].online_roll;
            online_pitch[i] = samples[i].online_pitch;
        }
        for (const auto &v : variants) {
            Series mah = run_mahony(samples, v);
            for (const auto &ref_pair : {std::pair<const char *, const Series *>("quat_ekf", &ekf),
                                         std::pair<const char *, const Series *>("madgwick_imu", &mad)}) {
                auto mr = compare_axis(online_roll, mah.roll, ref_pair.second->roll, samples);
                auto mp = compare_axis(online_pitch, mah.pitch, ref_pair.second->pitch, samples);
                out << flight << "," << v.name << "," << v.kp << "," << v.ki << "," << v.min_g << "," << v.max_g << "," << v.band_g
                    << "," << ref_pair.first << ",roll," << mr[0] << "," << mr[1] << "," << mr[2] << "," << mr[3] << "," << mah.acc_used_pct << "," << mah.acc_weight_mean << "\n";
                out << flight << "," << v.name << "," << v.kp << "," << v.ki << "," << v.min_g << "," << v.max_g << "," << v.band_g
                    << "," << ref_pair.first << ",pitch," << mp[0] << "," << mp[1] << "," << mp[2] << "," << mp[3] << "," << mah.acc_used_pct << "," << mah.acc_weight_mean << "\n";
            }
        }
        auto er = compare_axis(online_roll, mad.roll, ekf.roll, samples);
        auto ep = compare_axis(online_pitch, mad.pitch, ekf.pitch, samples);
        out << flight << ",madgwick_vs_ekf,0,0,0,0,0,quat_ekf,roll," << er[0] << "," << er[1] << "," << er[2] << "," << er[3] << "," << ekf.acc_used_pct << "," << ekf.acc_weight_mean << "\n";
        out << flight << ",madgwick_vs_ekf,0,0,0,0,0,quat_ekf,pitch," << ep[0] << "," << ep[1] << "," << ep[2] << "," << ep[3] << "," << ekf.acc_used_pct << "," << ekf.acc_weight_mean << "\n";
    }
    return 0;
}
