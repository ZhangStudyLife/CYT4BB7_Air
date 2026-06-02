#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct Row {
    double v[29] = {};
    int n = 0;
};

struct Stats {
    int n = 0;
    double mean = 0.0, p50 = 0.0, p95 = 0.0, p99 = 0.0, minv = 0.0, maxv = 0.0;
};

struct FileDef {
    const char *name;
    const wchar_t *path;
    bool learning;
    bool pull;
};

struct Segment {
    int start = 0, end = 0, gate = 0;
    double min_h = 1e9, max_vz = 0.0, max_pos = 0.0, hover0 = 0.0, hover1 = 0.0;
};

static std::vector<std::string> split_csv(const std::string &s)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ',')) out.push_back(x);
    return out;
}

static Stats stats(std::vector<double> x)
{
    Stats s;
    s.n = (int)x.size();
    if (x.empty()) return s;
    s.mean = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    std::sort(x.begin(), x.end());
    s.minv = x.front();
    s.maxv = x.back();
    auto p = [&](double q) {
        double i = (x.size() - 1) * q;
        size_t a = (size_t)std::floor(i), b = (size_t)std::ceil(i);
        return x[a] * (1.0 - (i - a)) + x[b] * (i - a);
    };
    s.p50 = p(0.50); s.p95 = p(0.95); s.p99 = p(0.99);
    return s;
}

static std::vector<Row> load(const wchar_t *path)
{
    std::vector<Row> rows;
    FILE *fp = _wfopen(path, L"rb");
    if (!fp) return rows;
    char buf[8192];
    bool header = true;
    while (std::fgets(buf, sizeof(buf), fp)) {
        if (header) {
            header = false;
            continue;
        }
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
        std::vector<std::string> c = split_csv(line);
        if (c.size() < 25) continue;
        Row r;
        r.n = (int)std::min<size_t>(29, c.size());
        bool ok = true;
        for (int i = 0; i < r.n; ++i) {
            char *end = 0;
            r.v[i] = std::strtod(c[i].c_str(), &end);
            if (end == c[i].c_str() || !std::isfinite(r.v[i])) ok = false;
        }
        if (ok) rows.push_back(r);
    }
    std::fclose(fp);
    return rows;
}

static double acc_g(const Row &r)
{
    return std::sqrt(r.v[4] * r.v[4] + r.v[5] * r.v[5] + r.v[6] * r.v[6]);
}

static double tof_spread(const Row &r)
{
    return std::max(std::max(r.v[10], r.v[11]), std::max(r.v[12], r.v[13])) -
           std::min(std::min(r.v[10], r.v[11]), std::min(r.v[12], r.v[13]));
}

static bool flying(const Row &r)
{
    return r.v[24] > 2200.0 && r.v[15] > 400.0 && r.v[15] < 1800.0;
}

static bool stable(const Row &r)
{
    return flying(r) && std::fabs(r.v[14]) < 0.18 && std::fabs(r.v[16]) < 0.08 &&
           std::fabs(r.v[7]) < 7.0 && std::fabs(r.v[8]) < 7.0 &&
           std::fabs(acc_g(r) - 1.0) < 0.20 && tof_spread(r) < 250.0;
}

static bool pull_event(const Row &r)
{
    return flying(r) && (r.v[15] < 850.0 || std::fabs(r.v[14]) > 0.45 || std::fabs(r.v[16]) > 0.22);
}

int main()
{
    FileDef files[] = {
        {"no_learning_no_pull", L"无学习油门,长时间飞行对照组.csv", false, false},
        {"learning_no_pull", L"学习油门,长时间飞行_第一次飞行,无线缆主动拉扯.csv", true, false},
        {"learning_pull", L"学习油门,长时间飞行_第二次飞行,线缆主动拉扯.csv", true, true},
    };

    std::ofstream csv("hover_learning_three_flights_summary.csv");
    csv << "flight,rows,duration_s,flight_rows,stable_rows,stable_pct,height_abs_mean,height_abs_p95,vz_abs_p95,pos_abs_p95,velout_abs_p95,throttle_mean,throttle_p95,hover_start,hover_end,hover_delta,learn_gate_pct,learn_step_abs_p95,raw_velout_abs_p95,pull_rows,pull_height_min,pull_height_abs_p95,pull_vz_abs_p95,pull_gate_rows,pull_hover_delta\n";

    std::ofstream md("hover_learning_three_flights_analysis.md");
    md << "# 三次长时间飞行高度闭环与学习油门对比\n\n";
    md << "列定义：新学习日志 `I25` 为学习基础油门，`I26` 为本拍学习步长，`I27` 为学习门控，`I28` 为学习前速度环输出。\n\n";

    for (const FileDef &f : files) {
        std::vector<Row> rows = load(f.path);
        std::vector<double> h_abs, vz_abs, pos_abs, vel_abs, thr, step_abs, raw_vel_abs;
        std::vector<Segment> segs;
        Segment seg;
        int flight_rows = 0, stable_rows = 0, gate_rows = 0;
        int pull_rows = 0, pull_gate_rows = 0;
        double hover_start = 0.0, hover_end = 0.0, pull_h_min = 1e9, pull_hover_first = 0.0, pull_hover_last = 0.0;
        bool hover_seen = false, pull_seen = false;
        std::vector<double> pull_h_abs, pull_vz_abs;

        for (size_t ri = 0; ri < rows.size(); ++ri) {
            const Row &r = rows[ri];
            if (f.learning && r.n >= 29) {
                if (!hover_seen && r.v[25] > 0.0) { hover_start = r.v[25]; hover_seen = true; }
                hover_end = r.v[25];
            }
            if (!flying(r)) continue;
            ++flight_rows;
            h_abs.push_back(std::fabs(r.v[15] - 1000.0));
            vz_abs.push_back(std::fabs(r.v[14]));
            pos_abs.push_back(std::fabs(r.v[16]));
            vel_abs.push_back(std::fabs(r.v[20]));
            thr.push_back(r.v[24]);
            if (stable(r)) ++stable_rows;
            if (f.learning && r.n >= 29) {
                step_abs.push_back(std::fabs(r.v[26]));
                raw_vel_abs.push_back(std::fabs(r.v[28]));
                if (r.v[27] > 0.5) ++gate_rows;
            }
            bool evt = pull_event(r);
            if (evt) {
                ++pull_rows;
                pull_h_min = std::min(pull_h_min, r.v[15]);
                pull_h_abs.push_back(std::fabs(r.v[15] - 1000.0));
                pull_vz_abs.push_back(std::fabs(r.v[14]));
                if (f.learning && r.n >= 29) {
                    if (!pull_seen) { pull_hover_first = r.v[25]; pull_seen = true; }
                    pull_hover_last = r.v[25];
                    if (r.v[27] > 0.5) ++pull_gate_rows;
                }
            }
            if (evt && seg.start == 0) {
                seg = Segment();
                seg.start = (int)ri;
                seg.hover0 = (f.learning && r.n >= 29) ? r.v[25] : 0.0;
            }
            if (evt) {
                seg.end = (int)ri;
                seg.min_h = std::min(seg.min_h, r.v[15]);
                seg.max_vz = std::max(seg.max_vz, std::fabs(r.v[14]));
                seg.max_pos = std::max(seg.max_pos, std::fabs(r.v[16]));
                if (f.learning && r.n >= 29) {
                    seg.hover1 = r.v[25];
                    if (r.v[27] > 0.5) ++seg.gate;
                }
            } else if (seg.start != 0) {
                if ((seg.end - seg.start) > 20) segs.push_back(seg);
                seg = Segment();
            }
        }
        if (seg.start != 0 && (seg.end - seg.start) > 20) segs.push_back(seg);

        Stats hs = stats(h_abs), vs = stats(vz_abs), ps = stats(pos_abs), vos = stats(vel_abs), ts = stats(thr);
        Stats ss = stats(step_abs), rvs = stats(raw_vel_abs), phs = stats(pull_h_abs), pvs = stats(pull_vz_abs);
        double duration = rows.size() > 1 ? (rows.back().v[0] - rows.front().v[0]) * 0.001 : 0.0;
        double stable_pct = flight_rows ? 100.0 * stable_rows / flight_rows : 0.0;
        double gate_pct = flight_rows ? 100.0 * gate_rows / flight_rows : 0.0;

        csv << f.name << "," << rows.size() << "," << duration << "," << flight_rows << "," << stable_rows << "," << stable_pct << ","
            << hs.mean << "," << hs.p95 << "," << vs.p95 << "," << ps.p95 << "," << vos.p95 << "," << ts.mean << "," << ts.p95 << ","
            << hover_start << "," << hover_end << "," << (hover_end - hover_start) << "," << gate_pct << "," << ss.p95 << "," << rvs.p95 << ","
            << pull_rows << "," << (pull_rows ? pull_h_min : 0.0) << "," << phs.p95 << "," << pvs.p95 << "," << pull_gate_rows << "," << (pull_hover_last - pull_hover_first) << "\n";

        md << "## " << f.name << "\n\n";
        md << "- 飞行段 " << flight_rows << " 行，稳态门控 " << stable_rows << " 行，占 " << stable_pct << "%。\n";
        md << "- 高度绝对误差 mean " << hs.mean << " mm，P95 " << hs.p95 << " mm；`|vz|` P95 " << vs.p95 << " m/s。\n";
        md << "- 速度环输出 `|I20|` P95 " << vos.p95 << " PWM；总油门 mean " << ts.mean << "，P95 " << ts.p95 << "。\n";
        if (f.learning) {
            md << "- 学习基础油门 " << hover_start << " -> " << hover_end << "，增量 " << (hover_end - hover_start)
               << "；门控占飞行段 " << gate_pct << "%；学习步长 P95 " << ss.p95 << " PWM/帧。\n";
            md << "- 学习前速度环输出 `|I28|` P95 " << rvs.p95 << " PWM。\n";
        }
        if (f.pull || pull_rows > 0) {
            md << "- 拉扯/冲击候选 " << pull_rows << " 行，最低高度 " << (pull_rows ? pull_h_min : 0.0)
               << " mm，高度误差 P95 " << phs.p95 << " mm，`|vz|` P95 " << pvs.p95 << " m/s。\n";
            if (f.learning) {
                md << "- 冲击候选中门控仍打开 " << pull_gate_rows << " 行，冲击期基础油门变化 "
                   << (pull_hover_last - pull_hover_first) << " PWM。\n";
            }
        }
        if (!segs.empty()) {
            std::sort(segs.begin(), segs.end(), [](const Segment &a, const Segment &b) {
                return a.min_h < b.min_h;
            });
            md << "- 最严重冲击段 TOP3：\n";
            for (size_t i = 0; i < segs.size() && i < 3; ++i) {
                double t0 = rows[segs[i].start].v[0] * 0.001;
                double t1 = rows[segs[i].end].v[0] * 0.001;
                md << "  - " << t0 << ".." << t1 << " s，最低高度 " << segs[i].min_h
                   << " mm，最大 |vz| " << segs[i].max_vz << " m/s，最大 |pos_out| "
                   << segs[i].max_pos << "，门控帧 " << segs[i].gate
                   << "，段内学习变化 " << (segs[i].hover1 - segs[i].hover0) << " PWM。\n";
            }
        }
        md << "\n";
    }
    return 0;
}
