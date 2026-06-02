#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#define HOVER_INIT 2800.0
#define HOVER_TC 8.0
#define HOVER_MIN (HOVER_INIT - 300.0)
#define HOVER_MAX (HOVER_INIT + 900.0)
#define VZ_MAX 0.18
#define POS_MAX 0.08
#define TILT_MAX 7.0
#define ACC_MAX 0.20
#define TOF_SPREAD_MAX 250.0
#define RATE_MAX 150.0

static bool parse_line(const std::string &line, double v[25])
{
    const char *p = line.c_str();
    char *end = 0;
    for (int i = 0; i < 25; ++i) {
        v[i] = std::strtod(p, &end);
        if (end == p) {
            return false;
        }
        p = (*end == ',') ? end + 1 : end;
    }
    return true;
}

int main()
{
    FILE *in = _wfopen(L"无学习油门,长时间飞行对照组.csv", L"rb");
    FILE *out = _wfopen(L"无学习油门,长时间飞行对照组_追加离线基础油门.csv", L"wb");
    if (in == 0 || out == 0) {
        return 1;
    }

    char buf[8192];
    double hover = HOVER_INIT;
    double last_t = 0.0;
    bool header = true;

    while (std::fgets(buf, sizeof(buf), in) != 0) {
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
            line.pop_back();
        }

        if (header) {
            std::fprintf(out, "%s,offline_hover_throttle\n", line.c_str());
            header = false;
            continue;
        }

        double v[25];
        if (parse_line(line, v)) {
            double dt = (last_t > 0.0) ? (v[0] - last_t) * 0.001 : 0.001;
            double tof_min = std::min(std::min(v[10], v[11]), std::min(v[12], v[13]));
            double tof_max = std::max(std::max(v[10], v[11]), std::max(v[12], v[13]));
            double step = (dt / (dt + HOVER_TC)) * v[20];
            if (dt < 0.0005 || dt > 0.05) {
                dt = 0.001;
            }
            if ((v[24] > 2200.0) && (v[15] > 400.0) && (v[15] < 1800.0) &&
                (std::fabs(v[14]) < VZ_MAX) && (std::fabs(v[16]) < POS_MAX) &&
                (std::fabs(v[7]) < TILT_MAX) && (std::fabs(v[8]) < TILT_MAX) &&
                (std::fabs(std::sqrt(v[4] * v[4] + v[5] * v[5] + v[6] * v[6]) - 1.0) < ACC_MAX) &&
                ((tof_max - tof_min) < TOF_SPREAD_MAX)) {
                hover += std::max(-RATE_MAX * dt, std::min(RATE_MAX * dt, step));
                hover = std::max(HOVER_MIN, std::min(HOVER_MAX, hover));
            }
            last_t = v[0];
        }
        std::fprintf(out, "%s,%.6f\n", line.c_str(), hover);
    }

    std::fclose(in);
    std::fclose(out);
    return 0;
}
