# Mahony parameter sweep vs Madgwick and roll/pitch EKF

This report uses the same six 0531_PID logs and the filtered I7-I12 6-axis data.
All curves are offset-aligned to the first 0.5 s before statistics. Online Euler is a firmware reference, not ground truth.

## Pairwise mean differences across six flights

| left               | right              | axis  | final_diff_deg | p95_abs_diff_deg | rms_diff_deg |
| ------------------ | ------------------ | ----- | -------------- | ---------------- | ------------ |
| bf_gate_kp025_ki0  | current_kp10_ki002 | pitch | -0.601         | 6.527            | 3.249        |
| bf_gate_kp025_ki0  | current_kp10_ki002 | roll  | 0.372          | 3.729            | 1.734        |
| current_kp06_ki002 | current_kp10_ki002 | pitch | 0.077          | 1.342            | 0.625        |
| current_kp06_ki002 | current_kp10_ki002 | roll  | 0.09           | 1.275            | 0.577        |
| current_kp10_ki0   | current_kp10_ki002 | pitch | -0.044         | 0.101            | 0.061        |
| current_kp10_ki0   | current_kp10_ki002 | roll  | 0.019          | 0.034            | 0.02         |
| current_kp10_ki002 | ekf_rp             | pitch | 0.004          | 2.441            | 1.172        |
| current_kp10_ki002 | ekf_rp             | roll  | -0.264         | 2.891            | 1.291        |
| current_kp10_ki002 | madgwick_imu       | pitch | 0.027          | 1.665            | 0.895        |
| current_kp10_ki002 | madgwick_imu       | roll  | 0.093          | 2.168            | 1.18         |
| current_kp15_ki002 | current_kp10_ki002 | pitch | -0.023         | 1.259            | 0.575        |
| current_kp15_ki002 | current_kp10_ki002 | roll  | -0.046         | 1.277            | 0.579        |
| madgwick_imu       | ekf_rp             | pitch | -0.023         | 2.178            | 1.218        |
| madgwick_imu       | ekf_rp             | roll  | -0.357         | 2.314            | 1.34         |
| wide_kp10_band020  | current_kp10_ki002 | pitch | 0.003          | 1.144            | 0.484        |
| wide_kp10_band020  | current_kp10_ki002 | roll  | -0.006         | 1.029            | 0.448        |
| wide_kp10_band050  | current_kp10_ki002 | pitch | -0.001         | 0.533            | 0.227        |
| wide_kp10_band050  | current_kp10_ki002 | roll  | 0.002          | 0.537            | 0.228        |

## Mean difference vs online Euler across six flights

| estimator          | axis  | final_vs_online_deg | p95_abs_vs_online_deg | rms_vs_online_deg |
| ------------------ | ----- | ------------------- | --------------------- | ----------------- |
| bf_gate_kp025_ki0  | pitch | -0.462              | 3.315                 | 1.78              |
| bf_gate_kp025_ki0  | roll  | -1.766              | 3.41                  | 2.131             |
| current_kp06_ki002 | pitch | 0.216               | 4.339                 | 2.164             |
| current_kp06_ki002 | roll  | -2.048              | 4.099                 | 2.471             |
| current_kp10_ki0   | pitch | 0.095               | 4.914                 | 2.454             |
| current_kp10_ki0   | roll  | -2.119              | 4.968                 | 2.738             |
| current_kp10_ki002 | pitch | 0.139               | 4.959                 | 2.476             |
| current_kp10_ki002 | roll  | -2.138              | 4.975                 | 2.747             |
| current_kp15_ki002 | pitch | 0.116               | 5.669                 | 2.795             |
| current_kp15_ki002 | roll  | -2.184              | 5.951                 | 3.081             |
| ekf_rp             | pitch | 0.135               | 4.348                 | 2.204             |
| ekf_rp             | roll  | -1.874              | 4.881                 | 2.808             |
| madgwick_imu       | pitch | 0.112               | 4.899                 | 2.485             |
| madgwick_imu       | roll  | -2.231              | 4.91                  | 2.826             |
| mid_gate_kp10      | pitch | 0.139               | 4.955                 | 2.475             |
| mid_gate_kp10      | roll  | -2.138              | 4.976                 | 2.747             |
| wide_kp10_band020  | pitch | 0.141               | 4.457                 | 2.224             |
| wide_kp10_band020  | roll  | -2.144              | 4.328                 | 2.533             |
| wide_kp10_band050  | pitch | 0.138               | 5.233                 | 2.597             |
| wide_kp10_band050  | roll  | -2.136              | 5.321                 | 2.875             |

## Mahony accelerometer usage by variant

| variant            | kp   | ki   | accel_min_g | accel_max_g | trust_band_g | acc_used_pct | acc_weight_mean |
| ------------------ | ---- | ---- | ----------- | ----------- | ------------ | ------------ | --------------- |
| bf_gate_kp025_ki0  | 0.25 | 0.0  | 0.9         | 1.1         | 0.1          | 58.832       | 0.35            |
| current_kp06_ki002 | 0.6  | 0.02 | 0.3         | 3.0         | 0.35         | 84.253       | 0.594           |
| current_kp10_ki0   | 1.0  | 0.0  | 0.3         | 3.0         | 0.35         | 84.253       | 0.594           |
| current_kp10_ki002 | 1.0  | 0.02 | 0.3         | 3.0         | 0.35         | 84.253       | 0.594           |
| current_kp15_ki002 | 1.5  | 0.02 | 0.3         | 3.0         | 0.35         | 84.253       | 0.594           |
| mid_gate_kp10      | 1.0  | 0.02 | 0.7         | 1.5         | 0.35         | 84.038       | 0.594           |
| wide_kp10_band020  | 1.0  | 0.02 | 0.3         | 3.0         | 0.2          | 77.666       | 0.491           |
| wide_kp10_band050  | 1.0  | 0.02 | 0.3         | 3.0         | 0.5          | 85.42        | 0.643           |

## Parameter interpretation

- `Kp` controls how strongly the gravity-vector error corrects gyro integration. Higher `Kp` reduces long gyro-only drift faster, but follows acceleration-induced false gravity more aggressively.
- `Ki` only matters in this firmware when the static detector is locked. It slowly learns roll/pitch gyro bias; it has little effect during active flight in these logs.
- The `0.9g-1.1g` hard gate removes many flight samples from correction. It can make the angle look smooth while the estimate is mostly gyro-integrated.
- Wider accel magnitude limits plus a trust band do not mean blindly trusting acceleration; the trust band still reduces correction when `|acc|-1g` grows.
- Madgwick and the simple roll/pitch EKF are independent references here. They are not ground truth, but agreement between them is a useful warning when Mahony differs strongly.

## Files

- `pairwise_summary.csv`: per-flight pairwise differences.
- `vs_online_summary.csv`: per-flight differences against online Euler.
- `mahony_variant_gate_summary.csv`: accelerometer correction usage by Mahony variant.
- `flightN_param_sweep.png`: plotted curves for each flight.