# Offline 6-axis attitude estimator comparison

Input: I7-I12 filtered gyro/accelerometer channels from the six 0531_PID CSV logs.
Reference: I13/I14 online roll/pitch. This is only the firmware output, not ground truth.
All offline estimator curves are offset-aligned to the first 0.5 s of the online roll/pitch before error statistics.

## Accelerometer gate diagnostics

| flight | samples | duration_s | acc_mag_mean_g | acc_mag_p05_g | acc_mag_p95_g | repo_acc_used_pct | soft_acc_used_pct | repo_longest_no_acc_ms | soft_longest_no_acc_ms |
| ------ | ------- | ---------- | -------------- | ------------- | ------------- | ----------------- | ----------------- | ---------------------- | ---------------------- |
| 1      | 70182   | 102.4      | 1.016          | 0.809         | 1.282         | 55.389            | 80.521            | 1284.0                 | 1242.0                 |
| 2      | 99694   | 137.208    | 1.012          | 0.836         | 1.225         | 56.223            | 81.528            | 735.0                  | 629.0                  |
| 3      | 154324  | 211.456    | 1.015          | 0.823         | 1.249         | 49.493            | 75.204            | 6616.0                 | 6616.0                 |
| 4      | 122856  | 167.748    | 1.009          | 0.838         | 1.214         | 63.533            | 89.369            | 608.0                  | 505.0                  |
| 5      | 117124  | 155.146    | 1.014          | 0.827         | 1.251         | 58.397            | 86.147            | 562.0                  | 546.0                  |
| 6      | 94143   | 128.688    | 1.01           | 0.836         | 1.213         | 69.957            | 92.75             | 883.0                  | 801.0                  |

## Mean error vs online Euler angle across six flights

| estimator        | axis  | final_vs_online_deg | p95_abs_vs_online_deg | rms_vs_online_deg |
| ---------------- | ----- | ------------------- | --------------------- | ----------------- |
| complementary_rp | pitch | 0.103               | 9.378                 | 4.259             |
| complementary_rp | roll  | -2.202              | 10.679                | 4.845             |
| gyro_only        | pitch | -22.749             | 22.548                | 13.269            |
| gyro_only        | roll  | 2.958               | 6.888                 | 3.797             |
| madgwick_imu     | pitch | 0.112               | 4.899                 | 2.485             |
| madgwick_imu     | roll  | -2.231              | 4.91                  | 2.826             |
| repo_mahony      | pitch | -0.462              | 3.315                 | 1.78              |
| repo_mahony      | roll  | -1.766              | 3.41                  | 2.131             |
| soft_mahony      | pitch | 0.139               | 4.959                 | 2.476             |
| soft_mahony      | roll  | -2.138              | 4.975                 | 2.747             |

## Mean disagreement vs estimator median across six flights

| estimator        | axis  | p95_abs_vs_algo_median_deg | rms_vs_algo_median_deg |
| ---------------- | ----- | -------------------------- | ---------------------- |
| complementary_rp | pitch | 6.728                      | 2.761                  |
| complementary_rp | roll  | 7.729                      | 3.302                  |
| gyro_only        | pitch | 23.941                     | 13.973                 |
| gyro_only        | roll  | 8.76                       | 4.697                  |
| madgwick_imu     | pitch | 2.877                      | 1.254                  |
| madgwick_imu     | roll  | 1.782                      | 1.058                  |
| repo_mahony      | pitch | 5.961                      | 2.734                  |
| repo_mahony      | roll  | 3.264                      | 1.459                  |
| soft_mahony      | pitch | 3.142                      | 1.298                  |
| soft_mahony      | roll  | 1.799                      | 0.762                  |

## Pairwise differences among the three fused quaternion estimators

| left        | right        | axis  | p95_abs_diff_deg | rms_diff_deg |
| ----------- | ------------ | ----- | ---------------- | ------------ |
| repo_mahony | madgwick_imu | pitch | 6.421            | 3.239        |
| repo_mahony | madgwick_imu | roll  | 3.974            | 1.97         |
| repo_mahony | soft_mahony  | pitch | 6.527            | 3.249        |
| repo_mahony | soft_mahony  | roll  | 3.729            | 1.734        |
| soft_mahony | madgwick_imu | pitch | 1.665            | 0.895        |
| soft_mahony | madgwick_imu | roll  | 2.168            | 1.18         |

## Findings from these logs

- The current repo Mahony gate uses accelerometer correction on average 58.8% of samples; the soft Mahony variant uses it on 84.3% of samples.
- On pitch, repo Mahony differs from soft Mahony by RMS 3.249 deg and P95 6.527 deg.
- On pitch, soft Mahony differs from Madgwick IMU by RMS 0.895 deg and P95 1.665 deg.
- This supports the hypothesis that the hard 0.9g-1.1g gate is too restrictive for these flights; it makes the estimator depend on gyro integration during too much of the flight segment.
- The online Euler angle is produced by the current firmware, so matching it is not proof of correctness. Agreement between independent offline filters is the stronger signal here.

## Algorithm references

- Mahony, Hamel, Pflimlin, Nonlinear Complementary Filters on SO(3): https://doi.org/10.1109/TAC.2008.923738
- Madgwick IMU gradient-descent formulation: https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
- x-io Fusion acceleration rejection/recovery notes: https://github.com/xioTechnologies/Fusion

## Files

- `estimator_vs_online_summary.csv`: per-flight, per-axis error against online Euler.
- `estimator_disagreement_summary.csv`: per-flight disagreement against the estimator median.
- `trusted_estimator_pairwise_summary.csv`: pairwise differences between repo Mahony, soft Mahony, and Madgwick IMU.
- `accel_gate_summary.csv`: acceleration magnitude and correction gate statistics.
- `flightN_estimators.png`: aligned roll/pitch curves plus acceleration gate traces.
- `flightN_offline_angles.csv`: compact aligned angle traces for external plotting.

## Interpretation rules

- If `repo_mahony` diverges from most other algorithms while its accel weight is near zero for long windows, suspect the hard acceleration/rotation gate.
- If all algorithms diverge in the same direction, suspect input axes, calibration, filter delay, or real vehicle attitude rather than only the Mahony correction gate.
- With 6-axis data, yaw is not observable; this report intentionally limits conclusions to roll and pitch.