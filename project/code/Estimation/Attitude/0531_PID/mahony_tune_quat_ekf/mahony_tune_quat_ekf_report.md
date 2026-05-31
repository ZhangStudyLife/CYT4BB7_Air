# Mahony Kp/Ki tuning against quaternion error-state EKF

Reference EKF: 3D quaternion error-state EKF using gyro propagation and normalized accelerometer gravity-vector updates with soft acceleration-magnitude rejection.
This is appropriate for 6-axis roll/pitch comparison; yaw remains unobservable without magnetometer or other heading information.

Score: mean roll/pitch RMS difference to both quaternion EKF and Madgwick IMU after first-0.5s offset alignment. Lower is closer to the independent references.

## Madgwick vs quaternion EKF sanity check

| axis  | p95_abs_diff_deg | rms_diff_deg |
| ----- | ---------------- | ------------ |
| pitch | 5.981            | 2.903        |
| roll  | 5.459            | 2.521        |

## Best Mahony variants

| variant               | kp  | ki   | trust_band_g | score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| --------------------- | --- | ---- | ------------ | ------------- | -------- | ------------ | ------------ | --------------- |
| mahony_kp0p6_band0p50 | 0.6 | 0.02 | 0.5          | 1.9681        | 2.5623   | 1.0759       | 85.4204      | 0.6427          |
| mahony_kp0p8_ki0p02   | 0.8 | 0.02 | 0.35         | 1.9895        | 2.6079   | 1.0407       | 84.2533      | 0.5939          |
| mahony_kp1p0_ki0p02   | 1.0 | 0.02 | 0.35         | 2.0095        | 2.6388   | 1.0373       | 84.2533      | 0.5939          |
| mahony_kp0p8_ki0p00   | 0.8 | 0.0  | 0.35         | 2.0098        | 2.6374   | 1.0414       | 84.2533      | 0.5939          |
| mahony_kp0p6_ki0p02   | 0.6 | 0.02 | 0.35         | 2.0223        | 2.6222   | 1.131        | 84.2533      | 0.5939          |
| mahony_kp1p0_ki0p00   | 1.0 | 0.0  | 0.35         | 2.0225        | 2.6584   | 1.0359       | 84.2533      | 0.5939          |
| mahony_kp0p6_ki0p00   | 0.6 | 0.0  | 0.35         | 2.0569        | 2.6711   | 1.138        | 84.2533      | 0.5939          |
| mahony_kp1p2_ki0p02   | 1.2 | 0.02 | 0.35         | 2.0628        | 2.6972   | 1.0942       | 84.2533      | 0.5939          |
| mahony_kp0p5_ki0p02   | 0.5 | 0.02 | 0.35         | 2.0676        | 2.6552   | 1.2166       | 84.2533      | 0.5939          |
| mahony_kp1p2_ki0p00   | 1.2 | 0.0  | 0.35         | 2.0717        | 2.711    | 1.0922       | 84.2533      | 0.5939          |
| mahony_kp0p5_ki0p00   | 0.5 | 0.0  | 0.35         | 2.1152        | 2.7215   | 1.2304       | 84.2533      | 0.5939          |
| mahony_kp0p4_ki0p02   | 0.4 | 0.02 | 0.35         | 2.1404        | 2.7136   | 1.3359       | 84.2533      | 0.5939          |

## Current firmware parameter score

| variant             | kp  | ki   | trust_band_g | score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ------------------- | --- | ---- | ------------ | ------------- | -------- | ------------ | ------------ | --------------- |
| mahony_kp1p0_ki0p02 | 1.0 | 0.02 | 0.35         | 2.0095        | 2.6388   | 1.0373       | 84.2533      | 0.5939          |

## BF hard-gate parameter score

| variant            | kp   | ki  | trust_band_g | score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ------------------ | ---- | --- | ------------ | ------------- | -------- | ------------ | ------------ | --------------- |
| bf_gate_kp0p25_ki0 | 0.25 | 0.0 | 0.1          | 3.3332        | 3.9279   | 2.6047       | 58.8321      | 0.3497          |

## Practical reading

- Best-fit Kp for these six logs is around 0.4-0.6 with the wide 0.30-3.00g gate and 0.35 trust band.
- Kp=1.0 is still close to the EKF/Madgwick references, but it is more aggressive and slightly farther in this score.
- Ki has almost no effect in these flight logs because firmware Ki only learns bias when the static detector is locked.
- The old BF hard gate scores poorly mainly because correction is unavailable for too much of the flight.

## Files

- `mahony_kpki_score.csv`: aggregate ranking.
- `mahony_vs_quat_ekf_pairwise.csv`: per-flight pairwise metrics.
- `mahony_tune_gate_summary.csv`: accelerometer correction usage.
- `kp_score_curve.png`: Kp sweep curve.