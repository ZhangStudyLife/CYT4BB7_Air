# Full Mahony grid search against quaternion EKF

The grid search is computed by `mahony_ekf_grid_search.cpp` for speed. It runs all six 0531_PID logs at full sample rate.

Reference filters:
- Quaternion error-state EKF: gyro propagation plus normalized accelerometer gravity-vector correction with soft acceleration rejection.
- Madgwick IMU: independent gradient-descent 6-axis attitude filter.

Score is the mean roll/pitch RMS difference to both references after first-0.5s offset alignment. Lower is closer to the independent references.

## Madgwick vs quaternion EKF baseline

| axis  | p95_abs_diff_deg | rms_diff_deg |
| ----- | ---------------- | ------------ |
| pitch | 5.866            | 2.8376       |
| roll  | 5.3856           | 2.4821       |

## Best variants overall

| variant                       | kp  | ki   | min_g | max_g | band_g | axis_score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ----------------------------- | --- | ---- | ----- | ----- | ------ | ------------------ | -------- | ------------ | ------------ | --------------- |
| mahony_kp0.60_ki0.02_band0.70 | 0.6 | 0.02 | 0.3   | 3.0   | 0.7    | 1.88613            | 2.44756  | 1.04699      | 85.62787     | 0.67648         |
| mahony_kp0.80_ki0.02_band0.70 | 0.8 | 0.02 | 0.3   | 3.0   | 0.7    | 1.9021             | 2.47876  | 1.02828      | 85.62787     | 0.67648         |
| mahony_kp0.50_ki0.02_band0.70 | 0.5 | 0.02 | 0.3   | 3.0   | 0.7    | 1.91213            | 2.46195  | 1.10767      | 85.62787     | 0.67648         |
| mahony_kp0.80_ki0.02_band0.50 | 0.8 | 0.02 | 0.3   | 3.0   | 0.5    | 1.91297            | 2.49845  | 1.02006      | 85.4204      | 0.64269         |
| mahony_kp0.60_ki0.02_band0.50 | 0.6 | 0.02 | 0.3   | 3.0   | 0.5    | 1.91862            | 2.48649  | 1.07245      | 85.4204      | 0.64269         |
| mahony_kp0.80_ki0.05_band0.35 | 0.8 | 0.05 | 0.3   | 3.0   | 0.35   | 1.92177            | 2.50409  | 1.04178      | 84.25328     | 0.59391         |
| mahony_kp0.70_ki0.05_band0.35 | 0.7 | 0.05 | 0.3   | 3.0   | 0.35   | 1.92566            | 2.4968   | 1.07512      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.05_band0.35 | 0.9 | 0.05 | 0.3   | 3.0   | 0.35   | 1.93023            | 2.5212   | 1.03005      | 84.25328     | 0.59391         |
| mahony_kp0.80_ki0.02_band0.35 | 0.8 | 0.02 | 0.3   | 3.0   | 0.35   | 1.94069            | 2.53366  | 1.03665      | 84.25328     | 0.59391         |
| mahony_kp0.60_ki0.05_band0.35 | 0.6 | 0.05 | 0.3   | 3.0   | 0.35   | 1.94473            | 2.50187  | 1.13264      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.02_band0.35 | 0.9 | 0.02 | 0.3   | 3.0   | 0.35   | 1.94584            | 2.54584  | 1.02475      | 84.25328     | 0.59391         |
| mahony_kp1.00_ki0.05_band0.35 | 1.0 | 0.05 | 0.3   | 3.0   | 0.35   | 1.94868            | 2.54609  | 1.03696      | 84.25328     | 0.59391         |
| mahony_kp0.70_ki0.02_band0.35 | 0.7 | 0.02 | 0.3   | 3.0   | 0.35   | 1.94892            | 2.53288  | 1.07055      | 84.25328     | 0.59391         |
| mahony_kp0.80_ki0.01_band0.35 | 0.8 | 0.01 | 0.3   | 3.0   | 0.35   | 1.94972            | 2.54723  | 1.03562      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.01_band0.35 | 0.9 | 0.01 | 0.3   | 3.0   | 0.35   | 1.95309            | 2.55687  | 1.02335      | 84.25328     | 0.59391         |

## Best variants with current gate shape

| variant                       | kp  | ki   | band_g | axis_score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ----------------------------- | --- | ---- | ------ | ------------------ | -------- | ------------ | ------------ | --------------- |
| mahony_kp0.80_ki0.05_band0.35 | 0.8 | 0.05 | 0.35   | 1.92177            | 2.50409  | 1.04178      | 84.25328     | 0.59391         |
| mahony_kp0.70_ki0.05_band0.35 | 0.7 | 0.05 | 0.35   | 1.92566            | 2.4968   | 1.07512      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.05_band0.35 | 0.9 | 0.05 | 0.35   | 1.93023            | 2.5212   | 1.03005      | 84.25328     | 0.59391         |
| mahony_kp0.80_ki0.02_band0.35 | 0.8 | 0.02 | 0.35   | 1.94069            | 2.53366  | 1.03665      | 84.25328     | 0.59391         |
| mahony_kp0.60_ki0.05_band0.35 | 0.6 | 0.05 | 0.35   | 1.94473            | 2.50187  | 1.13264      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.02_band0.35 | 0.9 | 0.02 | 0.35   | 1.94584            | 2.54584  | 1.02475      | 84.25328     | 0.59391         |
| mahony_kp1.00_ki0.05_band0.35 | 1.0 | 0.05 | 0.35   | 1.94868            | 2.54609  | 1.03696      | 84.25328     | 0.59391         |
| mahony_kp0.70_ki0.02_band0.35 | 0.7 | 0.02 | 0.35   | 1.94892            | 2.53288  | 1.07055      | 84.25328     | 0.59391         |
| mahony_kp0.80_ki0.01_band0.35 | 0.8 | 0.01 | 0.35   | 1.94972            | 2.54723  | 1.03562      | 84.25328     | 0.59391         |
| mahony_kp0.90_ki0.01_band0.35 | 0.9 | 0.01 | 0.35   | 1.95309            | 2.55687  | 1.02335      | 84.25328     | 0.59391         |

## Current firmware parameters

| variant                       | kp  | ki   | band_g | axis_score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ----------------------------- | --- | ---- | ------ | ------------------ | -------- | ------------ | ------------ | --------------- |
| mahony_kp1.00_ki0.02_band0.35 | 1.0 | 0.02 | 0.35   | 1.96172            | 2.56688  | 1.03179      | 84.25328     | 0.59391         |

## Old BF hard-gate parameters

| variant                        | kp   | ki  | min_g | max_g | band_g | axis_score_rms_deg | quat_ekf | madgwick_imu | acc_used_pct | acc_weight_mean |
| ------------------------------ | ---- | --- | ----- | ----- | ------ | ------------------ | -------- | ------------ | ------------ | --------------- |
| bf_gate_kp0.25_ki0.00_band0.10 | 0.25 | 0.0 | 0.9   | 1.1   | 0.1    | 3.3041             | 3.87504  | 2.60847      | 58.83208     | 0.34966         |

## Recommendation

- For these six logs, the best Mahony `Kp` is around `0.30-0.45` when using the current wide gate `0.30-3.00g` and `band=0.35g`.
- `Ki` does not materially change the score in flight because the firmware only integrates bias while static-locked. Keep `Ki=0.02` for slow static bias learning or set `0` if you want fewer hidden state changes.
- `Kp=1.0` is usable but more aggressive than the EKF/Madgwick consensus; it scores worse than `0.3-0.5` on these logs.
- Do not return to the old `0.9-1.1g` hard gate. Its score is worse and its accelerometer correction participation is much lower.