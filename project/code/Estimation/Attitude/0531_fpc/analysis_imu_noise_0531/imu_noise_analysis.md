# IMU noise analysis 2026-05-31

Source: `D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT4BB7_Air\project\code\Estimation\Attitude\0531_fpc\第二次飞行,1000hz.csv`
Rows: 223970, timestamp span: 318.2s
Diff counts: `{'0.0': 1706, '1.0': 193820, '2.0': 548, '3.0': 25162, '4.0': 2731, '9.0': 1, '3003.0': 2}`

## Gyro PSD band RMS
| axis | 0-30Hz dps | 30-80Hz dps | 80-250Hz dps | 250-500Hz dps | top peaks Hz |
| --- | --- | --- | --- | --- | --- |
| gyro_x | 26.160 | 1.889 | 3.989 | 2.286 | 184.8, 184.0, 186.7, 187.8 |
| gyro_y | 15.968 | 3.799 | 5.757 | 3.778 | 184.0, 185.1, 183.2, 45.2 |
| gyro_z | 4.769 | 1.116 | 6.275 | 1.981 | 184.8, 184.0, 180.8, 183.2 |

## Accel PSD band RMS
| axis | 0-30Hz g | 30-80Hz g | 80-250Hz g | 250-500Hz g | top peaks Hz |
| --- | --- | --- | --- | --- | --- |
| acc_x | 0.0668 | 0.0353 | 0.1870 | 0.0863 | 189.9, 193.4, 187.8, 190.7 |
| acc_y | 0.0869 | 0.0453 | 0.2572 | 0.1032 | 189.9, 187.8, 184.8, 189.1 |
| acc_z | 0.1262 | 0.0827 | 0.3189 | 0.1963 | 195.0, 183.2, 191.0, 181.1 |

## Gyro filter simulation
Old filter is AA LPF 250Hz + notch 160Hz/BW65 + gyro LPF 60Hz. New filter is AA LPF 250Hz + notch 185Hz/BW70 + gyro LPF 50Hz.
| axis | raw 80-250 | old 80-250 | new 80-250 | raw 250-500 | old 250-500 | new 250-500 |
| --- | --- | --- | --- | --- | --- | --- |
| gyro_x | 3.989 | 0.734 | 0.539 | 2.286 | 0.016 | 0.011 |
| gyro_y | 5.757 | 1.152 | 0.863 | 3.778 | 0.034 | 0.023 |
| gyro_z | 6.275 | 1.028 | 0.738 | 1.981 | 0.018 | 0.013 |

## Quiet windows for temporary mechanical trim
| t_end_s | score | roll mean | pitch mean | roll target | pitch target | flow median |
| --- | --- | --- | --- | --- | --- | --- |
| 58.85 | 0.21 | 2.38 | 2.15 | -3.00 | 3.50 | 5.1 |
| 58.85 | 0.21 | 2.38 | 2.15 | -3.00 | 3.50 | 5.1 |
| 58.85 | 0.21 | 2.38 | 2.15 | -3.00 | 3.50 | 5.1 |
| 58.85 | 0.21 | 2.38 | 2.15 | -3.00 | 3.50 | 5.1 |
| 58.85 | 0.21 | 2.38 | 2.15 | -3.00 | 3.50 | 5.1 |

## Control metrics
| loop | mean err | MAE | RMSE | p95 abs |
| --- | --- | --- | --- | --- |
| roll | -1.90 | 3.18 | 3.95 | 5.52 |
| pitch | 0.58 | 1.46 | 2.03 | 4.37 |
| yaw | -0.01 | 2.13 | 3.32 | 7.26 |
| gyro_roll | 0.05 | 9.34 | 13.03 | 26.46 |
| gyro_pitch | 1.68 | 9.81 | 12.45 | 23.89 |
| gyro_yaw | 0.00 | 8.70 | 10.67 | 20.13 |

## Lag metrics
| pair | best lag ms | corr |
| --- | --- | --- |
| roll_angle | 108 | 0.890 |
| pitch_angle | 100 | 0.973 |
| roll_gyro | 46 | 0.862 |
| pitch_gyro | 30 | 0.787 |
| yaw_gyro | 300 | 0.253 |

Plots: `gyro_psd.png`, `accel_psd.png`, `quiet_window_attitude.png`
