# PixToDistance 强增益分段设计

## 范围

只修改 `project/code/Planner/pix_to_distance.c` 中的
`PixToDistance_CalcStrongGain`。不修改调用方式、输出限幅或其他距离计算逻辑。

## 增益曲线

沿用现有 `PixToDistance_SmoothStep01` 平滑插值：

- `pixel_abs <= 30`：增益为 `1.0`。
- `30 < pixel_abs < 50`：从 `1.0` 平滑增加到 `1.3`。
- `50 <= pixel_abs < 60`：从 `1.3` 平滑增加到 `1.8`。
- `60 <= pixel_abs < 70`：从 `1.8` 平滑增加到 `2.0`。
- `pixel_abs >= 70`：增益封顶为 `2.0`。

各边界连续，不允许在 50、60 或 70 像素处出现增益跳变。

## 验证

修改后检查以下输入输出：

- `30 -> 1.0`
- `40 -> 1.15`
- `50 -> 1.3`
- `55 -> 1.55`
- `60 -> 1.8`
- `65 -> 1.9`
- `70 -> 2.0`
- 大于 `70 -> 2.0`

同时确认除 `PixToDistance_CalcStrongGain` 外没有业务代码变化。
