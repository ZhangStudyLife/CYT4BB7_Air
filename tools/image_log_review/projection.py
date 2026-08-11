"""上位机绘图使用的固定三摄车灯投影公式。"""

from __future__ import annotations

import math


IMAGE_HALF_WIDTH = 94.0
IMAGE_HALF_HEIGHT = 60.0
CENTER_HALF_WIDTH = 140.0
CENTER_HALF_HEIGHT = 110.0

_CENTER_TO_FRONT_X = (
    4.598873713,
    1.112662896,
    0.03961500727,
    -0.0009593892413,
    0.003638791921,
    0.0001953756646,
)
_CENTER_TO_FRONT_Y = (
    57.76555674,
    -0.04623574104,
    0.7418767472,
    -0.003921243931,
    0.00009845932112,
    -0.003075629357,
)
_CENTER_TO_BACK_X = (
    -11.82017882,
    -1.112148871,
    0.05965474006,
    0.001312940857,
    0.003580014712,
    -0.0005696456455,
)
_CENTER_TO_BACK_Y = (
    58.42283122,
    -0.04773654331,
    -0.7173750546,
    -0.003842275563,
    -0.0004617931,
    -0.005021741498,
)


def _evaluate(coefficients: tuple[float, ...], x: float, y: float) -> float:
    """执行与固件一致的六项二次多项式乘加。"""

    return (
        coefficients[0]
        + coefficients[1] * x
        + coefficients[2] * y
        + coefficients[3] * x * x
        + coefficients[4] * x * y
        + coefficients[5] * y * y
    )


def from_center(camera_index: int, center_x: float, center_y: float) -> tuple[float, float] | None:
    """把下摄公共坐标转换到指定摄像头坐标，越出标定范围时返回None。"""

    if (
        not math.isfinite(center_x)
        or not math.isfinite(center_y)
        or abs(center_x) > CENTER_HALF_WIDTH
        or abs(center_y) > CENTER_HALF_HEIGHT
    ):
        return None
    if camera_index == 1:
        source_x, source_y = center_x, center_y
    elif camera_index == 0:
        source_x = _evaluate(_CENTER_TO_FRONT_X, center_x, center_y)
        source_y = _evaluate(_CENTER_TO_FRONT_Y, center_x, center_y)
    elif camera_index == 2:
        source_x = _evaluate(_CENTER_TO_BACK_X, center_x, center_y)
        source_y = _evaluate(_CENTER_TO_BACK_Y, center_x, center_y)
    else:
        raise ValueError("camera_index must be 0, 1 or 2")
    if abs(source_x) > IMAGE_HALF_WIDTH or abs(source_y) > IMAGE_HALF_HEIGHT:
        return None
    return source_x, source_y
