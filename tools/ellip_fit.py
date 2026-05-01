# ============================================================
# 椭球拟合算法 — 与 Accel_Calibration.c 中 ellip_fit_solve() 完全一致
# ============================================================

# ---- 29 个有效姿态点的加速度均值 (g) ----
poses = [
    [-0.020766, -0.009126, -1.011503],
    [0.032579, 0.826635, -0.572024],
    [0.813559, -0.006215, -0.591798],
    [-0.000131, -0.855512, -0.531743],
    [-0.837114, -0.003669, -0.551435],
    [0.024960, -0.014867, 0.990222],
    [0.013099, 0.222328, -0.986387],
    [-0.015589, -0.271869, -0.974533],
    [0.245104, -0.028481, -0.981462],
    [-0.005566, 0.225957, -0.985719],
    [0.030640, 0.555564, -0.841713],
    [-0.596437, -0.007665, -0.811251],
    [-0.004837, -0.557971, -0.842309],
    [0.582974, -0.002709, -0.824025],
    [0.403376, 0.175138, 0.888681],
    [0.215942, -0.064675, 0.965166],
    [0.007967, -0.242539, 0.961146],
    [-0.222808, -0.035059, 0.963147],
    [-0.335894, -0.147344, 0.918459],
    [-0.065747, 0.375874, 0.913054],
    [0.171554, 0.361921, 0.905791],
    [0.368641, -0.003320, 0.920542],
    [0.006082, -0.345857, 0.928859],
    [-0.342650, 0.019300, 0.927401],
    [0.021889, 0.360948, 0.921516],
    [0.469921, -0.015390, 0.873457],
    [0.034380, 0.510577, 0.848244],
    [-0.451959, 0.032028, 0.878494],
    [-0.564340, 0.035101, 0.811245],
]

n = len(poses)
print(f"姿态点数: {n}")

# ---- Step 1: 构建设计矩阵 A (n x 9) ----
# 每一行: [x^2, y^2, z^2, xy, xz, yz, x, y, z]
A = []
for x, y, z in poses:
    A.append([x*x, y*y, z*z, x*y, x*z, y*z, x, y, z])

# 目标向量 b = [1, 1, ..., 1]
b = [1.0] * n

# ---- Step 2: 计算 AtA (9x9) 和 Atb (9x1) ----
AtA = [[0.0]*9 for _ in range(9)]
Atb = [0.0]*9
for i in range(n):
    for j in range(9):
        Atb[j] += A[i][j] * b[i]
        for k in range(9):
            AtA[j][k] += A[i][j] * A[i][k]

# ---- Step 3: 高斯消元求解 AtA * p = Atb (9x9, 带部分主元) ----
def gauss_solve(A_mat, b_vec, size):
    M = [[0.0]*(size+1) for _ in range(size)]
    for i in range(size):
        for j in range(size):
            M[i][j] = A_mat[i][j]
        M[i][size] = b_vec[i]

    for col in range(size):
        max_row = col
        max_val = abs(M[col][col])
        for row in range(col+1, size):
            if abs(M[row][col]) > max_val:
                max_val = abs(M[row][col])
                max_row = row
        if max_val < 1e-15:
            continue
        if max_row != col:
            M[col], M[max_row] = M[max_row], M[col]
        pivot = M[col][col]
        for row in range(col+1, size):
            factor = M[row][col] / pivot
            for j in range(col, size+1):
                M[row][j] -= factor * M[col][j]

    x = [0.0]*size
    for i in range(size-1, -1, -1):
        s = M[i][size]
        for j in range(i+1, size):
            s -= M[i][j] * x[j]
        if abs(M[i][i]) > 1e-15:
            x[i] = s / M[i][i]
        else:
            x[i] = 0.0
    return x

p = gauss_solve(AtA, Atb, 9)

print(f"\n椭球方程参数 p0-p8:")
for i in range(9):
    print(f"  p{i} = {p[i]:.10f}")

# ---- Step 4: 组装 Q 矩阵和 g 向量 ----
Q = [[p[0], p[3]/2.0, p[4]/2.0],
     [p[3]/2.0, p[1], p[5]/2.0],
     [p[4]/2.0, p[5]/2.0, p[2]]]

g_vec = [p[6]/2.0, p[7]/2.0, p[8]/2.0]

print(f"\nQ 矩阵:")
for row in Q:
    print(f"  [{row[0]:.10f}, {row[1]:.10f}, {row[2]:.10f}]")
print(f"g 向量: [{g_vec[0]:.10f}, {g_vec[1]:.10f}, {g_vec[2]:.10f}]")

# ---- Step 5: 对称 3x3 矩阵求逆 ----
def mat3_inv_sym(M):
    a, b, c = M[0][0], M[0][1], M[0][2]
    d, e    = M[1][1], M[1][2]
    f       = M[2][2]

    det = a*(d*f - e*e) - b*(b*f - e*c) + c*(b*e - d*c)
    if abs(det) < 1e-15:
        return [[0.0]*3 for _ in range(3)]

    inv_det = 1.0 / det
    result = [[0.0]*3 for _ in range(3)]
    result[0][0] = (d*f - e*e) * inv_det
    result[0][1] = (c*e - b*f) * inv_det
    result[0][2] = (b*e - c*d) * inv_det
    result[1][0] = result[0][1]
    result[1][1] = (a*f - c*c) * inv_det
    result[1][2] = (b*c - a*e) * inv_det
    result[2][0] = result[0][2]
    result[2][1] = result[1][2]
    result[2][2] = (a*d - b*b) * inv_det
    return result

Q_inv = mat3_inv_sym(Q)

# ---- Step 6: bias = -Q^{-1} * g ----
def mat3_mul_vec(M, v):
    return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
    ]

temp = mat3_mul_vec(Q_inv, g_vec)
bias = [-temp[0], -temp[1], -temp[2]]

print(f"\nbias (椭球中心, g): [{bias[0]:.8f}, {bias[1]:.8f}, {bias[2]:.8f}]")
print(f"bias norm: {((bias[0]**2 + bias[1]**2 + bias[2]**2)**0.5):.6f} g")

# ---- Step 7: k = bias^T * Q * bias + 1 ----
Qb = mat3_mul_vec(Q, bias)
k = bias[0]*Qb[0] + bias[1]*Qb[1] + bias[2]*Qb[2] + 1.0

print(f"k = {k:.10f}")

# ---- Step 8: Q_scaled = Q / k ----
Q_scaled = [[Q[i][j]/k for j in range(3)] for i in range(3)]

# ---- Step 9: Cholesky 分解 Q_scaled = L * L^T ----
def cholesky_3x3(A):
    L = [[0.0]*3 for _ in range(3)]
    L[0][0] = A[0][0]**0.5
    if L[0][0] < 1e-15:
        return None
    L[1][0] = A[1][0] / L[0][0]
    L[2][0] = A[2][0] / L[0][0]

    L[1][1] = (A[1][1] - L[1][0]*L[1][0])**0.5
    if L[1][1] < 1e-15:
        return None
    L[2][1] = (A[2][1] - L[2][0]*L[1][0]) / L[1][1]

    L[2][2] = (A[2][2] - L[2][0]*L[2][0] - L[2][1]*L[2][1])**0.5
    if L[2][2] < 1e-15:
        return None
    return L

L = cholesky_3x3(Q_scaled)

# M = L^T (上三角)
M = [[L[0][0], L[1][0], L[2][0]],
     [0.0,      L[1][1], L[2][1]],
     [0.0,      0.0,      L[2][2]]]

print(f"\n校正矩阵 M[3][3] = accel_corr_matrix:")
print(f"  [{M[0][0]:.10f}, {M[0][1]:.10f}, {M[0][2]:.10f}]")
print(f"  [{M[1][0]:.10f}, {M[1][1]:.10f}, {M[1][2]:.10f}]")
print(f"  [{M[2][0]:.10f}, {M[2][1]:.10f}, {M[2][2]:.10f}]")

diag_scale = [M[0][0], M[1][1], M[2][2]]
print(f"\n对角线 scale: [{diag_scale[0]:.8f}, {diag_scale[1]:.8f}, {diag_scale[2]:.8f}]")

# ---- 验证: 所有校正后向量的范数应 = 1 ----
print(f"\n========== 验证 ==========")
max_err = 0.0
rms_err = 0.0
for i, (x, y, z) in enumerate(poses):
    cx = x - bias[0]
    cy = y - bias[1]
    cz = z - bias[2]
    corr_x = M[0][0]*cx + M[0][1]*cy + M[0][2]*cz
    corr_y = M[1][0]*cx + M[1][1]*cy + M[1][2]*cz
    corr_z = M[2][0]*cx + M[2][1]*cy + M[2][2]*cz
    norm = (corr_x**2 + corr_y**2 + corr_z**2)**0.5
    err = abs(norm - 1.0)
    rms_err += err**2
    if err > max_err:
        max_err = err
    if i < 5 or err > 0.002:
        print(f"  Pose {i+1:2d}: raw=[{x:8.4f}, {y:8.4f}, {z:8.4f}] -> |cal|={norm:.6f}, err={err*1000:.3f}mg")

rms_err = (rms_err / n)**0.5
print(f"\n最大范数误差: {max_err*1000:.3f} mg")
print(f"RMS  范数误差: {rms_err*1000:.3f} mg")

# ---- 检查是否通过C代码的验证阈值 ----
print(f"\n========== C代码验证阈值 ==========")
bias_norm = (bias[0]**2 + bias[1]**2 + bias[2]**2)**0.5
print(f"bias_norm = {bias_norm:.4f} g  (阈值 < 0.35g) {'PASS' if bias_norm < 0.35 else 'FAIL'}")
for i in range(3):
    print(f"M[{i}][{i}] = {M[i][i]:.4f}  (阈值 [0.85, 1.15]) {'PASS' if 0.85 <= M[i][i] <= 1.15 else 'FAIL'}")
print(f"max_norm_err = {max_err*1000:.3f} mg  (阈值 < 30mg) {'PASS' if max_err < 0.03 else 'FAIL'}")

# ---- 非对角元检测 ----
has_off_diag = False
for i in range(3):
    for j in range(3):
        if i != j and abs(M[i][j]) > 1e-6:
            has_off_diag = True
print(f"\nuse_full_matrix = {1 if has_off_diag else 0}")

# ---- 最终输出: Flash blob 参数 (C代码格式) ----
print(f"\n========== Flash 存储参数 (直接拷贝到代码) ==========")
print(f"// accel_bias_g (椭球中心, 单位 g)")
print(f"g_accel_calibration.accel_bias_g[0] = {bias[0]:.8f}f;")
print(f"g_accel_calibration.accel_bias_g[1] = {bias[1]:.8f}f;")
print(f"g_accel_calibration.accel_bias_g[2] = {bias[2]:.8f}f;")
print(f"")
print(f"// accel_corr_matrix (3x3 校正矩阵)")
print(f"g_accel_calibration.accel_corr_matrix[0][0] = {M[0][0]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[0][1] = {M[0][1]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[0][2] = {M[0][2]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[1][0] = {M[1][0]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[1][1] = {M[1][1]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[1][2] = {M[1][2]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[2][0] = {M[2][0]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[2][1] = {M[2][1]:.8f}f;")
print(f"g_accel_calibration.accel_corr_matrix[2][2] = {M[2][2]:.8f}f;")
print(f"")
print(f"// accel_scale (对角线回退)")
print(f"g_accel_calibration.accel_scale[0] = {diag_scale[0]:.8f}f;")
print(f"g_accel_calibration.accel_scale[1] = {diag_scale[1]:.8f}f;")
print(f"g_accel_calibration.accel_scale[2] = {diag_scale[2]:.8f}f;")
print(f"")
print(f"g_accel_calibration.use_full_matrix = {1 if has_off_diag else 0};")
print(f"g_accel_calibration.is_calibrated = true;")

# ---- 打印原始数据范数统计 (校准前) ----
raw_norms = [(x**2 + y**2 + z**2)**0.5 for x,y,z in poses]
print(f"\n========== 校准前原始数据范数统计 ==========")
print(f"min={min(raw_norms):.6f}, max={max(raw_norms):.6f}, mean={sum(raw_norms)/n:.6f}")
