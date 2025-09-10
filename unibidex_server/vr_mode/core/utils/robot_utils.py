import numpy as np
from pytransform3d import rotations, coordinates

# QUESTROTATION2ROBOT = np.array([
#     [1, 0, 0],
#     [0, 1, 0],
#     [0, 0, 1],
# ])

QUESTROTATION2ROBOT = np.array([
    [ 0,  1,  0],
    [-1,  0,  0],
    [ 0,  0, -1],
])

QUESTTRANSLATION2ROBOT = np.array([
    [ 0,  0, 1],
    [-1,  0,  0],
    [ 0,  1,  0],
])

MANO2ROBOT = np.array(
    [
        [-1, 0, 0],
        [0, 0, -1],
        [0, -1, 0],
    ]
)

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ]
)

OPERATOR2AVP_RIGHT = OPERATOR2MANO_RIGHT

OPERATOR2AVP_LEFT = np.array(
    [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)


def project_average_rotation(quat_list: np.ndarray) -> np.ndarray:
    """
    从一系列四元数中，计算一个“投影平均”四元数：
      1. 将重力方向（0,0,1）对齐到最主要的本地轴；
      2. 在水平面上做航向角（azimuth）的圆平均；
      3. 重建并正交化旋转矩阵，返回单位四元数。
    """
    # 输入校验
    if quat_list.ndim != 2 or quat_list.shape[1] != 4 or quat_list.shape[0] < 1:
        raise ValueError("quat_list 必须是 N×4 数组，且 N ≥ 1")

    # 特殊情况：只有一个四元数，直接返回
    if quat_list.shape[0] == 1:
        return quat_list[0].copy()

    gravity_dir = np.array([0.0, 0.0, 1.0])

    # 1. 计算所有帧中，重力在各地坐标轴上的投影，并取平均
    mats = [rotations.matrix_from_quaternion(q) for q in quat_list]
    gravity_projs = np.stack([gravity_dir @ M for M in mats], axis=0)  # (N, 3)
    avg_gravity = gravity_projs.mean(axis=0)

    # 2. 选出投影绝对值最大的轴，并取符号
    max_grav_axis = int(np.argmax(np.abs(avg_gravity)))
    sign = 1.0 if avg_gravity[max_grav_axis] >= 0 else -1.0

    # 3. 水平面旋转轴索引
    next_axis       = (max_grav_axis + 1) % 3
    next_next_axis  = (max_grav_axis + 2) % 3

    # 4. 收集每帧在水平面上对应 next_axis 列向量的方位角（azimuth）
    angles = []
    for M in mats:
        v = M[:3, next_axis].copy()
        # 去除竖直分量
        v[2] = 0.0
        # spherical_from_cartesian 返回 (r, inclination, azimuth)
        _, _, az = coordinates.spherical_from_cartesian(v)
        angles.append(az)
    angles = np.array(angles)

    # 5. 对角度做圆平均
    c_mean = np.cos(angles).mean()
    s_mean = np.sin(angles).mean()
    mean_angle = np.arctan2(s_mean, c_mean)

    # 6. 根据对齐后的重力轴与平均水平角，构建旋转矩阵
    final_mat = np.zeros((3, 3))
    final_mat[:, max_grav_axis]      = sign * gravity_dir
    final_mat[:, next_axis]          = np.array([np.cos(mean_angle), np.sin(mean_angle), 0.0])
    final_mat[:, next_next_axis]     = np.cross(
        final_mat[:, max_grav_axis],
        final_mat[:, next_axis]
    )

    # 7. SVD 正交化，确保正交矩阵和行列式 +1
    U, _, Vt = np.linalg.svd(final_mat)
    final_mat_norm = U @ Vt
    if not np.isclose(np.linalg.det(final_mat_norm), 1.0, atol=1e-6):
        raise ValueError("正交化后矩阵的行列式不为 +1")

    # 8. 转回四元数并返回
    return rotations.quaternion_from_matrix(final_mat_norm, strict_check=True)


class LPFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.y = None
        self.is_init = False

    def next(self, x):
        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()
        self.y = self.y + self.alpha * (x - self.y)
        return self.y.copy()

    def reset(self):
        self.y = None
        self.is_init = False


class LPRotationFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.is_init = False

        self.y = None

    def next(self, x: np.ndarray):
        assert x.shape == (4,)

        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()

        self.y = rotations.quaternion_slerp(self.y, x, self.alpha, shortest_path=True)
        return self.y.copy()

    def reset(self):
        self.y = None
        self.is_init = False
