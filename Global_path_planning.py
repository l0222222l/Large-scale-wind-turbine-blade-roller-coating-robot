#!/usr/bin/env python3
import pandas as pd
import numpy as np
import math
import os
import multiprocessing
from scipy.spatial import cKDTree
import open3d as o3d

# 屏蔽 Open3D 的 Warning 输出
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)


# ==========================================
# 🌟 顶层可视化进程函数
# ==========================================
def show_pcd_window(points, colors, title, left_pos):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd],
                                      window_name=title,
                                      width=800, height=800,
                                      left=left_pos, top=50,
                                      point_show_normal=False)


# ==========================================
# 🌟 IK 逆运动学模块
# ==========================================
d2r = np.pi / 180.0

ROBOT_CONFIG = {
    'fanuc': {
        'dh': [
            [0.0, 0.0, 600.0],
            [-90.0 * d2r, 720.0, 0.0],
            [0.0, 1075.0, 0.0],
            [-90.0 * d2r, 225.0, 1690.0],
            [90.0 * d2r, 0.0, 0.0],
            [-90.0 * d2r, 0.0, 235.0]
        ],
        'limits': np.array([[-180, 180], [-120, 65], [-65, 45], [-360, 360], [-125, 125], [-360, 360]]) * d2r
    },
    'kuka': {
        'dh': [
            [0.0, 0.0, 590.0],
            [-90.0 * d2r, 750.0, 0.0],
            [0.0, 1350.0, 0.0],
            [-90.0 * d2r, -41.0, 1400.0],
            [90.0 * d2r, 0.0, 0.0],
            [-90.0 * d2r, 0.0, 240.0]
        ],
        'limits': np.array([[-185, 185], [-120, 70], [-210, 65], [-350, 350], [-122.5, 122.5], [-350, 350]]) * d2r
    }
}


def mdh_matrix(alpha, a, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])


def check_limits(q, robot_type):
    lower_bounds = ROBOT_CONFIG[robot_type]['limits'][:, 0]
    upper_bounds = ROBOT_CONFIG[robot_type]['limits'][:, 1]
    for i in range(6):
        if q[i] < lower_bounds[i] or q[i] > upper_bounds[i]:
            return False
    return True


def create_target_pose_from_normal(position, z_dir):
    z_dir = np.array(z_dir)
    z_dir = z_dir / np.linalg.norm(z_dir)
    v_ref = np.array([0.0, 0.0, 1.0])

    if abs(np.dot(z_dir, v_ref)) > 0.99:
        v_ref = np.array([0.0, 1.0, 0.0])

    x_dir = np.cross(v_ref, z_dir)
    x_dir /= np.linalg.norm(x_dir)
    y_dir = np.cross(z_dir, x_dir)

    T = np.eye(4)
    T[:3, 0] = x_dir
    T[:3, 1] = y_dir
    T[:3, 2] = z_dir
    T[:3, 3] = position
    return T


def analytical_ik(T_target, robot_type='fanuc'):
    valid_solutions = []
    dh = ROBOT_CONFIG[robot_type]['dh']

    d1, a1, a2, a3, d4, d6 = dh[0][2], dh[1][1], dh[2][1], dh[3][1], dh[3][2], dh[5][2]

    Pw = T_target[:3, 3] - d6 * T_target[:3, 2]
    x, y, z = Pw[0], Pw[1], Pw[2]
    r_xy = np.sqrt(x ** 2 + y ** 2)

    if r_xy < abs(a1): return valid_solutions

    theta1_1 = np.arctan2(y, x)
    theta1_2 = np.arctan2(-y, -x)

    for th1 in [theta1_1, theta1_2]:
        th1 = np.arctan2(np.sin(th1), np.cos(th1))
        c1, s1 = np.cos(th1), np.sin(th1)
        Px, Pz = x * c1 + y * s1 - a1, z - d1

        L_sq, L3_sq = Px ** 2 + Pz ** 2, a3 ** 2 + d4 ** 2
        val = (L_sq - a2 ** 2 - L3_sq) / (2 * a2 * np.sqrt(L3_sq))
        if abs(val) > 1.0: continue

        for beta in [np.arccos(val), -np.arccos(val)]:
            phi = np.arctan2(-d4, a3)
            th3 = beta + phi
            gamma = np.arctan2(np.sqrt(L3_sq) * np.sin(beta), a2 + np.sqrt(L3_sq) * np.cos(beta))
            th2 = np.arctan2(Pz, Px) - gamma

            th1 = np.arctan2(np.sin(th1), np.cos(th1))
            th2 = np.arctan2(np.sin(th2), np.cos(th2))
            th3 = np.arctan2(np.sin(th3), np.cos(th3))

            R0_3 = (mdh_matrix(dh[0][0], dh[0][1], dh[0][2], th1) @
                    mdh_matrix(dh[1][0], dh[1][1], dh[1][2], th2) @
                    mdh_matrix(dh[2][0], dh[2][1], dh[2][2], th3))[:3, :3]

            R3_6 = R0_3.T @ T_target[:3, :3]
            c5 = R3_6[1, 2]

            if abs(c5) < 0.9999:
                s5_1 = np.sqrt(1 - c5 ** 2)
                s5_2 = -s5_1
                for s5 in [s5_1, s5_2]:
                    th5 = np.arctan2(s5, c5)
                    th4 = np.arctan2(R3_6[2, 2] / s5, -R3_6[0, 2] / s5)
                    th6 = np.arctan2(-R3_6[1, 1] / s5, R3_6[1, 0] / s5)
                    q = np.array([th1, th2, th3, th4, th5, th6])
                    if check_limits(q, robot_type): valid_solutions.append(q)
            else:
                th5 = 0.0 if c5 > 0 else np.pi
                th4 = 0.0
                th6 = np.arctan2(-R3_6[0, 1], R3_6[0, 0]) if c5 > 0 else np.arctan2(R3_6[0, 1], -R3_6[0, 0])
                q = np.array([th1, th2, th3, th4, th5, th6])
                if check_limits(q, robot_type): valid_solutions.append(q)

    return valid_solutions


# ==========================================
# 🌟 全局参数与配置
# ==========================================
SAFE_DISTANCE = 2300
ROBOT_REACH = 3500
GRID_STEP = 300
MAX_BASES = 33

# 新增路径生成参数
X_SAMPLE_INTERVAL = 1000.0
START_OFFSET = 500.0

FANUC_OFFSET_X = -0.915
FANUC_OFFSET_Y = -0.100
FANUC_GLOBAL_Z = 3140.0
FANUC_YAW = -math.pi / 2
FANUC_Z_DIR = 1

KUKA_OFFSET_X = 1.095
KUKA_OFFSET_Y = -0.100
KUKA_GLOBAL_Z = 3210.0
KUKA_YAW = -math.pi / 2
KUKA_Z_DIR = -1


# ==========================================
# 🌟 核心计算类
# ==========================================
class CoverageComparisonCalculator:
    def __init__(self):
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.grid_pos, self.dirs_pos, self.meta_pos = self.load_npz_to_mask_grid('processed_工作空间数据上半.npz')
        self.grid_neg, self.dirs_neg, self.meta_neg = self.load_npz_to_mask_grid('processed_工作空间数据下半.npz')
        self.agv_xy = None
        self.df_surface_full = None  # 记录全量原点云以供显示

    def generate_surface_paths(self, df, interval=1000.0, start_offset=500.0):
        """核心步骤 1：在原始点云上根据辊筒长度抽稀生成连续路径点"""
        print(f"📏 正在生成 {interval}mm 间隔的表面辊涂路径 (起点偏移 {start_offset}mm) ...")
        min_x = df['X'].min()
        max_x = df['X'].max()

        base_x = (min_x // interval) * interval
        start_x = base_x + start_offset
        if start_x < min_x:
            start_x += interval

        path_dfs = []
        for target_x in range(int(start_x), int(max_x) + 1, int(interval)):
            slice_df = df[(df['X'] >= target_x - 200) & (df['X'] <= target_x + 200)].copy()
            if slice_df.empty: continue

            # Z轴每 50mm 抽稀一个点形成平滑连续的线段
            slice_df['Z_bin'] = (slice_df['Z'] // 50).astype(int)
            slice_df['dist_to_x'] = (slice_df['X'] - target_x).abs()
            clean_df = slice_df.loc[slice_df.groupby('Z_bin')['dist_to_x'].idxmin()].copy()
            clean_df['X'] = float(target_x)  # 强制对齐为绝对直线
            path_dfs.append(clean_df)

        if not path_dfs:
            return pd.DataFrame()

        df_paths = pd.concat(path_dfs, ignore_index=True)
        print(f"✅ 提取表面路径成功，共计 {len(df_paths)} 个作业关键点。")
        return df_paths

    def apply_3d_tool_offset(self, df, tcp_offset=[66.5, 88.5, 637.0]):
        """核心步骤 2：对生成的路径点进行姿态解算和三维偏置"""
        print(f"⚙️ 正在对路径点应用 3D 工具偏置 {tcp_offset} 计算法兰目标 ...")
        pts = df[['X', 'Y', 'Z']].values
        norms = df[['I', 'J', 'K']].values

        norms_len = np.linalg.norm(norms, axis=1, keepdims=True)
        norms = np.where(norms_len == 0, 1.0, norms / norms_len)

        Z_w = -norms
        ref_y = np.array([1.0, 0.0, 0.0])
        dot_product = np.sum(ref_y * Z_w, axis=1, keepdims=True)
        Y_w = ref_y - dot_product * Z_w

        y_len = np.linalg.norm(Y_w, axis=1, keepdims=True)
        degenerate_mask = (y_len < 1e-6).flatten()
        if np.any(degenerate_mask):
            Y_w[degenerate_mask] = np.cross(Z_w[degenerate_mask], np.array([0.0, 1.0, 0.0]))
            y_len[degenerate_mask] = np.linalg.norm(Y_w[degenerate_mask], axis=1, keepdims=True)

        Y_w = Y_w / y_len
        X_w = np.cross(Y_w, Z_w)

        T_x, T_y, T_z = tcp_offset
        offset_vector = (X_w * T_x) + (Y_w * T_y) + (Z_w * T_z)
        flange_pts = pts - offset_vector

        df_offset = df.copy()
        df_offset['X_orig'] = pts[:, 0]
        df_offset['Y_orig'] = pts[:, 1]
        df_offset['Z_orig'] = pts[:, 2]

        # 覆盖为偏置后的法兰目标点
        df_offset['X'] = flange_pts[:, 0]
        df_offset['Y'] = flange_pts[:, 1]
        df_offset['Z'] = flange_pts[:, 2]

        return df_offset

    def load_npz_to_mask_grid(self, filename):
        path = os.path.join(self.base_dir, filename)
        if not os.path.exists(path):
            path = os.path.join(self.base_dir, filename.replace('processed_', ''))
        try:
            data = np.load(path, allow_pickle=True)
            coords, masks = data['coords'], data['masks']
            directions = data['directions'] if 'directions' in data else None
            num_angles = masks.shape[1]

            coords_rounded = np.round(coords, decimals=1)
            unique_x, unique_y, unique_z = np.sort(np.unique(coords_rounded[:, 0])), np.sort(
                np.unique(coords_rounded[:, 1])), np.sort(np.unique(coords_rounded[:, 2]))

            meta = {
                'min_x': unique_x[0], 'step_x': unique_x[1] - unique_x[0] if len(unique_x) > 1 else 100.0,
                'dim_x': len(unique_x),
                'min_y': unique_y[0], 'step_y': unique_y[1] - unique_y[0] if len(unique_y) > 1 else 100.0,
                'dim_y': len(unique_y),
                'min_z': unique_z[0], 'step_z': unique_z[1] - unique_z[0] if len(unique_z) > 1 else 100.0,
                'dim_z': len(unique_z),
            }

            grid = np.zeros((meta['dim_x'], meta['dim_y'], meta['dim_z'], num_angles), dtype=np.bool_)
            ixs = np.round((coords[:, 0] - meta['min_x']) / meta['step_x']).astype(int)
            iys = np.round((coords[:, 1] - meta['min_y']) / meta['step_y']).astype(int)
            izs = np.round((coords[:, 2] - meta['min_z']) / meta['step_z']).astype(int)
            grid[ixs, iys, izs, :] = masks
            return grid, directions, meta
        except Exception as e:
            return np.zeros((100, 100, 100, 1), dtype=np.bool_), None, None

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _compute_base_coverage_mask(self, cx, cy, theta, pts_pos, pts_neg, norms_pos, norms_neg, grid_pos, grid_neg,
                                    num_pos, num_neg):
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        coverage = np.zeros(num_pos + num_neg, dtype=bool)

        if num_pos > 0:
            fx = cx + (FANUC_OFFSET_X * 1000.0) * cos_t - (FANUC_OFFSET_Y * 1000.0) * sin_t
            fy = cy + (FANUC_OFFSET_X * 1000.0) * sin_t + (FANUC_OFFSET_Y * 1000.0) * cos_t
            ftheta = theta + FANUC_YAW
            cos_f, sin_f = math.cos(-ftheta), math.sin(-ftheta)

            mask_xy = (np.abs(pts_pos[:, 0] - fx) < 4000) & (np.abs(pts_pos[:, 1] - fy) < 4000)
            if mask_xy.any():
                v_idx = np.where(mask_xy)[0]
                offset_lx = fx * cos_f - fy * sin_f
                offset_ly = fx * sin_f + fy * cos_f

                pre_lx = pts_pos[v_idx, 0] * cos_f - pts_pos[v_idx, 1] * sin_f
                pre_ly = pts_pos[v_idx, 0] * sin_f + pts_pos[v_idx, 1] * cos_f
                lx, ly = pre_lx - offset_lx, pre_ly - offset_ly
                lz = (pts_pos[v_idx, 2] - FANUC_GLOBAL_Z) * FANUC_Z_DIR

                nx = norms_pos[v_idx, 0] * cos_f - norms_pos[v_idx, 1] * sin_f
                ny = norms_pos[v_idx, 0] * sin_f + norms_pos[v_idx, 1] * cos_f
                nz = norms_pos[v_idx, 2] * FANUC_Z_DIR
                dot_products = -np.stack((nx, ny, nz), axis=-1) @ self.dirs_pos.T
                best_dir = np.argmax(dot_products, axis=-1)

                idx_x = np.round((lx - self.meta_pos['min_x']) / self.meta_pos['step_x']).astype(int)
                idx_y = np.round((ly - self.meta_pos['min_y']) / self.meta_pos['step_y']).astype(int)
                idx_z = np.round((lz - self.meta_pos['min_z']) / self.meta_pos['step_z']).astype(int)

                in_bounds = (idx_x >= 0) & (idx_x < self.meta_pos['dim_x']) & (idx_y >= 0) & (
                            idx_y < self.meta_pos['dim_y']) & (idx_z >= 0) & (idx_z < self.meta_pos['dim_z'])
                if in_bounds.any():
                    in_v_idx = v_idx[in_bounds]
                    coverage[in_v_idx] = grid_pos[
                        idx_x[in_bounds], idx_y[in_bounds], idx_z[in_bounds], best_dir[in_bounds]]

        if num_neg > 0:
            kx = cx + (KUKA_OFFSET_X * 1000.0) * cos_t - (KUKA_OFFSET_Y * 1000.0) * sin_t
            ky = cy + (KUKA_OFFSET_X * 1000.0) * sin_t + (KUKA_OFFSET_Y * 1000.0) * cos_t
            ktheta = theta + KUKA_YAW
            cos_k, sin_k = math.cos(-ktheta), math.sin(-ktheta)

            mask_xy = (np.abs(pts_neg[:, 0] - kx) < 4000) & (np.abs(pts_neg[:, 1] - ky) < 4000)
            if mask_xy.any():
                v_idx = np.where(mask_xy)[0]
                offset_lx = kx * cos_k - ky * sin_k
                offset_ly = -(kx * sin_k + ky * cos_k)

                pre_lx = pts_neg[v_idx, 0] * cos_k - pts_neg[v_idx, 1] * sin_k
                pre_ly = -(pts_neg[v_idx, 0] * sin_k + pts_neg[v_idx, 1] * cos_k)
                lx, ly = pre_lx - offset_lx, pre_ly - offset_ly
                lz = (pts_neg[v_idx, 2] - KUKA_GLOBAL_Z) * KUKA_Z_DIR

                nx = norms_neg[v_idx, 0] * cos_k - norms_neg[v_idx, 1] * sin_k
                ny = -(norms_neg[v_idx, 0] * sin_k + norms_neg[v_idx, 1] * cos_k)
                nz = norms_neg[v_idx, 2] * KUKA_Z_DIR
                dot_products = -np.stack((nx, ny, nz), axis=-1) @ self.dirs_neg.T
                best_dir = np.argmax(dot_products, axis=-1)

                idx_x = np.round((lx - self.meta_neg['min_x']) / self.meta_neg['step_x']).astype(int)
                idx_y = np.round((ly - self.meta_neg['min_y']) / self.meta_neg['step_y']).astype(int)
                idx_z = np.round((lz - self.meta_neg['min_z']) / self.meta_neg['step_z']).astype(int)

                in_bounds = (idx_x >= 0) & (idx_x < self.meta_neg['dim_x']) & (idx_y >= 0) & (
                            idx_y < self.meta_neg['dim_y']) & (idx_z >= 0) & (idx_z < self.meta_neg['dim_z'])
                if in_bounds.any():
                    in_v_idx = v_idx[in_bounds]
                    coverage[num_pos + in_v_idx] = grid_neg[
                        idx_x[in_bounds], idx_y[in_bounds], idx_z[in_bounds], best_dir[in_bounds]]

        return coverage

    def optimize_agv_poses(self, bases, pts_pos, pts_neg, norms_pos, norms_neg, grid_pos, grid_neg, normal_y_dir):
        if len(bases) == 0: return bases, np.array([])
        num_pos = len(pts_pos)
        num_neg = len(pts_neg)
        initial_theta = math.pi if normal_y_dir < 0 else 0.0
        theta_bounds = (math.pi * 0.35, math.pi * 1.65) if normal_y_dir < 0 else (-math.pi * 0.65, math.pi * 0.65)

        print(f"\n🔧 开始优化 {len(bases)} 个AGV基站的朝向和位置...")
        theta_search = np.linspace(theta_bounds[0], theta_bounds[1], 37)
        optimized_thetas = np.full(len(bases), initial_theta)

        for i in range(len(bases)):
            bx, by = bases[i]
            best_theta, best_cov = initial_theta, 0
            for th in theta_search:
                mask = self._compute_base_coverage_mask(bx, by, th, pts_pos, pts_neg, norms_pos, norms_neg, grid_pos,
                                                        grid_neg, num_pos, num_neg)
                cov = np.count_nonzero(mask)
                if cov > best_cov: best_cov, best_theta = cov, th
            optimized_thetas[i] = best_theta

        refined_bases = bases.copy().astype(float)
        position_offsets = [(0, 0), (150, 0), (-150, 0), (0, 150), (0, -150), (300, 0), (-300, 0), (0, 300), (0, -300),
                            (150, 150), (-150, 150), (150, -150), (-150, -150)]

        for i in range(len(refined_bases)):
            bx, by = refined_bases[i]
            best_pos = np.array([bx, by])
            best_cov = np.count_nonzero(
                self._compute_base_coverage_mask(bx, by, optimized_thetas[i], pts_pos, pts_neg, norms_pos, norms_neg,
                                                 grid_pos, grid_neg, num_pos, num_neg))
            for dx, dy in position_offsets:
                cand_pos = np.array([bx + dx, by + dy])
                dists, _ = self.global_tree.query(cand_pos.reshape(1, -1))
                if dists[0] <= SAFE_DISTANCE: continue
                cov = np.count_nonzero(
                    self._compute_base_coverage_mask(cand_pos[0], cand_pos[1], optimized_thetas[i], pts_pos, pts_neg,
                                                     norms_pos, norms_neg, grid_pos, grid_neg, num_pos, num_neg))
                if cov > best_cov: best_cov, best_pos = cov, cand_pos
            refined_bases[i] = best_pos

        print(f"✅ 优化完成\n")
        return refined_bases, optimized_thetas

    def greedy_initialization(self, pts_pos, pts_neg, norms_pos, norms_neg, grid_pos, grid_neg, normal_y_dir):
        all_xy = np.vstack((pts_pos[:, :2], pts_neg[:, :2])) if len(pts_pos) > 0 and len(pts_neg) > 0 else (
            pts_pos[:, :2] if len(pts_pos) > 0 else pts_neg[:, :2])
        min_x, max_x = np.min(all_xy[:, 0]), np.max(all_xy[:, 0])
        min_y, max_y = np.min(all_xy[:, 1]), np.max(all_xy[:, 1])

        y_grid = np.arange(min_y - ROBOT_REACH, max_y + 200, GRID_STEP) if normal_y_dir < 0 else np.arange(min_y - 200,
                                                                                                           max_y + ROBOT_REACH,
                                                                                                           GRID_STEP)
        x_grid = np.arange(min_x - ROBOT_REACH, min(0, max_x + ROBOT_REACH), GRID_STEP * 1.5)
        if len(x_grid) == 0: x_grid = np.array([-500.0])
        gx, gy = np.meshgrid(x_grid, y_grid)
        candidates = np.vstack([gx.ravel(), gy.ravel()]).T

        dists, _ = self.global_tree.query(candidates)
        valid_candidates = candidates[dists > SAFE_DISTANCE]

        initial_theta = math.pi if normal_y_dir < 0 else 0.0
        cos_t, sin_t = math.cos(initial_theta), math.sin(initial_theta)

        num_pos, num_neg = len(pts_pos), len(pts_neg)
        total_pts = num_pos + num_neg
        coverage_matrix = np.zeros((len(valid_candidates), total_pts), dtype=bool)

        if num_pos > 0:
            ftheta = initial_theta + FANUC_YAW
            cos_f, sin_f = math.cos(-ftheta), math.sin(-ftheta)
            nx, ny = norms_pos[:, 0] * cos_f - norms_pos[:, 1] * sin_f, norms_pos[:, 0] * sin_f + norms_pos[:,
                                                                                                  1] * cos_f
            nz = norms_pos[:, 2] * FANUC_Z_DIR
            dot_products = -np.stack((nx, ny, nz), axis=-1) @ self.dirs_pos.T
            best_dir_pos = np.argmax(dot_products, axis=-1)
            pre_lx_pos = pts_pos[:, 0] * cos_f - pts_pos[:, 1] * sin_f
            pre_ly_pos = pts_pos[:, 0] * sin_f + pts_pos[:, 1] * cos_f
            lz_pos = (pts_pos[:, 2] - FANUC_GLOBAL_Z) * FANUC_Z_DIR
            idx_z_pos = np.round((lz_pos - self.meta_pos['min_z']) / self.meta_pos['step_z']).astype(int)

        if num_neg > 0:
            ktheta = initial_theta + KUKA_YAW
            cos_k, sin_k = math.cos(-ktheta), math.sin(-ktheta)
            nx, ny = norms_neg[:, 0] * cos_k - norms_neg[:, 1] * sin_k, -(
                        norms_neg[:, 0] * sin_k + norms_neg[:, 1] * cos_k)
            nz = norms_neg[:, 2] * KUKA_Z_DIR
            dot_products = -np.stack((nx, ny, nz), axis=-1) @ self.dirs_neg.T
            best_dir_neg = np.argmax(dot_products, axis=-1)
            pre_lx_neg = pts_neg[:, 0] * cos_k - pts_neg[:, 1] * sin_k
            pre_ly_neg = -(pts_neg[:, 0] * sin_k + pts_neg[:, 1] * cos_k)
            lz_neg = (pts_neg[:, 2] - KUKA_GLOBAL_Z) * KUKA_Z_DIR
            idx_z_neg = np.round((lz_neg - self.meta_neg['min_z']) / self.meta_neg['step_z']).astype(int)

        for i, (cx, cy) in enumerate(valid_candidates):
            if num_pos > 0:
                fx = cx + (FANUC_OFFSET_X * 1000.0) * cos_t - (FANUC_OFFSET_Y * 1000.0) * sin_t
                fy = cy + (FANUC_OFFSET_X * 1000.0) * sin_t + (FANUC_OFFSET_Y * 1000.0) * cos_t
                mask_xy = (np.abs(pts_pos[:, 0] - fx) < 4000) & (np.abs(pts_pos[:, 1] - fy) < 4000)
                if mask_xy.any():
                    v_idx = np.where(mask_xy)[0]
                    offset_lx, offset_ly = fx * cos_f - fy * sin_f, fx * sin_f + fy * cos_f
                    lx, ly = pre_lx_pos[v_idx] - offset_lx, pre_ly_pos[v_idx] - offset_ly
                    idx_x = np.round((lx - self.meta_pos['min_x']) / self.meta_pos['step_x']).astype(int)
                    idx_y = np.round((ly - self.meta_pos['min_y']) / self.meta_pos['step_y']).astype(int)
                    in_bounds = (idx_x >= 0) & (idx_x < self.meta_pos['dim_x']) & (idx_y >= 0) & (
                                idx_y < self.meta_pos['dim_y']) & (idx_z_pos[v_idx] >= 0) & (
                                            idx_z_pos[v_idx] < self.meta_pos['dim_z'])
                    if in_bounds.any():
                        in_v_idx = v_idx[in_bounds]
                        coverage_matrix[i, in_v_idx] = grid_pos[
                            idx_x[in_bounds], idx_y[in_bounds], idx_z_pos[in_v_idx], best_dir_pos[in_v_idx]]

            if num_neg > 0:
                kx = cx + (KUKA_OFFSET_X * 1000.0) * cos_t - (KUKA_OFFSET_Y * 1000.0) * sin_t
                ky = cy + (KUKA_OFFSET_X * 1000.0) * sin_t + (KUKA_OFFSET_Y * 1000.0) * cos_t
                mask_xy = (np.abs(pts_neg[:, 0] - kx) < 4000) & (np.abs(pts_neg[:, 1] - ky) < 4000)
                if mask_xy.any():
                    v_idx = np.where(mask_xy)[0]
                    offset_lx, offset_ly = kx * cos_k - ky * sin_k, -(kx * sin_k + ky * cos_k)
                    lx, ly = pre_lx_neg[v_idx] - offset_lx, pre_ly_neg[v_idx] - offset_ly
                    idx_x = np.round((lx - self.meta_neg['min_x']) / self.meta_neg['step_x']).astype(int)
                    idx_y = np.round((ly - self.meta_neg['min_y']) / self.meta_neg['step_y']).astype(int)
                    in_bounds = (idx_x >= 0) & (idx_x < self.meta_neg['dim_x']) & (idx_y >= 0) & (
                                idx_y < self.meta_neg['dim_y']) & (idx_z_neg[v_idx] >= 0) & (
                                            idx_z_neg[v_idx] < self.meta_neg['dim_z'])
                    if in_bounds.any():
                        in_v_idx = v_idx[in_bounds]
                        coverage_matrix[i, num_pos + in_v_idx] = grid_neg[
                            idx_x[in_bounds], idx_y[in_bounds], idx_z_neg[in_v_idx], best_dir_neg[in_v_idx]]

        uncovered_mask = np.ones(total_pts, dtype=bool)
        selected_bases = []

        while uncovered_mask.any() and len(selected_bases) < MAX_BASES:
            new_coverage_counts = np.count_nonzero(coverage_matrix & uncovered_mask, axis=1)
            best_idx = np.argmax(new_coverage_counts)
            if new_coverage_counts[best_idx] == 0: break
            selected_bases.append(valid_candidates[best_idx])
            uncovered_mask &= ~coverage_matrix[best_idx]

        return np.array(selected_bases)

    def validate_and_save_results(self, agv_xy, agv_theta, df_path_offset):
        K_best = len(agv_xy)
        if K_best == 0: return df_path_offset

        ik_covered_list = []
        bases_f, bases_k = [], []
        for i in range(K_best):
            ax, ay, ath = agv_xy[i, 0], agv_xy[i, 1], agv_theta[i]
            ct, st = math.cos(ath), math.sin(ath)
            fx_val = ax + (FANUC_OFFSET_X * 1000.0) * ct - (FANUC_OFFSET_Y * 1000.0) * st
            fy_val = ay + (FANUC_OFFSET_X * 1000.0) * st + (FANUC_OFFSET_Y * 1000.0) * ct
            bases_f.append((fx_val, fy_val, self.normalize_angle(ath + FANUC_YAW)))

            kx_val = ax + (KUKA_OFFSET_X * 1000.0) * ct - (KUKA_OFFSET_Y * 1000.0) * st
            ky_val = ay + (KUKA_OFFSET_X * 1000.0) * st + (KUKA_OFFSET_Y * 1000.0) * ct
            bases_k.append((kx_val, ky_val, self.normalize_angle(ath + KUKA_YAW)))

        for idx, row in df_path_offset.iterrows():
            pt_global = np.array([row['X'], row['Y'], row['Z']])  # 已经是法兰目标点
            norm_global = np.array([row['I'], row['J'], row['K']])
            covered = 0

            # 按照原始高度进行双臂分工
            if row['Z_orig'] > 3000.0:
                for bx, by, bth in bases_f:
                    if (pt_global[0] - bx) ** 2 + (pt_global[1] - by) ** 2 + (
                            pt_global[2] - FANUC_GLOBAL_Z) ** 2 > ROBOT_REACH ** 2: continue
                    cf, sf = math.cos(-bth), math.sin(-bth)
                    dx, dy, dz = pt_global[0] - bx, pt_global[1] - by, pt_global[2] - FANUC_GLOBAL_Z
                    pt_local_mm = np.array([dx * cf - dy * sf, dx * sf + dy * cf, dz * FANUC_Z_DIR])
                    normal_local = np.array(
                        [norm_global[0] * cf - norm_global[1] * sf, norm_global[0] * sf + norm_global[1] * cf,
                         norm_global[2] * FANUC_Z_DIR])
                    if len(analytical_ik(create_target_pose_from_normal(pt_local_mm, -normal_local),
                                         robot_type='fanuc')) > 0:
                        covered = 1
                        break
            else:
                for bx, by, bth in bases_k:
                    if (pt_global[0] - bx) ** 2 + (pt_global[1] - by) ** 2 + (
                            pt_global[2] - KUKA_GLOBAL_Z) ** 2 > ROBOT_REACH ** 2: continue
                    ck, sk = math.cos(-bth), math.sin(-bth)
                    dx, dy, dz = pt_global[0] - bx, pt_global[1] - by, pt_global[2] - KUKA_GLOBAL_Z
                    pt_local_mm = np.array([dx * ck - dy * sk, -(dx * sk + dy * ck), dz * KUKA_Z_DIR])
                    normal_local = np.array(
                        [norm_global[0] * ck - norm_global[1] * sk, -(norm_global[0] * sk + norm_global[1] * ck),
                         norm_global[2] * KUKA_Z_DIR])
                    if len(analytical_ik(create_target_pose_from_normal(pt_local_mm, -normal_local),
                                         robot_type='kuka')) > 0:
                        covered = 1
                        break

            ik_covered_list.append(covered)

        df_path_offset['IK_Covered'] = ik_covered_list
        return df_path_offset

    def run_calculation_once(self):
        print("==================================================")
        print("⏳ 基于路径约束的贪婪规划计算中，请稍候...")
        print("==================================================")
        try:
            txt_path = os.path.join(self.base_dir, 'point.txt')
            df_full = pd.read_csv(txt_path)
            df_full['Z'] += 3000.0

            # 抽稀原始表面用于轻量化显示 (避免几百万个点把渲染卡死)
            self.df_surface_full = df_full.iloc[::20].copy()

            # 将整个叶片的 XY 存为 KD 树用于 AGV 避障检测
            self.global_xy = df_full[['X', 'Y']].values
            self.global_tree = cKDTree(self.global_xy)

            df_work = df_full[df_full['J'] < 0].copy()
            if df_work.empty: return

            # 1. 抽取连续的作业路径点 (在原点云上)
            df_path_orig = self.generate_surface_paths(df_work, interval=X_SAMPLE_INTERVAL, start_offset=START_OFFSET)
            if df_path_orig.empty: return

            # 2. 对所有路径点进行偏置，获得法兰目标点
            df_path_offset = self.apply_3d_tool_offset(df_path_orig, tcp_offset=[66.5, 88.5, 637.0])

            # ====== 准备贪婪算法所需的数据 ======
            # 根据原始高度分离路径点
            pts_pos_path = df_path_offset[df_path_offset['Z_orig'] > 3000.0][['X', 'Y', 'Z']].values
            pts_neg_path = df_path_offset[df_path_offset['Z_orig'] <= 3000.0][['X', 'Y', 'Z']].values
            norms_pos_path = df_path_offset[df_path_offset['Z_orig'] > 3000.0][['I', 'J', 'K']].values
            norms_neg_path = df_path_offset[df_path_offset['Z_orig'] <= 3000.0][['I', 'J', 'K']].values

            normal_y_dir = -1

            # 3. 对路径点进行贪婪站位搜索
            bases_greedy = self.greedy_initialization(
                pts_pos_path, pts_neg_path, norms_pos_path, norms_neg_path, self.grid_pos, self.grid_neg, normal_y_dir
            )

            if len(bases_greedy) == 0: return

            # 4. 微调优化站位
            agv_xy, agv_theta = self.optimize_agv_poses(
                bases_greedy, pts_pos_path, pts_neg_path, norms_pos_path, norms_neg_path, self.grid_pos, self.grid_neg,
                normal_y_dir
            )

            self.agv_xy = agv_xy

            # 5. 验证实际 IK 覆盖率，并保存
            df_evaluated = self.validate_and_save_results(agv_xy, agv_theta, df_path_offset)
            df_evaluated.to_csv(os.path.join(self.base_dir, '覆盖对比路径点.csv'), index=False)

            # 6. 可视化
            self.visualize_comparison(df_evaluated)

        except Exception as e:
            print(f"❌ 计算发生错误: {e}")

    def visualize_comparison(self, df_path):
        all_points = []
        all_colors = []

        # 1. 🔴 红色：抽稀后的原始叶片表面点云 (作为全量背景)
        if self.df_surface_full is not None:
            surf_pts = self.df_surface_full[['X', 'Y', 'Z']].values
            all_points.append(surf_pts)
            all_colors.append(np.tile([0.8, 0.0, 0.0], (len(surf_pts), 1)))  # 暗红

        # 2. 🟡 黄色：生成的原始路径点 (紧贴在叶片表面上的轨迹)
        path_surf_pts = df_path[['X_orig', 'Y_orig', 'Z_orig']].values
        all_points.append(path_surf_pts)
        all_colors.append(np.tile([1.0, 1.0, 0.0], (len(path_surf_pts), 1)))

        # 3. 🟠 橙色：未能覆盖的工具偏置点 (法兰悬空位置)
        red_df = df_path[df_path['IK_Covered'] == 0]
        if not red_df.empty:
            orange_pts = red_df[['X', 'Y', 'Z']].values
            all_points.append(orange_pts)
            all_colors.append(np.tile([1.0, 0.5, 0.0], (len(orange_pts), 1)))

        # 4. 🟢 绿色：已覆盖的工具偏置点 (不区分蓝绿，统一绿色)
        covered_df = df_path[df_path['IK_Covered'] == 1]
        if not covered_df.empty:
            cov_pts = covered_df[['X', 'Y', 'Z']].values
            all_points.append(cov_pts)
            all_colors.append(np.tile([0.0, 1.0, 0.0], (len(cov_pts), 1)))

        # 5. 🟣 紫色：全局坐标系中心原点
        origin_pts = []
        for dx in range(-50, 50, 10):
            for dy in range(-50, 50, 10):
                for dz in range(-50, 50, 10):
                    origin_pts.append([dx, dy, dz])
        if origin_pts:
            origin_pts = np.array(origin_pts)
            all_points.append(origin_pts)
            all_colors.append(np.tile([0.5, 0.0, 0.5], (len(origin_pts), 1)))

        # 6. 💠 青色：AGV 在地面的站位点
        if self.agv_xy is not None:
            agv_pts = []
            for ax, ay in self.agv_xy:
                for dx in range(-150, 150, 30):
                    for dy in range(-150, 150, 30):
                        agv_pts.append([ax + dx, ay + dy, 0.0])
            if agv_pts:
                agv_pts = np.array(agv_pts)
                all_points.append(agv_pts)
                all_colors.append(np.tile([0.0, 1.0, 1.0], (len(agv_pts), 1)))

        # ======== 最终信息打印 ========
        print("\n" + "=" * 50)
        print("--- 颜色与组件说明 ---")
        print("🟣 紫色: 全局坐标系原点 (0,0,0)")
        print("💠 青色: AGV 站位基座点 (地面)")
        print("🔴 红色: 原始叶片表面轮廓 (仅供对照)")
        print("🟡 黄色: 在表面生成的工艺路径点")
        print("🟠 橙色: 未覆盖的工具偏置点 (法兰位置)")
        print("🟢 绿色: 成功覆盖的工具偏置点 (法兰位置)")
        print(f"📏 辊子路径间隔 : {X_SAMPLE_INTERVAL} mm | 起点偏移: {START_OFFSET} mm")

        total_pts = len(df_path)
        uncovered_pts = len(red_df)
        agv_count = len(self.agv_xy) if self.agv_xy is not None else 0

        print(f"\n🔍 路径点 IK 实际漏涂: {uncovered_pts} 个 (共规划 {total_pts} 个连续路径点)")
        print(f"📍 共选出站位点数量: {agv_count} 个")
        print("=" * 50 + "\n")

        # 启动可视化
        if all_points:
            final_points = np.vstack(all_points)
            final_colors = np.vstack(all_colors)

            p_ik = multiprocessing.Process(target=show_pcd_window,
                                           args=(final_points, final_colors,
                                                 f"【任务驱动规划】双臂连续轨迹分布", 100))
            p_ik.start()
            p_ik.join()


if __name__ == '__main__':
    multiprocessing.freeze_support()
    calculator = CoverageComparisonCalculator()
    calculator.run_calculation_once()