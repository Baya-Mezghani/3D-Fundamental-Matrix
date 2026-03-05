# ==========================================================
# 3D Object – Fundamental Matrix Estimation from Two Views
# ==========================================================
# Author: Baya Mezghani
# M2 Data Science
# ==========================================================

import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Load 3D points from OBJ
# -----------------------------
def lire_obj(filename):
    vertices = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                x, y, z = map(float, line.strip().split()[1:4])
                vertices.append([x, y, z])
    return np.array(vertices)


# -----------------------------
# Camera Intrinsic Matrix
# -----------------------------
def matrice_intr(fx, fy, skew, cx, cy):
    return np.array([[fx, skew, cx],
                     [0, fy, cy],
                     [0, 0, 1]])


# -----------------------------
# Camera Extrinsic Matrix
# -----------------------------
def matrice_extr(rx_deg, ry_deg, rz_deg, tx, ty, tz):
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    t = np.array([[tx], [ty], [tz]])
    return np.hstack((R, t))


# -----------------------------
# Project 3D points to 2D
# -----------------------------
def projeter_points(points_3D, P):
    N = len(points_3D)
    homog = np.hstack((points_3D, np.ones((N, 1))))
    proj = (P @ homog.T).T
    x = proj[:, 0] / proj[:, 2]
    y = proj[:, 1] / proj[:, 2]
    return np.vstack((x, y)).T


def normalize_points(pts):
    """
    Normalize 2D points for better numerical stability.
    """
    centroid = np.mean(pts, axis=0)
    pts_centered = pts - centroid
    scale = np.sqrt(2) / np.mean(np.linalg.norm(pts_centered, axis=1))
    T = np.array([[scale, 0, -scale*centroid[0]],
                  [0, scale, -scale*centroid[1]],
                  [0, 0, 1]])
    pts_h = np.hstack((pts, np.ones((pts.shape[0],1))))
    pts_norm = (T @ pts_h.T).T
    return pts_norm[:, :2], T

def estimate_fundamental_matrix(pts1, pts2):
    """
    Estimate fundamental matrix F from two sets of 2D points.
    Uses normalization but does NOT enforce rank manually.
    """
    n = pts1.shape[0]
    if n < 20:
        raise ValueError(f"At least 20 correspondences required, got {n}")

    # Normalize points
    pts1_norm, T1 = normalize_points(pts1)
    pts2_norm, T2 = normalize_points(pts2)

    # Build linear system
    A = np.zeros((n,9))
    for i in range(n):
        x1, y1 = pts1_norm[i]
        x2, y2 = pts2_norm[i]
        A[i] = [x1*x2, x1*y2, x1,
                y1*x2, y1*y2, y1,
                x2, y2, 1]

    _, _, Vt = np.linalg.svd(A)
    F_norm = Vt[-1].reshape(3,3)

    # Denormalize
    F = T2.T @ F_norm @ T1
    return F


# -----------------------------
# Main pipeline
# -----------------------------
if __name__ == "__main__":
    # Load 3D points
    points_3D = lire_obj("data/Wooden chair.obj")
    points_3D = (points_3D - np.mean(points_3D, axis=0)) / np.max(np.linalg.norm(points_3D, axis=1))
    print(f"{len(points_3D)} points loaded.")

    # Camera 1
    K1 = matrice_intr(800, 800, 0, 320, 240)
    Rt1 = matrice_extr(10, 30, 0, 0, 0, 3)
    M1 = K1 @ Rt1

    # Camera 2 (different pose)
    K2 = matrice_intr(800, 800, 0, 320, 240)
    Rt2 = matrice_extr(20, 10, 5, 0.1, 0, 3)
    M2 = K2 @ Rt2

    # Project 3D points
    pts2D_1 = projeter_points(points_3D, M1)
    pts2D_2 = projeter_points(points_3D, M2)

    # Random 20 correspondences
    idx = np.random.choice(len(points_3D), 20, replace=False)
    corr_1 = pts2D_1[idx]
    corr_2 = pts2D_2[idx]

    # Estimate Fundamental Matrix
    F_est = estimate_fundamental_matrix(corr_1, corr_2)
    print("\nEstimated Fundamental Matrix F:\n", F_est)