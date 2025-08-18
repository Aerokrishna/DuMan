import numpy as np
import matplotlib.pyplot as plt

# ===== Parameters =====
link_lengths = [0.15, 0.2]  # lengths of the two links
joint_limits = [
    (np.pi/2, np.pi),  # joint 1 range
    (-0.75 * np.pi, 0)     # joint 2 range
]
shoulder_dist = 0.3  # meters
num_samples = 5000   # per arm

# ===== Forward Kinematics =====
def fk_planar(j1, j2, link_lengths):
    l1, l2 = link_lengths
    x = l1 * np.cos(j1) + l2 * np.cos(j1 + j2)
    y = l1 * np.sin(j1) + l2 * np.sin(j1 + j2)
    return x, y

# ===== Sample joint space =====
def sample_workspace(link_lengths, joint_limits, n_samples):
    j1 = np.random.uniform(joint_limits[0][0], joint_limits[0][1], n_samples)
    j2 = np.random.uniform(joint_limits[1][0], joint_limits[1][1], n_samples)
    xs, ys = fk_planar(j1, j2, link_lengths)
    return np.vstack((xs, ys)).T

# ===== Main =====
points_A = sample_workspace(link_lengths, joint_limits, num_samples)
points_A_global = points_A + np.array([-shoulder_dist / 2, 0])

points_B_global = points_A.copy()
points_B_global[:, 0] = -points_B_global[:, 0]
points_B_global += np.array([shoulder_dist / 2, 0])

# Intersection points
epsilon = 0.02
intersection_points = []
for p in points_A_global:
    if np.any(np.linalg.norm(points_B_global - p, axis=1) < epsilon):
        intersection_points.append(p)
intersection_points = np.array(intersection_points)

# ===== Plot workspace =====
plt.figure(figsize=(8, 8))
plt.scatter(points_A_global[:, 0], points_A_global[:, 1], s=1, color='blue', alpha=0.5, label='Arm A')
plt.scatter(points_B_global[:, 0], points_B_global[:, 1], s=1, color='red', alpha=0.5, label='Arm B')
if len(intersection_points) > 0:
    plt.scatter(intersection_points[:, 0], intersection_points[:, 1], s=3, color='purple', label='Intersection')

# Shoulder positions (green)
shoulder_positions = np.array([
    [-shoulder_dist / 2, 0],
    [ shoulder_dist / 2, 0]
])
plt.scatter(shoulder_positions[:, 0], shoulder_positions[:, 1], s=80, color='green', label='Shoulders')

# ===== Draw fixed configuration links =====
j1_fixed = np.deg2rad(120)
j2_fixed = np.deg2rad(-135)

def draw_arm(ax, shoulder_pos, mirror=False):
    if mirror:
        j1 = np.pi - j1_fixed   # mirror joint 1
        j2 = -j2_fixed          # mirror joint 2
    else:
        j1 = j1_fixed
        j2 = j2_fixed
    l1, l2 = link_lengths
    joint1 = shoulder_pos + np.array([l1 * np.cos(j1), l1 * np.sin(j1)])
    end_eff = shoulder_pos + np.array([l1 * np.cos(j1) + l2 * np.cos(j1 + j2),
                                       l1 * np.sin(j1) + l2 * np.sin(j1 + j2)])
    ax.plot([shoulder_pos[0], joint1[0]], [shoulder_pos[1], joint1[1]], 'k-', linewidth=2)
    ax.plot([joint1[0], end_eff[0]], [joint1[1], end_eff[1]], 'k-', linewidth=2)

draw_arm(plt, shoulder_positions[0], mirror=False)
draw_arm(plt, shoulder_positions[1], mirror=True)

plt.axvline(0, color='gray', linestyle='--', linewidth=1)
plt.gca().set_aspect('equal', adjustable='box')
plt.title(f"Dual Arm Planar Workspace (shoulder distance = {shoulder_dist} m)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend()
plt.show()
