import numpy as np
import matplotlib.pyplot as plt

# Link lengths
L1, L2, L3 = 1.5, 1.5, 0.3

def forward_kinematics(theta1, theta2, theta3):
    """Compute joint positions from angles"""
    # Shoulder
    x1, y1 = L1*np.cos(theta1), L1*np.sin(theta1)
    # Elbow
    x2, y2 = x1 + L2*np.cos(theta1+theta2), y1 + L2*np.sin(theta1+theta2)
    # Wrist / End effector
    x3, y3 = x2 + L3*np.cos(theta1+theta2+theta3), y2 + L3*np.sin(theta1+theta2+theta3)
    return np.array([[0,0],[x1,y1],[x2,y2],[x3,y3]])

def inverse_kinematics(x, y, phi):
    """Solve IK for 2-link arm + wrist orientation
       x,y = target position
       phi = desired end effector orientation (angle)"""
    # Wrist position (subtract link3 along phi direction)
    wx = x - L3*np.cos(phi)
    wy = y - L3*np.sin(phi)

    # Distance from base to wrist
    d = np.sqrt(wx**2 + wy**2)

    # Law of cosines for elbow
    cos_elbow = (d**2 - L1**2 - L2**2)/(2*L1*L2)
    cos_elbow = np.clip(cos_elbow, -1, 1)   # numerical safety
    elbow = -(np.arccos(cos_elbow))            # pick elbow-down solution

    # Shoulder
    k1 = L1 + L2*np.cos(elbow)
    k2 = L2*np.sin(elbow)
    shoulder = np.arctan2(wy, wx) - np.arctan2(k2, k1)

    # Wrist
    wrist = phi - (shoulder + elbow)

    return shoulder, elbow, wrist

def plot_arm(theta, target, phi):
    pts = forward_kinematics(*theta)
    plt.figure()
    plt.plot(pts[:,0], pts[:,1], 'ko-', lw=2)  # black links
    plt.scatter(*target, c='r', s=80, label='Target')
    plt.scatter(pts[-1,0], pts[-1,1], c='b', s=80, label='End effector')
    # draw orientation line of link3
    x_end, y_end = pts[-1]
    plt.plot([x_end, x_end+0.5*np.cos(phi)], [y_end, y_end+0.5*np.sin(phi)], 'g-', lw=2, label='Orientation')
    plt.axis('equal'); plt.grid(); plt.legend(); plt.show()

# Example usage
target_pos = (2.5, 0.0)
desired_phi = np.deg2rad(-30)  # desired orientation = 45°
shoulder, elbow, wrist = inverse_kinematics(*target_pos, desired_phi)

print("Angles (rad):", shoulder, elbow, wrist)
plot_arm((shoulder, elbow, wrist), target_pos, desired_phi)
