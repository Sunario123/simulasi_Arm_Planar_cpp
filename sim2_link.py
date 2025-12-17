import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =============================
# PARAMETER ROBOT
# =============================
l1 = 0.5
l2 = 1.5

theta1 = np.deg2rad(60)
theta2 = np.deg2rad(30)

thetaDot1 = 0.0
thetaDot2 = 0.0

# =============================
# TARGET & KONTROL
# =============================
xd, yd = 0.5, 1.5
K = 2.0
dt = 0.05
eps = 1e-3

# =============================
# JACOBIAN
# =============================
def hitungKiriAtas():
    return -l1*np.sin(theta1) - l2*np.sin(theta1 + theta2)

def hitungKiriBawah():
    return l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)

def hitungKananAtas():
    return -l2*np.sin(theta1 + theta2)

def hitungKananBawah():
    return l2*np.cos(theta1 + theta2)

# =============================
# FORWARD KINEMATICS
# =============================
def forwardKinematics():
    x = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
    y = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)
    return x, y

# =============================
# RESOLVED RATE CONTROL
# =============================
def resolved_rate_step():
    global theta1, theta2, thetaDot1, thetaDot2

    x, y = forwardKinematics()
    ex = xd - x
    ey = yd - y
    error = np.sqrt(ex**2 + ey**2)

    if error < eps:
        return

    xDot = K * ex
    yDot = K * ey

    detJ = (hitungKiriAtas()*hitungKananBawah()
          - hitungKananAtas()*hitungKiriBawah())

    if abs(detJ) < 1e-6:
        print("Jacobian singular!")
        return

    thetaDot1 = (hitungKananBawah()*xDot
               - hitungKananAtas()*yDot) / detJ

    thetaDot2 = (-hitungKiriBawah()*xDot
               + hitungKiriAtas()*yDot) / detJ

    theta1 += thetaDot1 * dt
    theta2 += thetaDot2 * dt

# =============================
# VISUALISASI
# =============================
fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_aspect('equal')
ax.grid()

line, = ax.plot([], [], 'o-', lw=4)
target, = ax.plot(xd, yd, 'rx', markersize=10)

def update(frame):
    resolved_rate_step()

    x1 = l1*np.cos(theta1)
    y1 = l1*np.sin(theta1)

    x2 = x1 + l2*np.cos(theta1 + theta2)
    y2 = y1 + l2*np.sin(theta1 + theta2)

    line.set_data([0, x1, x2], [0, y1, y2])
    return line,

ani = FuncAnimation(fig, update, frames=400, interval=50)
plt.show()
