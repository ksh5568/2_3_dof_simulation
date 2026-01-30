import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# === Load CSV data ===
csv_path = "/home/kimsh/ros2_ws/src/my_robot/my_robot/src/2dof_py_log_20250625_135905.csv"
data = pd.read_csv(csv_path)

t_exp = data['time_step'].values
pitch_gazebo = data['pitch_deg'].values
yaw_gazebo = data['yaw_deg'].values

# === System Parameters ===
Jp, Jy = 0.01, 0.015
Dp, Dy = 0.05, 0.07
Ksp, Ksy = 0.5, 0.6
Kpp, Kyy = 1.0, 1.0
Kpy, Kyp = 0.1, 0.1

dt = 0.01
t_interp = np.arange(t_exp[0], t_exp[-1], dt)
n = len(t_interp)

# === Interpolation ===
pitch_gazebo_interp = np.interp(t_interp, t_exp, pitch_gazebo)
yaw_gazebo_interp = np.interp(t_interp, t_exp, yaw_gazebo)

# === State-Space Model ===
def state_space(x, u):
    dx = np.zeros_like(x)
    dx[0] = x[1]
    dx[1] = -(Dp/Jp)*x[1] - (Ksp/Jp)*x[0] + (Kpp/Jp)*u[0] + (Kpy/Jp)*u[1]
    dx[2] = x[3]
    dx[3] = -(Dy/Jy)*x[3] - (Ksy/Jy)*x[2] + (Kyy/Jy)*u[1] + (Kyp/Jy)*u[0]
    return dx

# === Trapezoidal Method ===
def trapezoidal(x, u):
    dx1 = state_space(x, u)
    x_temp = x + dt * dx1
    dx2 = state_space(x_temp, u)
    return x + dt * 0.5 * (dx1 + dx2)

# === Input Signal Generation ===
u_input = np.zeros((n, 2))  # [pitch, yaw]

pitch_peak_times = [57, 114, 153, 162, 202]
pitch_durations = [7, 7, 7, 5, 8]
pitch_K_input = [5.0, -5.0, 5.0, -5.0, -5.0]

yaw_peak_times = [70, 125, 170]
yaw_durations = [5, 6, 4]
yaw_K_input = [-10.0, 10.0, -10.0]

# === Apply pitch input ===
for t_peak, dur, k in zip(pitch_peak_times, pitch_durations, pitch_K_input):
    t_start = t_peak - dur
    for i in range(n):
        if t_start <= t_interp[i] <= t_peak:
            u_input[i, 0] = k

# === Apply yaw input ===
for t_peak, dur, k in zip(yaw_peak_times, yaw_durations, yaw_K_input):
    t_start = t_peak - dur
    for i in range(n):
        if t_start <= t_interp[i] <= t_peak:
            u_input[i, 1] = k

# === Simulation ===
x_trapz = np.zeros((n, 4))
for i in range(1, n):
    x_trapz[i] = trapezoidal(x_trapz[i-1], u_input[i])

# === Evaluation ===
def eval_metrics(pred, true):
    rmse = np.sqrt(mean_squared_error(true, pred))
    corr = np.corrcoef(pred, true)[0, 1]
    return rmse, corr

pitch_rmse, pitch_corr = eval_metrics(x_trapz[:, 0], pitch_gazebo_interp)
yaw_rmse, yaw_corr = eval_metrics(x_trapz[:, 2], yaw_gazebo_interp)

# === Plot: Pitch ===
plt.figure(figsize=(12, 5))
plt.plot(t_interp, pitch_gazebo_interp, 'r-', label='Gazebo Pitch')
plt.plot(t_interp, x_trapz[:, 0], 'g--', label='Sim Pitch')
plt.xlabel("Time [s]")
plt.ylabel("Pitch Angle [deg]")
plt.title(f"Pitch Angle Response\nRMSE: {pitch_rmse:.4f}, Corr: {pitch_corr:.4f}")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# === Plot: Yaw ===
plt.figure(figsize=(12, 5))
plt.plot(t_interp, yaw_gazebo_interp, 'r-', label='Gazebo Yaw')
plt.plot(t_interp, x_trapz[:, 2], 'b--', label='Sim Yaw')
plt.xlabel("Time [s]")
plt.ylabel("Yaw Angle [deg]")
plt.title(f"Yaw Angle Response\nRMSE: {yaw_rmse:.4f}, Corr: {yaw_corr:.4f}")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
