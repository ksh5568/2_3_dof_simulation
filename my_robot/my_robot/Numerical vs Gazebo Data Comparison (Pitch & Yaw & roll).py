import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error
from scipy.signal import find_peaks

# Load CSV data
file_path = "/home/kimsh/ros2_ws/src/my_robot/my_robot/src/3dof_rpy_log_20250625_190936.csv"
df = pd.read_csv(file_path)

# Extract relevant columns
t_exp = df['time_step'].values
roll_exp = df['roll_deg'].values
pitch_exp = df['pitch_deg'].values
yaw_exp = df['yaw_deg'].values

# Simulation parameters
Jr, Jp, Jy = 0.012, 0.01, 0.015
Dr, Dp, Dy = 0.04, 0.05, 0.07
Ksr, Ksp, Ksy = 0.45, 0.5, 0.6
Krr, Kpp, Kyy = 1.0, 1.0, 1.0
dt = 0.01

# Interpolate time
t_interp = np.arange(t_exp[0], t_exp[-1], dt)
n = len(t_interp)
roll_interp = np.interp(t_interp, t_exp, roll_exp)
pitch_interp = np.interp(t_interp, t_exp, pitch_exp)
yaw_interp = np.interp(t_interp, t_exp, yaw_exp)

# Find top 5 absolute peaks in both directions
def find_top_peaks_signed(signal, time_array, num_peaks=5):
    peaks_pos, _ = find_peaks(signal, distance=100)
    peaks_neg, _ = find_peaks(-signal, distance=100)
    all_peaks = np.concatenate((peaks_pos, peaks_neg))
    top_indices = np.argsort(np.abs(signal[all_peaks]))[::-1][:num_peaks]
    return sorted(time_array[all_peaks][top_indices])

roll_peak_times = find_top_peaks_signed(roll_exp, t_exp)
pitch_peak_times = find_top_peaks_signed(pitch_exp, t_exp)
yaw_peak_times = find_top_peaks_signed(yaw_exp, t_exp)

# Create input signal
u_input = np.zeros((n, 3))  # roll, pitch, yaw inputs
K_input = 5.0
duration = 5

for t_peak in roll_peak_times:
    t_start = t_peak - duration
    for i in range(n):
        if t_start <= t_interp[i] <= t_peak:
            direction = np.sign(np.interp(t_peak, t_exp, roll_exp))
            u_input[i, 0] = direction * K_input

for t_peak in pitch_peak_times:
    t_start = t_peak - duration
    for i in range(n):
        if t_start <= t_interp[i] <= t_peak:
            direction = np.sign(np.interp(t_peak, t_exp, pitch_exp))
            u_input[i, 1] = direction * K_input

for t_peak in yaw_peak_times:
    t_start = t_peak - duration
    for i in range(n):
        if t_start <= t_interp[i] <= t_peak:
            direction = np.sign(np.interp(t_peak, t_exp, yaw_exp))
            u_input[i, 2] = direction * K_input

# 3DOF state-space model
def state_space_3dof(x, u):
    dx = np.zeros_like(x)
    dx[0] = x[1]
    dx[1] = -(Dr/Jr)*x[1] - (Ksr/Jr)*x[0] + (Krr/Jr)*u[0]
    dx[2] = x[3]
    dx[3] = -(Dp/Jp)*x[3] - (Ksp/Jp)*x[2] + (Kpp/Jp)*u[1]
    dx[4] = x[5]
    dx[5] = -(Dy/Jy)*x[5] - (Ksy/Jy)*x[4] + (Kyy/Jy)*u[2]
    return dx

def trapezoidal_3dof(x, u):
    dx1 = state_space_3dof(x, u)
    x_temp = x + dt * dx1
    dx2 = state_space_3dof(x_temp, u)
    return x + dt * 0.5 * (dx1 + dx2)

# Run simulation
x_sim = np.zeros((n, 6))
for i in range(1, n):
    x_sim[i] = trapezoidal_3dof(x_sim[i-1], u_input[i])

# Evaluation
def evaluate(pred, true):
    rmse = np.sqrt(mean_squared_error(true, pred))
    std_pred = np.std(pred)
    std_true = np.std(true)
    if std_pred == 0 or std_true == 0:
        corr = 0.0
    else:
        corr = np.corrcoef(pred, true)[0, 1]
    return rmse, corr

roll_rmse, roll_corr = evaluate(x_sim[:, 0], roll_interp)
pitch_rmse, pitch_corr = evaluate(x_sim[:, 2], pitch_interp)
yaw_rmse, yaw_corr = evaluate(x_sim[:, 4], yaw_interp)

# Evaluation DataFrame
eval_df = pd.DataFrame({
    'Axis': ['Roll', 'Pitch', 'Yaw'],
    'RMSE': [roll_rmse, pitch_rmse, yaw_rmse],
    'Correlation': [roll_corr, pitch_corr, yaw_corr]
})

# Plotting
plt.figure(figsize=(14, 8))

plt.subplot(3, 1, 1)
plt.plot(t_interp, roll_interp, label="Gazebo Roll", color='red')
plt.plot(t_interp, x_sim[:, 0], label="Sim Roll", linestyle='--', color='blue')
plt.title("Roll Comparison")
plt.ylabel("Roll [deg]")
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(t_interp, pitch_interp, label="Gazebo Pitch", color='red')
plt.plot(t_interp, x_sim[:, 2], label="Sim Pitch", linestyle='--', color='green')
plt.title("Pitch Comparison")
plt.ylabel("Pitch [deg]")
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(t_interp, yaw_interp, label="Gazebo Yaw", color='red')
plt.plot(t_interp, x_sim[:, 4], label="Sim Yaw", linestyle='--', color='orange')
plt.title("Yaw Comparison")
plt.xlabel("Time [s]")
plt.ylabel("Yaw [deg]")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
