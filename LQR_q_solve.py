import numpy as np

import scipy.linalg as linalg

import math


# --- 1. Define Physical Parameters ---

# Values from your screenshots and model

M = 0.248 # Mass of the body (kg)
import numpy as np
import scipy.linalg as linalg


# --- 1. Define Physical Parameters ---

# Values from your screenshots and model

M = 0.248  # Mass of the body (kg)
m_w = 0.018  # Mass of one wheel (kg)
M_total = M + (2 * m_w)  # Total mass of the robot cart
m = 0.248  # Mass of the pendulum body (kg)

I_massless = 0.00133
I = I_massless * m  # Real Moment of Inertia (kg*m^2)


# --- 2. YOUR CRITICAL TUNING PARAMETERS ---

l = 0.0285  # (m) Effective length to Center of Mass. TUNE THIS.
r = 0.022  # (m) Wheel radius. MUST BE ACCURATE.

g = 9.81
b = 0.0  # Damping (set to 0 per your request)


# --- 3. NEW: Motor Model (Velocity Control) ---
# We assume Torque = k_motor * u_velocity
# Let's define the limits of our motor command

max_torque = 2.5  # (Nm)
max_velocity_cmd = 10.0  # (rad/s) <-- NEW TUNING PARAMETER

# This is our guess for the "max speed" of the motor controller.

# Calculate the proportional gain of our motor model
k_motor = max_torque / max_velocity_cmd

print("--- LQR Gain Calculator (VELOCITY Model) ---")
print(f"Motor Model Gain (k_motor): {k_motor:.4f}")


# --- 4. System Matrices (A, B) ---
# Denominator term
p = I * (M_total + m) + M_total * m * l ** 2


# State-space matrix A (same as before)
A = np.array([
	[0, 1, 0, 0],
	[0, -((I + m * l ** 2) * b) / p, (m ** 2 * g * l ** 2) / p, 0],
	[0, 0, 0, 1],
	[0, - (m * l * b) / p, m * g * l * (M_total + m) / p, 0]
])


# B matrix for FORCE
B_force = np.array([
	[0],
	[(I + m * l ** 2) / p],
	[0],
	[(m * l) / p]
])


# NEW B matrix for VELOCITY
# Force = Torque / r = (k_motor * u_vel) / r
# B_vel = B_force * (k_motor / r)
B_vel = B_force * (k_motor / r)


# --- 5. LQR Cost Matrices (Q, R) ---
# Q: Penalizes state error [x, x_dot, theta, theta_dot]
Q = np.diag([
	300.0,  # Penalize x (position) - Keep this low
	20.0,  # Penalize x_dot (velocity)
	500.0,  # Penalize theta (angle) - Keep this HIGH
	15.0  # Penalize theta_dot (angular vel) - Damping
])


# R: Penalizes control input (u_vel)
# Normalize R based on our max velocity command
R_val = 1.0 / (max_velocity_cmd ** 2)
R = np.array([[R_val]])


# --- 6. Solve LQR ---
try:
	# Solve continuous-time LQR (Continuous Algebraic Riccati Equation)
	P = linalg.solve_continuous_are(A, B_vel, Q, R)

	# Calculate gain matrix K
	K = np.linalg.inv(R) @ B_vel.T @ P

	print("\nSUCCESS! Your new K matrix (for VELOCITY control) is:\n")
	print(f"self.K = np.array([[{K[0,0]:.4f}, {K[0,1]:.4f}, {K[0,2]:.4f}, {K[0,3]:.4f}]])")
	print("\nPaste this line into your Part 2 (CoppeliaSim) script.")

except Exception as e:
	print("\n--- LQR Calculation FAILED ---")
	print(f"Error: {e}")