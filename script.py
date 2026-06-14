import math
import numpy as np

# --- Global Variables ---
sim = None
body = None
left_joint = None
right_joint = None
dt = 0.01

# LQR & Control Variables
K = None
x = 0.0
x_dot = 0.0
theta = 0.0
theta_dot = 0.0
theta_dot_filtered = 0.0

# Setpoints & Inputs
x_setpoint = 0.0
current_turn_velocity = 0.0
max_velocity = 20.0
goal_speed_mps = 3.5
turn_velocity = 3.0

# Tuning constants
ALPHA = 0.7
LEAK  = 0.998
MAX_POS = 5.0
MAX_LEAN_FOR_TURN = 0.5
MAX_LEAN_ANGLE = 0.3        # ~17 deg — blocks new setpoint commands beyond this
BLEED_THRESHOLD = 0.2       # ~11 deg — starts bleeding setpoint back above this
BLEED_RATE = 0.3            # setpoint pulled back this many units/sec when leaning


def sysCall_init():
    global sim, body, left_joint, right_joint, dt, K

    sim = require('sim')

    try:
        body        = sim.getObject('/body')
        left_joint  = sim.getObject('/body/left_joint')
        right_joint = sim.getObject('/body/right_joint')
    except Exception as e:
        sim.addLog(sim.verbosity_errors,
                   "Error finding objects. Check hierarchy: body -> left_joint / right_joint")
        raise e

    dt = sim.getSimulationTimeStep()

    K = np.array([[-141.4214, -87.1545, 253.5076, 11.3576]])


def sysCall_sensing():
    global x, x_dot, theta, theta_dot, theta_dot_filtered
    global x_setpoint, current_turn_velocity

    # --- A. STATE UPDATE ---
    try:
        lin_vel_world, ang_vel_world = sim.getObjectVelocity(body)

        m = sim.getObjectMatrix(body, -1)
        m[3] = 0; m[7] = 0; m[11] = 0
        m_inv = sim.getMatrixInverse(m)

        local_lin_vel = sim.multiplyVector(m_inv, lin_vel_world)
        local_ang_vel = sim.multiplyVector(m_inv, ang_vel_world)

        x_dot = local_lin_vel[1]
        x = (x + x_dot * dt) * LEAK

        world_up = [0, 0, 1]
        local_up = sim.multiplyVector(m_inv, world_up)
        theta = math.atan2(local_up[1], local_up[2])

        theta_dot_raw      = local_ang_vel[0]
        theta_dot_filtered = ALPHA * theta_dot_raw + (1.0 - ALPHA) * theta_dot_filtered
        theta_dot          = theta_dot_filtered

    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"[sensing] {type(e).__name__}: {e}")

    # --- B. KEYBOARD INPUT ---
    current_turn_velocity = 0.0
    increment = goal_speed_mps * dt

    while True:
        message, data, data2 = sim.getSimulatorMessage()
        if message == -1:
            break
        if message == sim.message_keypress:
            if data[0] == 2007:   # Up
                if theta < MAX_LEAN_ANGLE:
                    x_setpoint += increment
            if data[0] == 2008:   # Down
                if theta > -MAX_LEAN_ANGLE:
                    x_setpoint -= increment
            if data[0] == 2009:   # Left
                current_turn_velocity = turn_velocity
            if data[0] == 2010:   # Right
                current_turn_velocity = -turn_velocity

    # Bleed x_setpoint back toward 0 when leaning beyond safe threshold
    # Prevents accumulated setpoint from compounding lean angle instability
    if abs(theta) > BLEED_THRESHOLD:
        bleed = BLEED_RATE * dt * math.copysign(1.0, x_setpoint)
        if abs(x_setpoint - bleed) < abs(x_setpoint):
            x_setpoint -= bleed

    x_setpoint = max(min(x_setpoint, MAX_POS), -MAX_POS)


def sysCall_actuation():
    global x, x_dot, theta, theta_dot, x_setpoint, current_turn_velocity

    try:
        x_state  = np.array([[x], [x_dot], [theta], [theta_dot]])
        x_target = np.array([[x_setpoint], [0.0], [0.0], [0.0]])

        e = x_state - x_target
        u = -K @ e

        vel = float(u[0])
        vel = max(min(vel, max_velocity), -max_velocity)

        turn_scale = max(0.0, 1.0 - abs(theta) / MAX_LEAN_FOR_TURN)
        v_left  = vel + current_turn_velocity * turn_scale
        v_right = vel - current_turn_velocity * turn_scale

        if not (math.isfinite(v_left) and math.isfinite(v_right)):
            sim.addLog(sim.verbosity_errors,
                       "[actuation] Non-finite motor command — stopping")
            v_left = v_right = 0.0

        sim.setJointTargetVelocity(left_joint,  v_left)
        sim.setJointTargetVelocity(right_joint, v_right)

    except Exception as e:
        sim.addLog(sim.verbosity_errors, f"[actuation] {type(e).__name__}: {e}")


def sysCall_cleanup():
    try:
        sim.setJointTargetVelocity(left_joint,  0)
        sim.setJointTargetVelocity(right_joint, 0)
    except:
        pass