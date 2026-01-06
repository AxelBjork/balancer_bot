import math

# ==========================================
# 1. Physics Parameters (from balancer_simulator.h & User)
# ==========================================
m = 1.032       # kg
l = 0.06        # m (COM height)
g = 9.81        # m/s^2

# Inertia: Point Mass Model
I = m * l**2    # kg m^2

print(f"Physics Config:\n  Mass={m}\n  COM_Height={l}\n  Inertia={I:.7f}")

# ==========================================
# 2. Control Layout & Gains (from config_pid.h)
# ==========================================
# Gains
K_angle = 12.0        # Angle -> Rate
Kp_rate = 0.75        # Rate P gain (User Request)
Kd_rate = 0.6        # Rate D gain (Restored)

# Motor conversion gain
# pid out 1.0 -> 3200 sps.
# -3200 sps -> Forward Velocity (Positive m/s).
# r = 0.04m.
# v = (-u_sps / 3200) * (2 * pi * r)
# G_motor = -1.0 * (2 * pi * r) = -0.2513
G_motor = -0.2513  # (m/s) / (PID_unit)

# Effective Gains
# v = G_motor * [ Kp_rate * (rate_sp - dtheta) + Kd_rate * d/dt(rate_sp - dtheta) ]
# rate_sp = -K_angle * theta
# v = G_motor * [ -Kp_rate*K_angle * theta - (Kp_rate + Kd_rate*K_angle) * dtheta - Kd_rate * ddtheta ]

K_p_eff = G_motor * (-Kp_rate * K_angle)
K_d_eff = G_motor * (-1.0 * (Kp_rate + Kd_rate * K_angle))
K_dd_eff = G_motor * (-1.0 * Kd_rate)

# Wait, sign logic check in derivation:
# v = -Kp_eff * theta - Kd_eff * dtheta ...
# My G_motor includes the negative sign.
# So K_p_eff should be Positive number (stiffness).
# G (-0.25) * (-Kp Kangle) = Positive. Correct.
# K_d_eff = G * (-...) = Positive. Correct.

print(f"Effective Control Stiffness/Damping:\n  Kp_theta={K_p_eff:.4f}\n  Kd_theta={K_d_eff:.4f}\n  Kdd_theta={K_dd_eff:.4f}")

# ==========================================
# 3. System Dynamics & Characteristic Equation
# ==========================================
# Plant: I theta'' - m g l theta = - m l v'
# Loop: v = -K_p_eff theta - K_d_eff theta' - K_dd_eff theta''
# v' = -K_p_eff theta' - K_d_eff theta'' - K_dd_eff theta'''
#
# I theta'' - m g l theta = - m l [ -K_p_eff theta' - K_d_eff theta'' - K_dd_eff theta''' ]
# I theta'' - m g l theta = m l K_p_eff theta' + m l K_d_eff theta'' + m l K_dd_eff theta'''
#
# Rearrange to LHS:
# - (m l K_dd_eff) theta''' + (I - m l K_d_eff) theta'' - (m l K_p_eff) theta' - (m g l) theta = 0
#
# Multiply by -1 to make s^3 term positive (since K_dd_eff > 0):
# (m l K_dd_eff) s^3 + (m l K_d_eff - I) s^2 + (m l K_p_eff) s + (m g l) = 0

K1 = m * l
K2 = m * g * l

a3 = K1 * K_dd_eff
a2 = K1 * K_d_eff - I
a1 = K1 * K_p_eff
a0 = K2

print(f"\nCharacteristic Eq Coeffs:\n  s^3: {a3:.6f}\n  s^2: {a2:.6f}\n  s^1: {a1:.6f}\n  s^0: {a0:.6f}")

# ==========================================
# 4. Stability Check (Routh-Hurwitz)
# ==========================================
print("\n--- Stability Analysis ---")
stable = True

# 1. All coeffs must have same sign (Positive here)
if a3 <= 0:
    print(f"[FAIL] s^3 coeff <= 0. Val={a3}")
    stable = False
if a2 <= 0:
    print(f"[FAIL] s^2 coeff <= 0. Val={a2}")
    print("       (Damping term < Inertia term). Increase D gains.")
    stable = False
if a1 <= 0:
    print(f"[FAIL] s^1 coeff <= 0. Val={a1}")
    stable = False
if a0 <= 0:
    print(f"[FAIL] s^0 coeff <= 0. Val={a0}")
    stable = False

# 2. Routh Conditions
# s3 | a3  a1
# s2 | a2  a0
# s1 | b1
# b1 = (a2 a1 - a3 a0) / a2
# Need b1 > 0

if a2 != 0:
    b1 = (a2 * a1 - a3 * a0) / a2
    print(f"Routh b1 value: {b1:.6f}")
    if b1 <= 0:
        print("[FAIL] Routh condition b1 > 0 failed (Oscillatory instability).")
        stable = False
else:
    print("[FAIL] a2 is zero.")
    stable = False

print("\n--- Sweeping K_angle for P=0.75 ---")
for k_test in [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 25.0, 30.0]:
    K_angle = k_test
    # Re-calc coeffs
    K_p_eff = G_motor * (-Kp_rate * K_angle)
    K_d_eff = G_motor * (-1.0 * (Kp_rate + Kd_rate * K_angle))
    K_dd_eff = G_motor * (-1.0 * Kd_rate)
    
    a3 = K1 * K_dd_eff
    a2 = K1 * K_d_eff - I
    a1 = K1 * K_p_eff
    a0 = K2
    
    # Check Stability
    stable_local = True
    if a3<=0 or a2<=0 or a1<=0 or a0<=0: stable_local = False
    
    b1_val = -100.0
    if a2 != 0:
        b1_val = (a2 * a1 - a3 * a0) / a2
        if b1_val <= 0: stable_local = False
        
    print(f"K={k_test:4.1f} | b1={b1_val:.4f} | {'STABLE' if stable_local else 'UNSTABLE'} | a1={a1:.2f}")
