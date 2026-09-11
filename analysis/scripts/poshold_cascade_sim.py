#!/usr/bin/env python3
"""
poshold_cascade_sim.py — time-domain, firmware-exact reproduction of the FULL
4-stage POS_HOLD cascade (position -> velocity -> attitude -> rate -> plant),
to check whether the persistent ~1.73-1.76s-period position/attitude
oscillation seen in SILS (pos_gain_deficit_smallnudge_hold40.scn, nominal
conditions) is reproducible from the individually-healthy per-loop margins
found by acro_gain_rate_loop_margins.py and the velocity-loop-wraps-attitude-
loop analysis in docs/plans/smc-rate-loop-plan.md §7.30.

位置ホールドの持続振動（約1.73-1.76秒周期）の全4段カスケード時間領域再現。

Unlike poshold_loop_design.py (which collapses attitude+rate into a single
gain*delay "tilt_cmd -> accel" plant), this simulates the REAL rate loop
(torque -> motor lag -> dead time -> I*angular_accel -> rate) and the REAL
attitude loop (rate_sp = PID(roll_sp, roll_meas)) explicitly, using the same
firmware-exact discrete PID class (trapezoidal integral, conditional-
integration anti-windup, incomplete-derivative filter) and the SAME
calibrated rate-loop plant constants already validated in
acro_gain_rate_loop_margins.py (I_roll, tau_m, L, eta_t -- chosen there so
the CURRENT roll rate-loop gains reproduce a real-flight-plausible PM~55deg).

Roll axis only (the disturbed axis in the SILS scenario this investigation
uses). Current production gains throughout (params.cpp, confirmed 2026-09-11):
  rate.roll   kp=1.0e-3   ti=0.7  td=0.002
  attitude.roll kp=5.0    ti=2.0  td=0.04
  position.vel  kp=3.0    ti=2.0  td=0.0
  position.pos  kp=0.4    ti=5.0  td=0.0
"""
import numpy as np

G = 9.80665
DT = 0.0025                  # 400 Hz control rate (matches firmware)
MAX_POS_VEL = 1.0            # position-loop output limit [m/s] (params.cpp MAX_POS_VEL)
MAX_POS_TILT = 0.1745        # tilt clamp [rad] (10 deg, computePositionHold's max_pos_tilt_)
VEL_OUT = G * MAX_POS_TILT   # velocity-loop output limit [m/s^2]

# Rate-loop plant (roll), calibrated in acro_gain_rate_loop_margins.py so the
# current roll rate gains reproduce a real-flight-plausible PM~55deg:
#   G(s) = eta_t * e^{-L s} / ( I * s * (tau_m s + 1) )   torque -> rate
I_ROLL = 9.16e-6
TAU_M = 0.02                 # [s] motor first-order lag
L_DELAY = 0.012              # [s] transport/compute dead time
ETA_T = 0.10                 # effective torque gain (calibrated)

# Current production gains (params.cpp, 2026-09-11)
RATE_GAINS = (1.0e-3, 0.7, 0.002)
ATT_GAINS  = (5.0, 2.0, 0.04)
VEL_GAINS  = (3.0, 2.0, 0.0)
POS_GAINS  = (0.4, 5.0, 0.0)

# Rate/attitude output limits: generously large so they never bind in this
# small-signal regime (the .scn this investigation uses is explicitly
# designed to stay linear/unsaturated -- see its header comment), matching
# reality for a ~0.02-0.03 rad roll_sp nudge.
RATE_OUT_LIMIT = 1.0         # [Nm] torque -- generous, not expected to saturate
ATT_OUT_LIMIT  = 20.0        # [rad/s] rate setpoint -- generous


class PID:
    """Exact replica of firmware pid.hpp PID::compute() (same as
    poshold_loop_design.py's PID class -- kept identical for fidelity)."""
    def __init__(self, kp, ti, td=0.0, eta=0.125, output_limit=1.0):
        self.kp = kp; self.ti = ti; self.td = td; self.eta = eta
        self.output_limit = output_limit
        self.integral = 0.0; self.deriv_filter = 0.0
        self.prev_error = 0.0; self.prev_measurement = 0.0
        self.first_run = True

    def compute(self, setpoint, measurement, dt):
        if dt <= 0:
            return 0.0
        error = setpoint - measurement
        p_term = self.kp * error
        d_term = 0.0
        if self.td > 0:
            if self.first_run:
                self.prev_measurement = measurement
            else:
                alpha = 2.0 * self.eta * self.td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * self.td / ((alpha + 1.0) * dt)
                self.deriv_filter = a * self.deriv_filter - b * (measurement - self.prev_measurement)
                d_term = self.kp * self.deriv_filter
        self.prev_measurement = measurement
        self.first_run = False
        if self.ti >= 0.01:
            i_next = self.integral + (self.kp / self.ti) * (error + self.prev_error) * (dt * 0.5)
            out_test = p_term + i_next + d_term
            push_high = (out_test > self.output_limit) and (error > 0)
            push_low = (out_test < -self.output_limit) and (error < 0)
            if not (push_high or push_low):
                self.integral = i_next
            self.integral = max(-self.output_limit, min(self.output_limit, self.integral))
        self.prev_error = error
        out = p_term + self.integral + d_term
        return max(-self.output_limit, min(self.output_limit, out))


def simulate_cascade(rate_gains=RATE_GAINS, att_gains=ATT_GAINS,
                      vel_gains=VEL_GAINS, pos_gains=POS_GAINS,
                      T=40.0, x0=0.03, dt=DT, verbose_every=None):
    """Full 4-stage nonlinear discrete-time POS_HOLD cascade, roll/Y axis only.

    position -> velocity -> attitude -> rate -> (motor lag + dead time) ->
    angular_accel -> rate -> angle -> (g*sin) -> lateral accel -> velocity ->
    position. Setpoints all zero (hold); x0 = initial position offset [m]
    (the "few-cm nudge" from pos_gain_deficit_smallnudge_hold40.scn).

    Returns dict of time series (t, roll_sp, roll, rate_sp, rate, vy_sp, vy, py).
    """
    pos_pid = PID(*pos_gains, output_limit=MAX_POS_VEL)
    vel_pid = PID(*vel_gains, output_limit=VEL_OUT)
    att_pid = PID(*att_gains, output_limit=ATT_OUT_LIMIT)
    rate_pid = PID(*rate_gains, output_limit=RATE_OUT_LIMIT)

    n = int(T / dt)
    ndelay = max(1, int(round(L_DELAY / dt)))
    torque_buf = [0.0] * ndelay

    # plant state
    py = x0          # lateral position [m]
    vy = 0.0          # lateral velocity [m/s]
    roll = 0.0        # actual roll angle [rad]
    rate = 0.0        # actual roll rate [rad/s]
    torque_lag_state = 0.0   # first-order-lag internal state [Nm]

    t_arr = np.empty(n)
    roll_sp_arr = np.empty(n); roll_arr = np.empty(n)
    rate_sp_arr = np.empty(n); rate_arr = np.empty(n)
    vy_sp_arr = np.empty(n); vy_arr = np.empty(n)
    py_arr = np.empty(n)

    alpha_lag = dt / (TAU_M + dt)   # discrete first-order lag coefficient

    for k in range(n):
        # ---- cascade (mirrors computePositionHold + compute()'s attitude/rate) ----
        vy_sp = pos_pid.compute(0.0, py, dt)
        ay_ned = vel_pid.compute(vy_sp, vy, dt)
        roll_sp = ay_ned / G
        roll_sp = max(-MAX_POS_TILT, min(MAX_POS_TILT, roll_sp))
        rate_sp = att_pid.compute(roll_sp, roll, dt)
        torque_cmd = rate_pid.compute(rate_sp, rate, dt)

        # ---- rate-loop plant: torque -> [dead time] -> [motor lag] -> I*accel ----
        torque_buf.append(torque_cmd)
        torque_delayed = torque_buf.pop(0)
        torque_lag_state += alpha_lag * (torque_delayed - torque_lag_state)
        ang_accel = ETA_T * torque_lag_state / I_ROLL
        rate += ang_accel * dt
        roll += rate * dt

        # ---- translational plant: roll -> lateral accel -> velocity -> position ----
        ay_actual = G * np.sin(roll)
        vy += ay_actual * dt
        py += vy * dt

        t_arr[k] = k * dt
        roll_sp_arr[k] = roll_sp; roll_arr[k] = roll
        rate_sp_arr[k] = rate_sp; rate_arr[k] = rate
        vy_sp_arr[k] = vy_sp; vy_arr[k] = vy
        py_arr[k] = py

    return dict(t=t_arr, roll_sp=roll_sp_arr, roll=roll_arr,
                rate_sp=rate_sp_arr, rate=rate_arr,
                vy_sp=vy_sp_arr, vy=vy_arr, py=py_arr)


def fit_pole(t, x, t_skip=5.0):
    """Same zero-crossing/amplitude-ratio pole estimator as
    poshold_loop_design.py's fit_pole()."""
    m = t >= t_skip
    t = t[m]; x = x[m]
    dx = np.diff(x)
    peaks = []
    for i in range(1, len(dx)):
        if dx[i - 1] > 0 and dx[i] <= 0:
            peaks.append((t[i], x[i]))
        elif dx[i - 1] < 0 and dx[i] >= 0:
            peaks.append((t[i], x[i]))
    if len(peaks) < 3:
        return None
    tp = np.array([p[0] for p in peaks]); ap = np.array([abs(p[1]) for p in peaks])
    halfper = np.median(np.diff(tp))
    omega = np.pi / halfper if halfper > 0 else float('nan')
    good = ap > 1e-6
    if good.sum() >= 3:
        coef = np.polyfit(tp[good], np.log(ap[good]), 1)
        sigma = coef[0]
    else:
        sigma = float('nan')
    zeta = -sigma / np.hypot(sigma, omega) if np.isfinite(sigma) else float('nan')
    return omega, sigma, zeta, len(peaks)


if __name__ == "__main__":
    print("=== Full 4-stage nonlinear cascade sim, current production gains ===")
    print(f"rate.roll={RATE_GAINS}  attitude.roll={ATT_GAINS}  "
          f"position.vel={VEL_GAINS}  position.pos={POS_GAINS}")
    print(f"plant: I_roll={I_ROLL:.3e}  tau_m={TAU_M*1000:.0f}ms  "
          f"L={L_DELAY*1000:.0f}ms  eta_t={ETA_T}\n")

    r = simulate_cascade(x0=0.03, T=40.0)

    for name in ["py", "roll", "vy"]:
        pole = fit_pole(r["t"], r[name])
        if pole:
            o, s, z, npk = pole
            period = 2 * np.pi / o if o else float('nan')
            print(f"{name:6}: omega={o:.3f} rad/s (period={period:.3f}s)  "
                  f"sigma={s:+.4f}/s  zeta={z:+.4f}  ({'GROWING' if s > 0 else 'decaying'}, {npk} peaks)")
        else:
            print(f"{name:6}: could not fit a pole (too few peaks / decayed to noise floor)")

    print(f"\nFinal py at t=40s: {r['py'][-1]*100:.3f} cm  "
          f"(started at x0=3.0cm; near-zero = decayed to hold, "
          f"still oscillating near +-3-5cm = matches SILS's persistent behavior)")

    print("\n--- sample time series (t=15..20s, roll_sp/roll/py) for a quick sanity look ---")
    idx = np.where((r["t"] >= 15.0) & (r["t"] <= 20.0))[0][::20]  # every 0.05s like the SILS log
    for i in idx:
        print(f"t={r['t'][i]:6.2f}  roll_sp={r['roll_sp'][i]:8.4f}  roll={r['roll'][i]:8.4f}  "
              f"vy={r['vy'][i]:8.4f}  py={r['py'][i]*100:7.3f}cm")
