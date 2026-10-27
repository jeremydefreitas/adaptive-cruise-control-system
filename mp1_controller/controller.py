"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple
import math


# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold  # 4.0 m hard safety limit

        # Simulation time step (matches CARLA fixed_delta_seconds)
        self.dt = 0.1

        # Current operating mode
        self.mode = Mode.CRUISING

        # ----------------------------------------------------------------
        # Cruising mode — PI speed controller
        # ----------------------------------------------------------------
        self.Kp_cruise = 3.0
        self.Ki_cruise = 0.2
        self.cruise_integral = 0.0
        self.cruise_integral_limit = 15.0  # anti-windup clamp

        # ----------------------------------------------------------------
        # Following mode — PD gap controller
        # ----------------------------------------------------------------
        self.Kp_follow = 1.0
        self.Kd_follow = 3.0

        # ----------------------------------------------------------------
        # Safety parameters
        # ----------------------------------------------------------------
        self.d_safe_min = 12.0         # minimum following gap (m, center-to-center)
        self.time_headway = 0.8        # additional gap: seconds per m/s of ego speed
        self.emergency_dist = 9.0      # hard braking threshold (m)
        self.max_decel_est = 3.0       # conservative max deceleration (m/s²)
        self.d_stop_margin = 8.0       # stop-margin for safe-speed calculation

        # ----------------------------------------------------------------
        # Mode-switching thresholds (hysteresis band prevents chattering)
        #   CRUISING  when d > cruise_enter_dist   (STL requires this at 40 m)
        #   FOLLOWING when d < follow_enter_dist
        #   Keep current mode in between
        # ----------------------------------------------------------------
        self.cruise_enter_dist = 40.0
        self.follow_enter_dist = 30.0

        # ----------------------------------------------------------------
        # State
        # ----------------------------------------------------------------
        self.prev_gap_error = None
        self.prev_distance = None

    # ------------------------------------------------------------------
    # Safe approach speed — the maximum speed that still allows stopping
    # before the gap shrinks to d_stop_margin, assuming worst case (lead
    # suddenly stationary).  v_safe = sqrt(2 * a_max * (d - margin))
    # ------------------------------------------------------------------
    def _safe_speed(self, d: float) -> float:
        return math.sqrt(max(0.0, 2.0 * self.max_decel_est * (d - self.d_stop_margin)))

    # ------------------------------------------------------------------
    # Main entry point — called once per simulation step (10 Hz)
    # ------------------------------------------------------------------
    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        v = obs.ego_velocity
        v_des = obs.desired_speed
        d = obs.distance_to_lead

        d_inf = math.isinf(d) or d > 500.0

        # ---- Detect if the gap is opening (lead pulling away) ----
        gap_opening = False
        if self.prev_distance is not None and not d_inf:
            gap_opening = (d - self.prev_distance) > 0.05

        # ---- Mode switching with hysteresis ----
        if d_inf or d > self.cruise_enter_dist:
            if self.mode != Mode.CRUISING:
                self.prev_gap_error = None        # reset following state
            self.mode = Mode.CRUISING
        elif d < self.follow_enter_dist:
            if self.mode != Mode.FOLLOWING:
                self.cruise_integral = 0.0        # reset cruise integrator
            self.mode = Mode.FOLLOWING
        # else: stay in current mode (hysteresis band)

        # ---- Compute acceleration ----
        if self.mode == Mode.CRUISING:
            acc = self._cruise(v, v_des, d, d_inf)
        else:
            acc = self._follow(v, v_des, d)

        # ---- Global safety clamps ----
        acc = max(-10.0, min(10.0, acc))

        # Prevent reversing: limit braking when nearly stopped
        if v < 0.3 and acc < 0.0:
            acc = max(acc, -0.5)

        # Anti-stop: prevent ego from stopping while lead is still moving.
        # Only applies when we are NOT dangerously close (d > emergency_dist).
        if v < 0.5 and not d_inf and d > self.emergency_dist:
            if gap_opening:
                # Lead is pulling away — definitely keep moving
                acc = max(acc, 2.0)
            elif d > self.d_safe_min:
                # Healthy gap — creep forward
                acc = max(acc, 1.0)

        # Track previous distance
        if not d_inf:
            self.prev_distance = d

        return (acc, self.mode)

    # ------------------------------------------------------------------
    # Cruising controller (PI on speed error, with safe-speed limiting)
    # ------------------------------------------------------------------
    def _cruise(self, v: float, v_des: float, d: float, d_inf: bool) -> float:
        """PI speed controller with distance-aware approach-speed limit."""
        # Limit target speed so we can always stop safely
        if not d_inf:
            target = min(v_des, self._safe_speed(d))
        else:
            target = v_des

        error = target - v

        # If target is well below v_des (braking to approach), clamp integral
        # to prevent positive windup from fighting the deceleration
        if target < v_des - 2.0:
            self.cruise_integral = min(self.cruise_integral, 0.0)

        # Accumulate integral with anti-windup
        self.cruise_integral += error * self.dt
        self.cruise_integral = max(-self.cruise_integral_limit,
                                   min(self.cruise_integral_limit,
                                       self.cruise_integral))

        return self.Kp_cruise * error + self.Ki_cruise * self.cruise_integral

    # ------------------------------------------------------------------
    # Following controller (PD on gap error, with safe-speed backstop)
    # ------------------------------------------------------------------
    def _follow(self, v: float, v_des: float, d: float) -> float:
        """PD gap controller with safe-speed backstop."""
        # Velocity-dependent desired gap with a safety floor
        desired_gap = self.d_safe_min + self.time_headway * v

        # Gap error: positive = more room than needed, negative = too close
        gap_error = d - desired_gap

        # Numerical derivative of gap error (dominated by closing rate)
        if self.prev_gap_error is not None:
            gap_error_dot = (gap_error - self.prev_gap_error) / self.dt
        else:
            gap_error_dot = 0.0
        self.prev_gap_error = gap_error

        # PD control
        acc = self.Kp_follow * gap_error + self.Kd_follow * gap_error_dot

        # --- Safe-speed backstop ---
        # If going faster than what allows a safe stop, override with braking
        v_safe = self._safe_speed(d)
        if v > v_safe:
            acc = min(acc, -self.max_decel_est)

        # --- Emergency braking for dangerously small gaps ---
        if d < self.emergency_dist:
            severity = (self.emergency_dist - d) / \
                       (self.emergency_dist - self.distance_threshold)
            severity = max(0.0, min(1.0, severity))
            acc = min(acc, -10.0 * severity)

        # --- Don't exceed desired cruise speed in following mode ---
        if v > v_des and acc > 0.0:
            acc = min(acc, 2.0 * (v_des - v))

        return acc
