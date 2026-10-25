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
        #   Goal: reach v_des (20 m/s) quickly and hold it.
        # ----------------------------------------------------------------
        self.Kp_cruise = 3.0
        self.Ki_cruise = 0.3
        self.cruise_integral = 0.0
        self.cruise_integral_limit = 20.0  # anti-windup clamp

        # ----------------------------------------------------------------
        # Following mode — PD gap controller
        #   Goal: maintain a velocity-dependent safe gap to the lead car.
        # ----------------------------------------------------------------
        self.Kp_follow = 1.0
        self.Kd_follow = 3.0

        # Following-distance parameters
        self.d_safe_min = 10.0       # minimum following gap (m, center-to-center)
        self.time_headway = 0.5      # additional gap per m/s of ego speed
        self.emergency_dist = 7.0    # trigger hard braking below this (m)

        # ----------------------------------------------------------------
        # Mode-switching thresholds (hysteresis band prevents chattering)
        #   CRUISING  when d > cruise_enter_dist
        #   FOLLOWING when d < follow_enter_dist
        #   Keep current mode in between
        # ----------------------------------------------------------------
        self.cruise_enter_dist = 40.0
        self.follow_enter_dist = 25.0

        # State for derivative computation
        self.prev_gap_error = None

    # ------------------------------------------------------------------
    # Main entry point — called once per simulation step (10 Hz)
    # ------------------------------------------------------------------
    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        """Process observation and return (acceleration, mode)."""
        v = obs.ego_velocity
        v_des = obs.desired_speed
        d = obs.distance_to_lead

        # Handle infinite distance (lead car left the lane)
        d_inf = math.isinf(d) or d > 500.0

        # ---- Mode selection with hysteresis ----
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
            acc = self._cruise(v, v_des)
        else:
            acc = self._follow(v, v_des, d)

        # ---- Global safety clamps ----
        acc = max(-10.0, min(10.0, acc))

        # Prevent reversing: if nearly stopped, limit braking force
        if v < 0.3 and acc < 0.0:
            acc = max(acc, -0.5)

        return (acc, self.mode)

    # ------------------------------------------------------------------
    # Cruising controller (PI on speed error)
    # ------------------------------------------------------------------
    def _cruise(self, v: float, v_des: float) -> float:
        """PI speed controller — drive toward desired cruise speed."""
        error = v_des - v

        # Accumulate integral with anti-windup
        self.cruise_integral += error * self.dt
        self.cruise_integral = max(-self.cruise_integral_limit,
                                   min(self.cruise_integral_limit,
                                       self.cruise_integral))

        return self.Kp_cruise * error + self.Ki_cruise * self.cruise_integral

    # ------------------------------------------------------------------
    # Following controller (PD on gap error)
    # ------------------------------------------------------------------
    def _follow(self, v: float, v_des: float, d: float) -> float:
        """PD gap controller — maintain safe following distance."""
        # Velocity-dependent desired gap with a safety floor
        desired_gap = self.d_safe_min + self.time_headway * v

        # Gap error: positive = more room than needed, negative = too close
        gap_error = d - desired_gap

        # Numerical derivative of gap error (≈ closing-rate signal)
        if self.prev_gap_error is not None:
            gap_error_dot = (gap_error - self.prev_gap_error) / self.dt
        else:
            gap_error_dot = 0.0
        self.prev_gap_error = gap_error

        # PD control
        acc = self.Kp_follow * gap_error + self.Kd_follow * gap_error_dot

        # --- Emergency braking for dangerously small gaps ---
        if d < self.emergency_dist:
            severity = (self.emergency_dist - d) / \
                       (self.emergency_dist - self.distance_threshold)
            severity = max(0.0, min(1.0, severity))
            acc = min(acc, -10.0 * severity)

        # --- Don't exceed desired cruise speed in following mode ---
        if v > v_des and acc > 0.0:
            acc = min(acc, 2.0 * (v_des - v))

        # --- Prevent unnecessary stopping when gap is healthy ---
        # Supports the dontStopUnlessLeadStops requirement: if we still
        # have plenty of room ahead, keep a minimum forward acceleration
        # so we don't come to a standstill while the lead is moving.
        if v < 0.5 and d > self.d_safe_min + 2.0:
            acc = max(acc, 1.5)

        return acc
