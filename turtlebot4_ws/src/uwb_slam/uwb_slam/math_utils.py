"""
Math utilities for UWB trilateration and ESKF sensor fusion.
Provides core algorithms for range-based localization and Kalman filtering.
"""

import math

import numpy as np
from typing import Tuple, List, Optional


def parse_anchor_positions(value) -> np.ndarray:
    """Parse anchor_positions parameter into an Nx2 float array.

    Accepts a flat list/array [x1, y1, x2, y2, ...], an Nx2 array, or a
    string representation thereof (as received from ROS 2 parameter server).
    """
    if isinstance(value, str):
        import ast
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError) as e:
            raise ValueError(f'anchor_positions: failed to parse string value: {e}') from e
    arr = np.array(value, dtype=float)
    if arr.ndim == 1:
        if arr.size < 6 or (arr.size % 2) != 0:
            raise ValueError('anchor_positions must be [x1, y1, x2, y2, ...] with at least 3 anchors')
        arr = arr.reshape((-1, 2))
    elif not (arr.ndim == 2 and arr.shape[1] == 2):
        raise ValueError('anchor_positions must be Nx2 or flat [x1, y1, x2, y2, ...]')
    return arr


def attach_debugger_if_requested() -> None:
    """Attach a waiting debugpy session if ROS_DEBUG_PORT is set."""
    import os  # noqa: PLC0415  (lazy import — only when this utility is called)
    port_str = os.environ.get('ROS_DEBUG_PORT')
    if port_str:
        import debugpy  # noqa: PLC0415  (lazy import — only when debugging)
        debugpy.listen(('localhost', int(port_str)))
        print(f'[debugpy] Waiting for VS Code debugger on port {port_str}...')
        debugpy.wait_for_client()


class Trilateration:
    """
    Trilateration solver using range measurements from multiple anchors.
    Converts UWB range data into estimated position.
    """
    
    def __init__(self, anchor_positions: List[Tuple[float, float]]):
        """
        Initialize trilateration solver with fixed anchor positions.
        
        Args:
            anchor_positions: List of (x, y) coordinates for anchors
        """
        if len(anchor_positions) < 3:
            raise ValueError("At least 3 anchors required for trilateration")
        
        self.anchor_positions = np.array(anchor_positions)
        self.num_anchors = len(anchor_positions)

    def _validate_ranges(self, ranges: List[float]) -> np.ndarray:
        if len(ranges) != self.num_anchors:
            raise ValueError(f"Expected {self.num_anchors} ranges, got {len(ranges)}")

        values = np.asarray(ranges, dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError('Ranges must be finite')
        if np.any(values < 0.0):
            raise ValueError('Ranges must be non-negative')
        return values

    def _initial_guess(self, initial_guess: Optional[Tuple[float, float]]) -> np.ndarray:
        if initial_guess is None:
            return np.array([
                np.mean(self.anchor_positions[:, 0]),
                np.mean(self.anchor_positions[:, 1]),
            ], dtype=float)

        guess = np.asarray(initial_guess, dtype=float)
        if guess.shape != (2,) or not np.all(np.isfinite(guess)):
            raise ValueError('Initial guess must be a finite (x, y) tuple')
        return guess

    def _solve_position(
        self,
        ranges: np.ndarray,
        initial_guess: Optional[Tuple[float, float]] = None,
        weights: Optional[np.ndarray] = None,
        max_iterations: int = 20,
    ) -> Tuple[float, float]:
        estimate = self._initial_guess(initial_guess)

        if weights is not None:
            weights = np.asarray(weights, dtype=float)
            if weights.shape != (self.num_anchors,):
                raise ValueError(f"Expected {self.num_anchors} weights, got {len(weights)}")
            if not np.all(np.isfinite(weights)):
                raise ValueError('Weights must be finite')
            if np.any(weights < 0.0):
                raise ValueError('Weights must be non-negative')

            if not np.any(weights > 0.0):
                raise ValueError('At least one weight must be positive')

            sqrt_weights = np.sqrt(weights)
        else:
            sqrt_weights = None

        anchor_xy = self.anchor_positions[:, :2].astype(float)

        for _ in range(max_iterations):
            deltas = estimate - anchor_xy
            distances = np.linalg.norm(deltas, axis=1)
            safe_distances = np.maximum(distances, 1e-9)

            residual = distances - ranges
            jacobian = deltas / safe_distances[:, None]

            if sqrt_weights is not None:
                residual = residual * sqrt_weights
                jacobian = jacobian * sqrt_weights[:, None]

            try:
                step, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
            except np.linalg.LinAlgError:
                step = np.linalg.pinv(jacobian) @ (-residual)

            if not np.all(np.isfinite(step)):
                break

            estimate = estimate + step
            if np.linalg.norm(step) < 1e-6:
                break

        # Compute final RMS residual so callers can assess solution quality
        deltas_final = estimate - anchor_xy
        distances_final = np.maximum(np.linalg.norm(deltas_final, axis=1), 1e-9)
        err = distances_final - ranges
        if sqrt_weights is not None:
            final_residual = float(np.linalg.norm(err * sqrt_weights))
        else:
            final_residual = float(np.linalg.norm(err))

        return float(estimate[0]), float(estimate[1]), final_residual
    
    def solve(self, ranges: List[float], initial_guess: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        """
        Solve for position given ranges from anchors.
        
        Args:
            ranges: List of measured distances from each anchor
            initial_guess: Initial position estimate (x, y). Uses anchor centroid if None.
        
        Returns:
            Tuple of (x, y) estimated position
        """
        ranges_array = self._validate_ranges(ranges)
        x, y, residual = self._solve_position(ranges_array, initial_guess)
        if residual > 0.5:
            raise RuntimeError(
                f'Trilateration solver non-convergent: residual {residual:.3f} m > 0.5 m threshold'
            )
        return x, y
    
    def solve_with_weights(self, ranges: List[float], weights: List[float], 
                          initial_guess: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        """
        Solve with weighted range measurements (newer anchors = higher weight).
        
        Args:
            ranges: List of measured distances
            weights: List of confidence weights for each range
            initial_guess: Initial position estimate
        
        Returns:
            Tuple of (x, y) estimated position
        """
        ranges_array = self._validate_ranges(ranges)
        x, y, residual = self._solve_position(ranges_array, initial_guess, np.asarray(weights, dtype=float))
        if residual > 0.5:
            raise RuntimeError(
                f'Trilateration solver non-convergent: residual {residual:.3f} m > 0.5 m threshold'
            )
        return x, y


class ErrorStateKalmanFilter:
    """
    2D Error-State Kalman Filter (ESKF) for sensor fusion.
    Fuses IMU, LiDAR, and UWB measurements with 8-state model:
    [x, y, vx, vy, yaw, bias_ax, bias_ay, bias_wz]
    """

    IDX_X = 0
    IDX_Y = 1
    IDX_VX = 2
    IDX_VY = 3
    IDX_YAW = 4
    IDX_BIAS_AX = 5
    IDX_BIAS_AY = 6
    IDX_BIAS_WZ = 7
    STATE_DIM = 8

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Wrap angles to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def __init__(self, process_noise_std: float = 0.1, 
                 imu_measurement_std: float = 0.05,
                 uwb_measurement_std: float = 0.2,
                 lidar_measurement_std: float = 0.15):
        """
        Initialize ESKF with process and measurement noise parameters.
        
        Args:
            process_noise_std: Standard deviation of process noise
            imu_measurement_std: STD of IMU acceleration measurements
            uwb_measurement_std: STD of UWB range measurements
            lidar_measurement_std: STD of LiDAR pose measurements
        """
        self.x = np.zeros(self.STATE_DIM)

        self.P = np.eye(self.STATE_DIM) * 0.5
        self.P[self.IDX_X:self.IDX_Y + 1, self.IDX_X:self.IDX_Y + 1] *= 1.0
        self.P[self.IDX_VX:self.IDX_VY + 1, self.IDX_VX:self.IDX_VY + 1] *= 0.5
        self.P[self.IDX_YAW, self.IDX_YAW] = 0.25
        self.P[self.IDX_BIAS_AX:self.IDX_BIAS_AY + 1, self.IDX_BIAS_AX:self.IDX_BIAS_AY + 1] *= 0.1
        self.P[self.IDX_BIAS_WZ, self.IDX_BIAS_WZ] = 0.05

        q_pos = process_noise_std ** 2
        q_vel = (process_noise_std * 10.0) ** 2
        q_yaw = (process_noise_std * 0.5) ** 2
        q_bias_acc = (process_noise_std * 0.01) ** 2
        q_bias_wz = (process_noise_std * 0.01) ** 2
        self.Q = np.diag([
            q_pos,
            q_pos,
            q_vel,
            q_vel,
            q_yaw,
            q_bias_acc,
            q_bias_acc,
            q_bias_wz,
        ])

        self.R_uwb = np.array([[uwb_measurement_std ** 2]])
        self.R_lidar = np.eye(2) * (lidar_measurement_std ** 2)
        self.R_yaw_imu = np.array([[imu_measurement_std ** 2]])
        self.R_yaw_lidar = np.array([[lidar_measurement_std ** 2]])
        
        self.dt = 0.01  # Default time step (100 Hz)
    
    def predict(self, accel_body: Tuple[float, float], gyro_z: float, dt: Optional[float] = None):
        """
        Predict state using body-frame acceleration and yaw-rate measurements.
        
        Args:
            accel_body: Measured body-frame acceleration (ax, ay) from IMU
            gyro_z: Measured body yaw-rate from IMU
            dt: Time step (uses self.dt if None)
        """
        if dt is None:
            dt = self.dt

        x, y, vx, vy, yaw, bias_ax, bias_ay, bias_wz = self.x

        ax_body = accel_body[0] - bias_ax
        ay_body = accel_body[1] - bias_ay
        yaw_new = self.wrap_angle(yaw + (gyro_z - bias_wz) * dt)

        cos_y = math.cos(yaw_new)
        sin_y = math.sin(yaw_new)
        ax_world = cos_y * ax_body - sin_y * ay_body
        ay_world = sin_y * ax_body + cos_y * ay_body

        self.x[self.IDX_X] = x + vx * dt + 0.5 * ax_world * (dt ** 2)
        self.x[self.IDX_Y] = y + vy * dt + 0.5 * ay_world * (dt ** 2)
        self.x[self.IDX_VX] = vx + ax_world * dt
        self.x[self.IDX_VY] = vy + ay_world * dt
        self.x[self.IDX_YAW] = yaw_new

        F = np.eye(self.STATE_DIM)
        F[self.IDX_X, self.IDX_VX] = dt
        F[self.IDX_Y, self.IDX_VY] = dt
        F[self.IDX_YAW, self.IDX_BIAS_WZ] = -dt

        dax_world_dyaw = -sin_y * ax_body - cos_y * ay_body
        day_world_dyaw = cos_y * ax_body - sin_y * ay_body

        F[self.IDX_X, self.IDX_YAW] = 0.5 * (dt ** 2) * dax_world_dyaw
        F[self.IDX_Y, self.IDX_YAW] = 0.5 * (dt ** 2) * day_world_dyaw
        F[self.IDX_VX, self.IDX_YAW] = dt * dax_world_dyaw
        F[self.IDX_VY, self.IDX_YAW] = dt * day_world_dyaw

        F[self.IDX_X, self.IDX_BIAS_AX] = -0.5 * (dt ** 2) * cos_y
        F[self.IDX_X, self.IDX_BIAS_AY] = 0.5 * (dt ** 2) * sin_y
        F[self.IDX_Y, self.IDX_BIAS_AX] = -0.5 * (dt ** 2) * sin_y
        F[self.IDX_Y, self.IDX_BIAS_AY] = -0.5 * (dt ** 2) * cos_y
        F[self.IDX_VX, self.IDX_BIAS_AX] = -dt * cos_y
        F[self.IDX_VX, self.IDX_BIAS_AY] = dt * sin_y
        F[self.IDX_VY, self.IDX_BIAS_AX] = -dt * sin_y
        F[self.IDX_VY, self.IDX_BIAS_AY] = -dt * cos_y

        self.P = F @ self.P @ F.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)
    
    def compute_uwb_innovation(self, range_measurement: float, anchor_pos: Tuple[float, float]):
        """
        Compute UWB innovation statistics for gating.

        Returns:
            (residual, S_scalar, H, predicted_range)
        """
        x, y = self.x[self.IDX_X:self.IDX_Y + 1]
        dx = x - anchor_pos[0]
        dy = y - anchor_pos[1]
        predicted_range = np.sqrt(dx ** 2 + dy ** 2)

        residual = range_measurement - predicted_range

        H = np.zeros((1, self.STATE_DIM))
        if predicted_range > 1e-6:
            H[0, self.IDX_X] = dx / predicted_range
            H[0, self.IDX_Y] = dy / predicted_range

        S = H @ self.P @ H.T + self.R_uwb
        return float(residual), float(S[0, 0]), H, float(predicted_range)

    def update_uwb(self, range_measurement: float, anchor_pos: Tuple[float, float]):
        """
        Update state with UWB range measurement from single anchor.
        
        Args:
            range_measurement: Measured distance to anchor
            anchor_pos: (x, y) position of anchor
        """
        residual, s_scalar, H, _ = self.compute_uwb_innovation(range_measurement, anchor_pos)
        if s_scalar <= 1e-12:
            return

        K = self.P @ H.T / s_scalar
        self.x = self.x + K.flatten() * residual

        # Joseph-form covariance update to preserve PSD behavior numerically.
        I = np.eye(self.STATE_DIM)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ self.R_uwb @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_odometry_velocity(self, measured_vel: Tuple[float, float], measurement_std: float = 0.05):
        """
        Update state with wheel-odometry linear velocity measurement.

        Args:
            measured_vel: (vx, vy)
            measurement_std: Velocity measurement std-dev in m/s
        """
        H = np.zeros((2, self.STATE_DIM))
        H[0, self.IDX_VX] = 1.0
        H[1, self.IDX_VY] = 1.0

        R_odom = np.eye(2) * (measurement_std ** 2)
        residual = np.array(measured_vel) - self.x[self.IDX_VX:self.IDX_VY + 1]
        S = H @ self.P @ H.T + R_odom
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # degenerate covariance; skip update this cycle
        K = self.P @ H.T @ S_inv

        self.x = self.x + K @ residual
        I = np.eye(self.STATE_DIM)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ R_odom @ K.T
        self.P = 0.5 * (self.P + self.P.T)
    
    def update_lidar_pose(self, measured_pos: Tuple[float, float]):
        """
        Update state with LiDAR/vision pose measurement.
        
        Args:
            measured_pos: (x, y) position from LiDAR SLAM
        """
        # Measurement matrix (observe x, y)
        H = np.zeros((2, self.STATE_DIM))
        H[0, self.IDX_X] = 1.0
        H[1, self.IDX_Y] = 1.0
        
        # Residual
        residual = np.array(measured_pos) - self.x[self.IDX_X:self.IDX_Y + 1]
        
        # Kalman gain
        S = H @ self.P @ H.T + self.R_lidar
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # degenerate covariance; skip update this cycle
        K = self.P @ H.T @ S_inv
        
        # Update
        self.x = self.x + K @ residual
        self.P = (np.eye(self.STATE_DIM) - K @ H) @ self.P @ (np.eye(self.STATE_DIM) - K @ H).T + K @ self.R_lidar @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_yaw(self, measured_yaw: float, measurement_std: Optional[float] = None):
        """Update state with a wrapped yaw measurement."""
        H = np.zeros((1, self.STATE_DIM))
        H[0, self.IDX_YAW] = 1.0

        residual = self.wrap_angle(measured_yaw - self.x[self.IDX_YAW])
        noise = self.R_yaw_lidar if measurement_std is None else np.array([[measurement_std ** 2]])
        S = H @ self.P @ H.T + noise
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ H.T @ S_inv

        self.x = self.x + (K.flatten() * residual)
        self.x[self.IDX_YAW] = self.wrap_angle(self.x[self.IDX_YAW])
        I = np.eye(self.STATE_DIM)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ noise @ K.T
        self.P = 0.5 * (self.P + self.P.T)
    
    def get_position(self) -> Tuple[float, float]:
        """Return current estimated position (x, y)."""
        return tuple(self.x[self.IDX_X:self.IDX_Y + 1])
    
    def get_velocity(self) -> Tuple[float, float]:
        """Return current estimated velocity (vx, vy)."""
        return tuple(self.x[self.IDX_VX:self.IDX_VY + 1])

    def get_yaw(self) -> float:
        """Return current filtered yaw estimate."""
        return float(self.x[self.IDX_YAW])
    
    def get_state(self) -> np.ndarray:
        """Return full state vector."""
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Return covariance matrix."""
        return self.P.copy()
