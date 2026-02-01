# src/kalman.py
import numpy as np

class KalmanFilterCV:
    """
    Constant-velocity Kalman Filter in 1D.
    State x = [position, velocity]^T
    """
    def __init__(self, dt: float, process_var: float, meas_var: float):
        self.dt = dt

        # State transition model
        self.F = np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])

        # Measurement model: we measure position only
        self.H = np.array([[1.0, 0.0]])

        # Process noise covariance (simple acceleration noise model)
        # Q = q * [[dt^4/4, dt^3/2],
        #          [dt^3/2, dt^2]]
        q = process_var
        self.Q = q * np.array([
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2]
        ])

        # Measurement noise covariance
        self.R = np.array([[meas_var]])

        # State estimate and covariance (initialized later)
        self.x = np.zeros((2, 1))
        self.P = np.eye(2)

    def init(self, position: float, velocity: float, pos_var: float = 10.0, vel_var: float = 10.0):
        self.x = np.array([[position], [velocity]], dtype=float)
        self.P = np.array([[pos_var, 0.0],
                           [0.0, vel_var]], dtype=float)

    def predict(self):
        # x = F x
        self.x = self.F @ self.x
        # P = F P F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z_pos: float):
        z = np.array([[z_pos]], dtype=float)

        # Innovation: y = z - Hx
        y = z - (self.H @ self.x)

        # Innovation covariance: S = HPH^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain: K = P H^T S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state: x = x + K y
        self.x = self.x + K @ y

        # Update covariance: P = (I - K H) P
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def step(self, z_pos: float):
        self.predict()
        self.update(z_pos)

    def get_state(self):
        return float(self.x[0, 0]), float(self.x[1, 0])

    def get_pos_std(self):
        # sqrt of position variance
        return float(np.sqrt(self.P[0, 0]))
