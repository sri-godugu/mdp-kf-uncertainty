# src/simulate.py
import numpy as np

def simulate_motion(T: int, dt: float, x0: float, v0: float, a_profile=None):
    """
    Simulate 1D motion.
    x_{t+1} = x_t + v_t dt + 0.5 a dt^2
    v_{t+1} = v_t + a dt
    a_profile: function(t)->acceleration
    """
    x = np.zeros(T)
    v = np.zeros(T)
    a = np.zeros(T)

    x[0] = x0
    v[0] = v0

    for t in range(T - 1):
        at = 0.0 if a_profile is None else float(a_profile(t))
        a[t] = at
        x[t+1] = x[t] + v[t] * dt + 0.5 * at * dt * dt
        v[t+1] = v[t] + at * dt

    a[T-1] = 0.0 if a_profile is None else float(a_profile(T-1))
    return x, v, a

def noisy_observations(true_x: np.ndarray, meas_std: float, rng: np.random.Generator):
    noise = rng.normal(loc=0.0, scale=meas_std, size=true_x.shape[0])
    return true_x + noise
