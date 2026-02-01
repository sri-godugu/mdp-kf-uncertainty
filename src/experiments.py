# src/experiments.py
import numpy as np
import matplotlib.pyplot as plt

from kalman import KalmanFilterCV
from simulate import simulate_motion, noisy_observations
from policy import PolicyConfig, decide_action, BRAKE, MAINTAIN

def run_once(meas_std: float, seed: int = 0):
    rng = np.random.default_rng(seed)

    dt = 0.1
    T = 200

    # True motion: moving toward stop zone
    x0, v0 = 0.0, 10.0  # 10 m/s
    x_stop = 120.0

    # True acceleration profile: initially maintain, then would brake if commanded
    # We'll simulate "ground truth dynamics" separately from policy for simplicity
    true_x, true_v, _ = simulate_motion(T=T, dt=dt, x0=x0, v0=v0, a_profile=lambda t: 0.0)

    z = noisy_observations(true_x, meas_std=meas_std, rng=rng)

    # Kalman filter
    kf = KalmanFilterCV(dt=dt, process_var=1.0, meas_var=meas_std**2)
    kf.init(position=z[0], velocity=v0, pos_var=25.0, vel_var=25.0)

    # Policies
    cfg_obs = PolicyConfig(x_stop=x_stop, brake_margin=20.0, use_uncertainty=False)
    cfg_kf  = PolicyConfig(x_stop=x_stop, brake_margin=20.0, use_uncertainty=True, k_sigma=2.0)

    x_kf = np.zeros(T)
    v_kf = np.zeros(T)
    std_kf = np.zeros(T)

    act_obs = np.zeros(T, dtype=int)
    act_kf  = np.zeros(T, dtype=int)

    late_brake_obs = 0
    late_brake_kf  = 0

    # Define "late braking" as: if you decide to brake when already too close
    # (distance < 8m) — simplistic but demonstrates effect.
    too_late_thresh = 8.0

    for t in range(T):
        # KF update
        kf.step(z[t])
        x_est, v_est = kf.get_state()
        pos_std = kf.get_pos_std()

        x_kf[t] = x_est
        v_kf[t] = v_est
        std_kf[t] = pos_std

        # Observation-based decision (uses raw z)
        a1 = decide_action(x_est=z[t], pos_std=0.0, cfg=cfg_obs)
        act_obs[t] = a1

        # KF-based decision (uses belief mean + std)
        a2 = decide_action(x_est=x_est, pos_std=pos_std, cfg=cfg_kf)
        act_kf[t] = a2

        # Late brake checks
        dist_true = x_stop - true_x[t]
        if a1 == BRAKE and dist_true < too_late_thresh:
            late_brake_obs += 1
        if a2 == BRAKE and dist_true < too_late_thresh:
            late_brake_kf += 1

    return {
        "true_x": true_x, "true_v": true_v,
        "z": z,
        "x_kf": x_kf, "v_kf": v_kf, "std_kf": std_kf,
        "act_obs": act_obs, "act_kf": act_kf,
        "late_brake_obs": late_brake_obs,
        "late_brake_kf": late_brake_kf,
        "x_stop": x_stop,
        "dt": dt
    }

def plot_run(out, title=""):
    t = np.arange(out["true_x"].shape[0]) * out["dt"]

    plt.figure()
    plt.plot(t, out["true_x"], label="true position")
    plt.plot(t, out["z"], label="noisy observation")
    plt.plot(t, out["x_kf"], label="KF estimate")
    plt.axhline(out["x_stop"], linestyle="--", label="stop zone")
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.title(title or "Position: true vs observation vs KF")
    plt.legend()
    plt.tight_layout()

    plt.figure()
    plt.plot(t, out["act_obs"], label="action (obs-based)")
    plt.plot(t, out["act_kf"], label="action (KF-based)")
    plt.yticks([0, 1], ["BRAKE", "MAINTAIN"])
    plt.xlabel("time (s)")
    plt.ylabel("action")
    plt.title("Actions over time")
    plt.legend()
    plt.tight_layout()

def main():
    # Try a couple of noise levels
    for meas_std in [0.5, 2.0, 5.0]:
        out = run_once(meas_std=meas_std, seed=0)
        plot_run(out, title=f"meas_std={meas_std} | late_brake obs={out['late_brake_obs']} KF={out['late_brake_kf']}")
    plt.show()

if __name__ == "__main__":
    main()
