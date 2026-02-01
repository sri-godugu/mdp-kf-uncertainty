# src/policy.py
from dataclasses import dataclass

BRAKE = 0
MAINTAIN = 1

@dataclass
class PolicyConfig:
    x_stop: float          # stop/obstacle location
    brake_margin: float    # how early to brake (meters)
    use_uncertainty: bool = False
    k_sigma: float = 2.0   # safety buffer multiplier if using uncertainty

def decide_action(x_est: float, pos_std: float, cfg: PolicyConfig):
    """
    If estimated distance to stop zone is small, brake.
    Optional uncertainty-aware buffer: brake earlier when uncertainty is high.
    """
    distance = cfg.x_stop - x_est
    buffer = cfg.brake_margin + (cfg.k_sigma * pos_std if cfg.use_uncertainty else 0.0)

    if distance <= buffer:
        return BRAKE
    return MAINTAIN
