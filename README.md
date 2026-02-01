# MDP + Kalman Filtering: How Open-Loop Perception Uncertainty Impacts Decisions

This project demonstrates, in a minimal and reproducible way, how noisy open-loop perception can lead to unstable or late decisions in autonomy, and how belief-state estimation (Kalman filtering) can mitigate this.

## Motivation
Autonomous driving is partially observable: sensors provide noisy, incomplete observations of the true world state. If downstream decision-making relies directly on noisy observations (open-loop perception), errors can propagate into unsafe actions.

## What this repo contains
- A simple 1D vehicle motion model (state = position, velocity)
- Noisy position observations (simulating perception noise)
- A constant-velocity Kalman Filter to maintain a belief state
- A simplified MDP-style decision rule with discrete actions (BRAKE / MAINTAIN)
- Experiments showing how increasing observation noise affects decisions

## Key idea
Using a belief estimate (mean + uncertainty) produces earlier and more stable decisions compared to using raw noisy observations.

## How to run
```bash
pip install -r requirements.txt
python src/experiments.py
