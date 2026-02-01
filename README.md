# MDP + Kalman Filtering: How Open-Loop Perception Uncertainty Impacts Decisions

This project demonstrates how open-loop perception noise can destabilize downstream decision-making in autonomy, and how belief-state estimation using Kalman filtering restores temporal consistency. Using a minimal 1D vehicle motion model, we compare decisions made from raw observations versus filtered belief states, and quantify decision instability under increasing sensor noise.

## Motivation
Autonomous driving operates under partial observability:
- Sensors provide noisy, incomplete measurements of the world
- Decisions are often made near safety-critical thresholds (e.g., braking distance)
A common failure mode is not gross inaccuracy, but decision instability:
> small measurement noise causes repeated switching between actions (e.g., brake / no-brake).
This project shows that belief-state estimation (rather than raw perception) is essential for stable and safe decision-making.

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
