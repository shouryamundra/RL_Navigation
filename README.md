# RL_Navigation — ARCC Sentry 1v1 RL (v0)

Fast vectorized simulator and recurrent-PPO policy for the ARCC Sentry Tournament
(1 autonomous Sentry vs 1, 120 s, 5×5 m field).

- `plans/` — architecture, env spec, evaluation plan, work breakdown (read these first)
- `sentry_sim/` — PyTorch vectorized environment, shooting model, scripted bots, replay
- `scripts/` — training / evaluation entry points
- `tests/` — unit tests for the rules (HP, lifesteal, win ladder) and the shooting model

v0 scope: agent controls chassis velocity + turret yaw/pitch rate + fire toggle.
No heat, power limit, penalties or setup period yet; those come after v0.
