# 00 — Architecture (Sentry 1v1 RL, v0)

Status: v0 draft, 2026-11-16. Owner: Fable. Rules source: ARCC Season 2 Events Manual (EM) §4–5; numbers re-checked against EM p.37–38 (Fig 4-3/4-4, §4.3.2, §4.4) and Table 5-2.

## Goal
A simple, fast, vectorized simulator of the Sentry Tournament (1 Sentry vs 1 Sentry, 120 s, 5×5 m) plus a recurrent-PPO policy trained by self-play, with scripted baselines and an evaluation harness that gives win rates with confidence intervals. v0 deliberately omits heat, chassis power, penalties, the setup period and sensing noise; they are added after v0 works end to end.

## Principles
1. **One config object.** Every constant (field geometry, HP, damage, fire rate, shooting-model parameters, FoV, vision range, dynamics limits) lives in `sentry_sim/config.py` (`SimConfig` dataclass, YAML round-trip). No magic numbers in the env.
2. **Batched over envs, in PyTorch.** All state is a dict of tensors with leading dim `N` (number of envs). No Python loops over envs. Same code runs on CPU (tests, replay) and GPU (training).
3. **Canonical frame.** The field is point-symmetric about (2.5, 2.5). Each robot sees the world in a frame where *its own* start zone is top-left: Red = world frame, Blue = world rotated 180°. One policy plays both colors.
4. **Actor sees only what the robot could know; critic sees everything.** The actor observation is built from visible-only opponent data; the critic gets the true state of both robots.
5. **Deterministic given seed + config.** Replays and evaluations reproduce exactly.
6. **Small deliverables, tests per piece.** See `03-work-breakdown.md`.

## Repository layout
```
plans/                     this folder (read first)
sentry_sim/
  config.py                SimConfig dataclass + yaml load/save + default config
  geometry.py              field, walls, bunkers (oriented boxes), ray casting, 2D/3D LOS, collision push-out
  shooting.py              hit-probability model f(range, crossing rate, aim error, plate facing, LOS)
  env.py                   SentryEnv: batched state, step(), reset(), rewards, dones, infos, event log
  obs.py                   actor/critic observation builders (canonical frame)
  frames.py                world <-> canonical transforms
  bots.py                  scripted opponents (static, charger, cover-camper, random-walker)
  replay.py                record episodes -> top-down matplotlib animation (mp4/gif) + still frame
  eval.py                  round-robin runner, Wilson CIs, markdown report
  ppo/
    nets.py                recurrent actor (GRU) + asymmetric critic
    buffer.py              rollout storage with hidden states
    ppo.py                 recurrent PPO update
    selfplay.py            opponent pool (past checkpoints + scripted bots), sampling
    train.py               training loop + logging
scripts/
  render_replay.py         play one match (policy/bot vs policy/bot) and render it
  run_eval.py              round-robin of named agents -> report.md
  train.py                 thin wrapper around sentry_sim.ppo.train
tests/
  test_rules.py            HP, lifesteal, ammo, fire rate, timer, win logic
  test_geometry.py         bunker boxes, LOS, rays, collisions, frame symmetry
  test_shooting.py         hit model monotonicity and limits
  test_env.py              shapes, determinism, canonical-frame symmetry, speed smoke test
  test_bots.py             each bot runs, basic behavior fingerprints
  test_eval.py             CI math, report generation on a fixture
configs/
  default.yaml             dumped SimConfig (source of truth is config.py; yaml is for overrides)
reports/                   evaluation outputs (committed: markdown + small gifs only)
```

## Data flow (one step, all N envs at once)
```
actions_red, actions_blue  (canonical frame, [N,5] each)
      │  frames.canonical_to_world (blue only: flip velocity sign, yaw+π)
      ▼
env.step:
  1. chassis: v_cmd -> first-order lag -> integrate -> wall clip -> bunker push-out -> robot-robot push-out
  2. turret: yaw += yaw_rate*dt (wrap), pitch += pitch_rate*dt (clip)
  3. fire: if fire_on and cooldown<=0 and ammo>0: shot event; cooldown = 1/fire_rate; ammo -= 1
  4. for each shot: p_hit = shooting.hit_probability(state) ; hit ~ Bernoulli(p)
  5. hit: target HP -= 20 ; shooter HP = min(400, HP+10)
  6. t += dt ; done = HP<=0 | t>=120 ; outcome via win ladder (kill > more HP > draw)
  7. reward = terminal ± 1 + shaping c*Δ(HP_me − HP_opp)/400   (zero-sum)
  8. visibility: LOS clear & in FoV & range<=3 m  -> update last_seen / time_since_seen
  9. obs_actor_{red,blue}, obs_critic_{red,blue} in each robot's canonical frame
 10. auto-reset envs that are done (infos carry final outcome)
```

## Time stepping
`dt = 0.05 s` (20 Hz), so one 120 s match = 2400 steps. Fire rate 3 shots/s is enforced by a per-robot cooldown of 1/3 s, so a robot holding fire gets shots at t = 0, 0.333, 0.667, … (one shot every ~6.67 steps; implemented with a float cooldown, not an integer counter, so there is no aliasing to 2.86 or 3.33 shots/s).

## Policy
Recurrent PPO: shared GRU trunk (hidden 256) on the actor; separate critic MLP/GRU on the privileged observation. Action heads: 4 Gaussian (vx, vy, yaw rate, pitch rate; tanh-squashed to [−1,1]) + 1 Bernoulli (fire). The same network is used for Red and Blue (the env presents each side in canonical frame). Self-play: at each rollout, each env's opponent is sampled from {current policy (mirror), past checkpoints (prioritized by their win rate vs the learner), scripted bots}. Details in `02-evaluation.md` and WP7/WP8.

## Performance target
- CPU, 1024 envs: ≥ 20 k env-steps/s (both robots counted as one env-step). Enough for tests and bot round-robins.
- GPU, 8192 envs: ≥ 400 k env-steps/s. One generation of 8192 matches (≈ 20 M steps) then takes ≈ 50 s of sim time. Measured numbers go into `reports/perf.md` (WP3 acceptance).

## Out of scope for v0 (next additions, in this order)
1. Full win ladder (first-damage, shots-fired tie-breaks) — trivial to add, kept out so v0 outcomes are easy to reason about.
2. Barrel heat (10/shot, cool 30/s, lock > 260) and chassis power (120 W, 60 J buffer).
3. Yellow-card proxies (robot contact, wall contact 2 HP).
4. Sensing noise: pose noise, detection dropout, latency; muzzle-speed distribution.
5. Chassis spin as an action (plate-facing then becomes controllable).
6. Projectile physics replacing the probabilistic hit model.
