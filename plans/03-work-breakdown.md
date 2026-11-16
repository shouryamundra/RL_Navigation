# 03 — Work breakdown with acceptance tests (v0)

Each work package (WP) is self-contained enough to hand to another agent: scope, files, API, tests, and what not to do. Dependencies: WP1 → WP2 → WP3 → {WP4, WP5} → WP6 → WP7 → WP8 → WP9. Conventions: Python 3.10+, PyTorch, pytest, `ruff` clean, no loops over envs, every constant from `SimConfig`.

Deliverable mapping (from the brief): D1 = WP1–WP4, D2 = WP5–WP6, D3 = WP7–WP8, D4 = WP9.

---

## WP1 — Config + geometry
**Files:** `sentry_sim/config.py`, `sentry_sim/geometry.py`, `sentry_sim/frames.py`, `configs/default.yaml`, `tests/test_geometry.py`.

**API**
```python
cfg = SimConfig()                     # defaults from plans/01-env-spec.md §8
cfg.save(path); SimConfig.load(path); cfg.hash() -> str
geo = FieldGeometry(cfg, device)      # precomputed bunker boxes: centers [B,2], half-extents [B,2], rotation [B,2,2]
geo.point_in_bunker(p: [..,2]) -> bool[..]
geo.segment_blocked_2d(p0, p1) -> bool            # any bunker box intersects the segment (top-down)
geo.segment_blocked_3d(p0, h0, p1, h1) -> bool    # blocked only if the segment's height where it crosses the box is < bunker_height
geo.ray_distances(p: [N,2], angles: [K]) -> [N,K] # to walls and bunkers, capped at field diagonal
geo.resolve_collisions(pos [N,2,2], vel [N,2,2]) -> (pos, vel)   # walls, bunkers, robot-robot
frames.to_canonical(x, is_blue), frames.to_world(x, is_blue)     # for positions, velocities, yaws
```
**Acceptance tests**
1. The four bunker corner points are 1.0 apart along the centerline and 0.17 across it; centerline angle = atan2(0.8, 0.6) = 53.13° ± 0.01°; the two bunkers are point-symmetric about (2.5, 2.5) to 1e-6.
2. `segment_blocked_2d` agrees with an independent shapely computation on 1000 random segments (shapely is a test-only dependency); the segment from Red start center to Blue start center and the segment between the two bunker centers are both included explicitly.
3. `segment_blocked_3d` with both heights 0.5 > 0.422 is never blocked; with both heights 0.2 it equals `segment_blocked_2d`; with h0 = 0.5, h1 = 0.2 a bunker crossed at fraction t < (0.5 − 0.422)/0.3 = 0.26 does not block and at t > 0.26 does.
4. Ray distances from the field center at angles 0, π/2, π, 3π/2 equal 2.5 ± 1e-6 unless a bunker is in the way (compare against shapely).
5. A disc placed 0.1 m inside a bunker is pushed out to exactly `radius` from the box surface; a disc at the wall is clipped; two overlapping discs end up 2r apart.
6. `to_world(to_canonical(x, blue), blue) == x` for positions, velocities, yaws; `to_canonical(red_start_center, blue=True) == blue_start_center`-mirrored check: Blue's own start zone maps onto Red's.
**Do not:** add noise, randomize geometry, or model wall height.

---

## WP2 — Shooting model
**Files:** `sentry_sim/shooting.py`, `tests/test_shooting.py`.

**API**
```python
hit_probability(cfg, geo, p_i, v_i, yaw_i, pitch_i, p_j, v_j, chassis_yaw_j) -> p [N]   # all batched
aim_solution(cfg, p_i, p_j, v_j) -> (yaw_star, pitch_star, tau)                          # shared with bots
```
Exactly the formula in `plans/01-env-spec.md` §6, each factor exposed as its own function for tests and plotting.

**Acceptance tests**
1. Perfect aim at a static head-on target: p = p_range(d); p_range(0.5) > 0.95, p_range(1.5) = 0.5, p_range(3.0) < 0.15 (defaults).
2. Monotone: p decreases with |e_yaw|, |e_pitch|, d, ω_cross, φ (sampled grids).
3. Lead: for a target moving at 2 m/s across the LOS at d = 2 m, aiming at the current position gives p < 0.2 × the p obtained aiming at `aim_solution`'s yaw.
4. Occlusion: target directly behind a bunker (segment crosses it at t > 0.26) ⇒ p = 0; shooter right against the bunker shooting over it to a target 2 m beyond ⇒ p > 0.
5. Plate facing: p at φ = 45° equals 0.5 × p at φ = 0 (defaults) with everything else fixed.
6. `p ∈ [0, 1]`, no NaNs for d → 0 (clamp d ≥ 1e-3) and for all pitch values.
7. A plotting helper writes `reports/shooting_model.png` (p vs range for 3 crossing speeds) — checked visually, committed.
**Do not:** add projectile physics, randomness, or heat.

---

## WP3 — Vectorized env + observations
**Files:** `sentry_sim/env.py`, `sentry_sim/obs.py`, `tests/test_rules.py`, `tests/test_env.py`, `reports/perf.md`.

**API** — see `plans/01-env-spec.md` §9.

**Acceptance tests (rules, `test_rules.py`)** — drive the env with hand-set state (expose `env.state` tensors; tests may set them directly and call `env.step`).
1. A forced hit (test monkeypatches `hit_probability` to 1) does exactly 20 damage and heals the shooter 10; a shooter at 400 stays at 400; a shooter at 395 goes to 400 (cap), not 405.
2. Holding fire for 10 s at 3 shots/s fires 30 ± 1 shots; ammo decreases by the same count; with ammo 0 no shots; `shots_fired` counter matches.
3. Fire toggled every other step (10 Hz) still gives 3 shots/s (cooldown, not edge-triggered).
4. 20 forced hits on a 400-HP robot with the shooter at full HP ⇒ target HP 0, `done`, outcome = shooter wins, terminal reward +1 / −1, episode ends at that step even if t < 120.
5. No hits for 2400 steps ⇒ done at t = 120 s exactly (step 2400, not 2399 or 2401), outcome draw, rewards 0.
6. Red 380 vs Blue 400 at timeout ⇒ Blue wins (more HP).
7. Shaping: one hit changes the shooter's reward by `+c_shape·30/400` and the target's by the negative; the per-step rewards of the two robots sum to 0 always.
8. Auto-reset: after done, the next `step` starts a fresh match (HP 400, ammo 250, t = 0, positions inside own start zones).
**Acceptance tests (env, `test_env.py`)**
9. Shapes: actor obs [N,2,44], critic [N,2,60], rew [N,2], done [N]; obs in [−1.5, 1.5] after 2400 random steps; no NaN.
10. Determinism: two envs with the same seed and action sequence produce identical states; different seeds differ.
11. Symmetry: for a Blue robot at the 180°-rotated pose of a Red robot, the actor observations are equal (bit-for-bit up to float error) — including the ray distances.
12. Visibility: opponent at 2.9 m in front of the turret with clear LOS ⇒ visible; at 3.1 m ⇒ not; at 2.9 m but 40° off the turret axis ⇒ not (FoV 70°); behind a bunker ⇒ not; `time_since_seen` grows by dt per step while not visible and resets to 0 when seen; `last_seen_pos` holds.
13. Robots never end a step inside a bunker or outside the field (10 000 random steps, 256 envs).
14. Speed: `python -m sentry_sim.bench` prints env-steps/s for N ∈ {256, 1024, 4096, 8192} on CPU and (if available) GPU; the numbers go into `reports/perf.md`. Target: ≥ 20 k/s CPU @1024, ≥ 400 k/s GPU @8192.
**Do not:** implement bots, PPO, or the full win ladder.

---

## WP4 — Replay renderer
**Files:** `sentry_sim/replay.py`, `scripts/render_replay.py`, `tests/test_replay.py`.

**API**
```python
rec = Recorder(env, env_index=0)          # call rec.capture() after every env.step; stores cpu snapshots + shot events
rec.save_gif(path, fps=10) ; rec.save_mp4(path) (if ffmpeg) ; rec.still(path, t)
```
**Acceptance tests**
1. Recording a 2400-step match and saving a gif produces a file < 5 MB with 1200 frames at 10 fps (every 2nd step) in < 60 s on CPU.
2. A still frame at t = 0 shows both robots inside their start zones (test checks pixel colour at the robot positions against the start-zone colour — or simply that the function runs and the robot coordinates are inside the zones).
3. Shot events are drawn: a forced-hit match shows ≥ 1 hit marker (the recorder's event list is non-empty and the frame function is called with it).
**Do not:** build an interactive viewer.

---

## WP5 — Scripted bots
**Files:** `sentry_sim/bots.py`, `tests/test_bots.py`.

**API**
```python
class Bot:  # stateless w.r.t. env; may keep per-env tensors (e.g. waypoint, timers) reset via bot.reset(mask)
    def act(self, obs_actor: Tensor[N,44], is_blue: bool) -> Tensor[N,5]     # canonical frame
make_bot(name, cfg, n_envs, device) -> Bot    # names: static, charger, cover_camper, random_walker
```
Bots use only the actor observation (visible-only opponent info, rays, own state) plus `aim_solution` from WP2 for leading.
**Acceptance tests**
1. Each bot produces actions in range with no NaNs for 2400 steps over 256 envs.
2. `static`: mean displacement from the start pose < 0.05 m over a match.
3. `charger`: within 10 s it is within 1.5 m of the opponent's start-zone center in ≥ 90 % of envs vs a `static` opponent; never ends a step inside a bunker (env guarantees) and is not stuck: mean speed over the first 5 s > 0.5 m/s.
4. `cover_camper`: within 10 s its LOS to the enemy start-zone center is blocked by a bunker in ≥ 90 % of envs.
5. Fingerprints (via WP6, 200 matches each colour): `charger` and `cover_camper` each beat `random_walker` and `static` with CI lower bound > 0.5; `charger` vs `charger` per-colour win rates are symmetric within CI.
**Do not:** give bots the true opponent state; no learned components.

---

## WP6 — Round-robin evaluation
**Files:** `sentry_sim/eval.py`, `scripts/run_eval.py`, `tests/test_eval.py`.

**API**
```python
play_matches(cfg, agent_red, agent_blue, n, seed, device) -> MatchResults   # outcomes [n], hp [n,2], shots, hits, length, fingerprint
wilson(k, n, z=1.96) -> (lo, hi)
round_robin(cfg, agents: dict[str, AgentFactory], n, seed, device) -> RoundRobinResult
RoundRobinResult.to_markdown() ; .save_json(path)
```
Agents are loaded by name: bots by name, policies as `policy:<path>` (WP7 provides the loader; until then the eval tests use bots only).
**Acceptance tests**
1. `wilson(70, 100)` = (0.604, 0.781) ± 0.001; `wilson(0, 10)` lower = 0, `wilson(10, 10)` upper = 1.
2. Same seed twice ⇒ identical `RoundRobinResult`.
3. Colour split: results for A-as-red-vs-B and B-as-red-vs-A are both present; the pooled win rate is the average weighted by n.
4. Markdown output contains one row and one column per agent and the CI string `0.xx [0.xx, 0.xx]` in every off-diagonal cell.
5. End-to-end: `scripts/run_eval.py --agents static charger cover_camper random_walker --n 200` finishes in < 10 min on CPU and writes `reports/eval_bots.md` + json.
**Do not:** add Elo/TrueSkill or exploitability runs in v0.

---

## WP7 — Recurrent PPO (asymmetric critic)
**Files:** `sentry_sim/ppo/{nets.py,buffer.py,ppo.py,train.py}`, `scripts/train.py`, `tests/test_ppo.py`, `configs/ppo_default.yaml`.

**Design**
- Actor: MLP(44→256) → GRU(256) → heads: 4 Gaussian (state-dependent mean, state-independent log-std, tanh squash) + 1 Bernoulli logit. Critic: MLP(60→256) → GRU(256) → value. Separate parameters.
- Rollout: T = 128 steps × N envs (both robots in the same env batch give 2N actor samples when the opponent is the mirror policy; when the opponent is a bot or a frozen checkpoint only the learner's side enters the buffer). Hidden states stored at sequence starts; sequences truncated at episode ends via the done mask.
- PPO: clip 0.2, GAE λ 0.95, γ 0.995, 4 epochs, minibatches of 16 sequences × 128 steps, Adam 3e-4, entropy 0.003, value clip, grad-norm 0.5, advantage normalisation, KL early stop at 0.02. Observation normalisation (running mean/std, frozen at export).
- Curriculum: stage 1 vs `static` (+ 20 % `random_walker`); advance when win rate ≥ 80 %. Stage 2 vs all bots uniformly; advance at ≥ 70 % vs each. Stage 3 = WP8 self-play.
- Logging: TensorBoard + a CSV; checkpoints every `ckpt_every` updates with the config hash and the obs-normaliser.
**Acceptance tests**
1. `test_ppo.py`: a forward pass on a dummy batch has the right shapes; the loss decreases over 50 updates on a fixed synthetic batch (overfit test); checkpoint save → load reproduces the same actions bit-exactly for the same inputs and hidden state.
2. Stage 1 gate: ≥ 80 % vs `static` (both colours, 256 matches, CI lower > 70 %) within 1 h on one GPU. Record the curve in `reports/train_stage1.md`.
3. Stage 2 gate: ≥ 70 % vs every bot.
**Do not:** start self-play before stage 2 passes; do not add PPO-Lagrangian or BC warm start in v0.

---

## WP8 — Self-play
**Files:** `sentry_sim/ppo/selfplay.py`, additions to `train.py`.

**Design**
- Opponent pool: frozen checkpoints (max 20, add on the rule in `02-evaluation.md`) + the 4 bots. Per rollout, each env draws its opponent: 40 % current policy (mirror, both sides learn), 40 % pool checkpoint sampled ∝ (1 − win rate of the learner against it) + ε (PFSP-lite), 20 % bots.
- Learner colour drawn uniformly per env; the env's canonical frame makes this free.
- Evaluation every `eval_every` updates per `02-evaluation.md`.
**Acceptance tests**
1. Pool sampling unit test: with recorded win rates [0.9, 0.5, 0.1] the sampling frequencies are ordered [low, mid, high] over 10 000 draws.
2. Running self-play for the budget (target ~6 h on one GPU; report whatever the budget was) keeps the win rate vs all bots ≥ 70 % and the win rate vs the oldest pool checkpoint > 60 %.
3. A frozen checkpoint loaded as an opponent produces identical actions to the live net at save time (no normaliser drift).

---

## WP9 — Report
**Files:** `reports/v0-report.md`, `reports/replays/*.gif`.
Contents: round-robin matrix (bots + final policy + 2 intermediate checkpoints), fingerprints, 3–5 annotated replays, observed behaviors with numbers, training curves, env speed, and the ordered next-fidelity list with the specific change to the env for each (heat, power, penalties/contact, full win ladder, sensing noise, chassis spin). Acceptance: every behavior claim cites a fingerprint number or a replay.
