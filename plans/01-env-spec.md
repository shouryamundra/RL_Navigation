# 01 — Environment spec (v0)

All units SI (m, s, rad). World frame: origin bottom-left of the 5×5 m field, x right, y up, yaw counter-clockwise from +x. N = number of parallel envs; every state tensor has leading dim N. Index 0 = Red, 1 = Blue where a tensor has a robot dim.

## 1. Rules implemented (EM citations)
| Rule | Value | Source |
|---|---|---|
| Match length | 120 s | EM §5 ("two-minute Battle") |
| Max HP, start HP | 400 | EM Table 3-4 |
| 17 mm hit damage | 20 HP | EM Table 3-8 |
| Lifesteal | shooter recovers 50 % of damage dealt = +10, capped at 400 | EM §4.3.2 |
| Ammo | 250, no resupply | EM Table 5-2 |
| Fire rate | fixed 3 shots/s while fire is on (v0 stand-in for the heat rule: 10 heat/shot, cooling 30/s ⇒ 3/s sustained) | EM §3.3.1.2 |
| End of match | a robot reaches 0 HP, or 120 s | EM §4.4 |
| Winner (v0) | kill → win; else more HP → win; else draw. (EM §4.4 steps 3–6 — first damage, shots fired, first shot, weight — are **not** in v0.) | EM §4.4 |
| Field | 5 × 5 m, walls on all sides | EM §4.2 |
| Red start zone | x ∈ [0, 1], y ∈ [4, 5] | EM Fig 4-3 |
| Blue start zone | x ∈ [4, 5], y ∈ [0, 1] | EM Fig 4-3 |
| Bunker A | centerline (1.40, 2.15) → (2.00, 2.95); 1.0 long × 0.17 thick × 0.422 high | EM Fig 4-3, 4-4 |
| Bunker B | centerline (3.00, 2.05) → (3.60, 2.85); same size | EM Fig 4-3, 4-4 |

Bunker thickness is centered on the centerline (±0.085 m); the drawing does not say which side it extends to, and the ±10 % tolerance covers the difference. Config field `bunker_thickness_offset` (default 0) shifts it if we learn otherwise.

Not modelled in v0: setup period, heat, chassis power, speed-limit penalties, cards, collision damage, Referee latency, sensing noise.

## 2. State (per env)
| Tensor | Shape | Meaning |
|---|---|---|
| `pos` | [N,2,2] | robot center (x, y) |
| `vel` | [N,2,2] | robot velocity (world) |
| `chassis_yaw` | [N,2] | fixed in v0: Red 0, Blue π (plate normals at yaw + k·π/2) |
| `turret_yaw` | [N,2] | world-frame turret yaw, wrapped to (−π, π] |
| `turret_pitch` | [N,2] | clipped to [pitch_min, pitch_max] |
| `hp` | [N,2] | float, 0…400 |
| `ammo` | [N,2] | int, 0…250 |
| `fire_cooldown` | [N,2] | seconds until next shot allowed |
| `shots_fired`, `hits_landed` | [N,2] | counters |
| `t` | [N] | match time |
| `last_seen_pos` | [N,2,2] | where robot i last saw its opponent (world); start = opponent start-zone center |
| `time_since_seen` | [N,2] | seconds; starts at `vision_memory_cap` (never seen) |
| `visible` | [N,2] | bool: robot i currently sees its opponent |
| `done`, `outcome` | [N], [N] | outcome ∈ {0 draw, 1 red wins, 2 blue wins}; valid when done |

Start: `pos` uniform in own start zone shrunk by robot radius; `vel` 0; `turret_yaw` pointing at field center (+ uniform ±30°); `turret_pitch` 0; HP 400; ammo 250; cooldown 0; t 0.

## 3. Action (per robot, canonical frame, 5 dims)
| Index | Name | Range | Mapping |
|---|---|---|---|
| 0, 1 | `v_cmd_x`, `v_cmd_y` | [−1, 1] | × `v_max` (2.0 m/s); the vector is clipped to norm ≤ `v_max` |
| 2 | `yaw_rate` | [−1, 1] | × `turret_yaw_rate_max` (2π rad/s) |
| 3 | `pitch_rate` | [−1, 1] | × `turret_pitch_rate_max` (π rad/s) |
| 4 | `fire` | {0, 1} | fire toggle for this step |

Chassis dynamics: `vel += (v_cmd − vel) · dt / tau_v` (tau_v = 0.15 s), `pos += vel·dt`, then collision resolution. Turret integrates rates directly (no lag). Canonical → world for Blue: negate velocity, add π to the resulting turret yaw target (implemented by transforming the yaw *state*, not the rate; rates are frame-independent).

## 4. Collision
Robots are discs of radius `robot_radius` (0.30 m). After integration:
1. Walls: clip center to [r, 5 − r]; zero the velocity component into the wall.
2. Bunkers: each bunker is an oriented box (center, half-length 0.5, half-thickness 0.085, angle 53.13°). Transform the disc center into box frame, compute the closest point on the box, push the disc out along the penetration normal, zero the normal velocity component.
3. Robot–robot: if |p_red − p_blue| < 2r, push both apart equally along the center line.
One pass of 1–3 is enough at 20 Hz with v ≤ 2 m/s (penetration ≤ 0.1 m/step).

## 5. Visibility (what the actor may observe)
Robot i sees its opponent j iff all of:
1. `|p_j − p_i| ≤ vision_range` (3.0 m),
2. `|wrap(bearing_ij − turret_yaw_i)| ≤ fov / 2` (fov = 70°),
3. the 3D segment from (p_i, `camera_height` = 0.50) to (p_j, `target_body_height` = 0.35) does not intersect any bunker box (height 0.422) — walls can never be between two robots inside the field.
When visible: `last_seen_pos ← p_j`, `time_since_seen ← 0`; else `time_since_seen += dt` (capped at `vision_memory_cap` = 10 s).

## 6. Shooting model (per shot, no projectile physics)
Shooter i fires at j. Let `d = |p_j − p_i|`, flight time `τ = d / v_proj` (v_proj = 22 m/s), lead point `q = p_j + v_j·τ`.

Ideal aim at the lead point: `yaw* = atan2(q − p_i)`, `pitch* = atan2(plate_height − turret_height, |q − p_i|)` (plate_height 0.20, turret_height 0.50 ⇒ shots go slightly downward).

Errors: `e_yaw = wrap(turret_yaw_i − yaw*)`, `e_pitch = turret_pitch_i − pitch*`.

Angular tolerance grows with the plate's angular size: `σ_yaw(d) = sqrt(σ_disp² + (plate_half_width / d)²)`, `σ_pitch(d) = sqrt(σ_disp² + (plate_half_height / d)²)` with σ_disp = 0.6° (hardware dispersion), plate half-width 0.07 m, half-height 0.06 m.

```
p_aim    = exp(−0.5 (e_yaw/σ_yaw)²) · exp(−0.5 (e_pitch/σ_pitch)²)
p_range  = 1 / (1 + (d/d0)^k)                 d0 = 1.5 m, k = 3   (0.96 @0.5 m, 0.77 @1 m, 0.30 @2 m, 0.11 @3 m)
ω_cross  = |v_j − v_i|_perp / d               target angular rate across the LOS, rad/s
p_motion = exp(−0.5 (ω_cross/ω0)²)            ω0 = 2.0 rad/s   (a 2 m/s crossing target at 1 m: ω=2 → 0.61)
φ        = angle between the LOS (j→i) and the nearest plate normal of j, in [0, π/4] (4 plates at 90°)
p_plate  = cos(φ)^a                           a = 2  (1.0 head-on, 0.5 at the 45° corner)
clear    = 3D segment (p_i, turret_height) → (q, plate_height) misses every bunker box
p_hit    = clear · p_range · p_aim · p_motion · p_plate · [d ≤ shot_range_max (4.0 m)]
```
The lead error is real, not a penalty term: a policy that aims at the *current* opponent position misses a crossing target by `ω_cross·τ` radians, so the actor must learn to lead using the observed opponent velocity. All constants live in `SimConfig.shooting`.

Hit resolution: `hit ~ Bernoulli(p_hit)` per shot using the env's torch.Generator; damage/lifesteal applied immediately (no flight delay in v0).

## 7. Observations
All in the observer's canonical frame (Red: world; Blue: rotate 180° about (2.5, 2.5): `p' = 5 − p`, `v' = −v`, `yaw' = yaw + π`). Normalised to roughly [−1, 1].

### 7.1 Actor observation (`obs_dim = 14 + 9 + n_rays + 5`)
| Group | Features | Count |
|---|---|---|
| Own | x/5, y/5, vx/v_max, vy/v_max, cos(turret_yaw), sin(turret_yaw), pitch/pitch_max, hp/400, ammo/250, t_left/120, cooldown_ready (0/1), shots_fired/250, hits_landed/50, is_blue (0/1 — lets the policy break symmetry if the field tolerance ever matters) | 14 |
| Opponent | visible (0/1); if visible: rel_x/5, rel_y/5, rel_vx/v_max, rel_vy/v_max, cos/sin of (bearing − turret_yaw) — else zeros; time_since_seen/vision_memory_cap; last_seen rel_x/5, last_seen rel_y/5 | 9 |
| Map | `n_rays` = 16 ray distances /5, cast from own center in canonical-frame directions 2πk/16 against walls and bunkers (not the opponent) | 16 |
| Last action | previous action vector | 5 |
Total 44. Opponent HP is **not** observed in v0 (open question whether the Referee feed exposes it); the recurrent policy can infer it from hits landed.

### 7.2 Critic observation
Actor obs (44) ⊕ true opponent state in the same canonical frame: rel_x, rel_y, rel_vx, rel_vy, cos/sin(opp turret yaw), opp pitch, opp hp/400, opp ammo/250, opp_sees_me (0/1), opp time_since_seen, opp last action (5) ⇒ +16 = 60.

### 7.3 Reward (per robot, per step)
`r = r_terminal + c_shape · Δ(hp_me − hp_opp)/400`, `c_shape = 0.5` (so one net hit ≈ ±0.0375), `r_terminal` = +1 win / −1 loss / 0 draw at the terminal step. Zero-sum between the two robots. The HP term is a potential difference, so it does not change the optimal policy and will be annealed to 0 late in training.

## 8. Config (`SimConfig`, defaults)
```yaml
field: {size: 5.0,
        red_start: [0.0, 1.0, 4.0, 5.0], blue_start: [4.0, 5.0, 0.0, 1.0],   # xmin xmax ymin ymax
        bunkers: [[1.40, 2.15, 2.00, 2.95], [3.00, 2.05, 3.60, 2.85]],         # centerline x0 y0 x1 y1
        bunker_thickness: 0.17, bunker_height: 0.422, bunker_thickness_offset: 0.0}
rules:  {match_time: 120.0, max_hp: 400.0, hit_damage: 20.0, lifesteal_frac: 0.5, ammo: 250, fire_rate: 3.0}
robot:  {radius: 0.30, v_max: 2.0, tau_v: 0.15, turret_yaw_rate_max: 6.2832, turret_pitch_rate_max: 3.1416,
         pitch_min: -0.5236, pitch_max: 0.5236, turret_height: 0.50, plate_height: 0.20,
         plate_half_width: 0.07, plate_half_height: 0.06, camera_height: 0.50, target_body_height: 0.35}
vision: {range: 3.0, fov: 1.2217, memory_cap: 10.0, n_rays: 16}
shooting: {v_proj: 22.0, d0: 1.5, k: 3.0, sigma_disp: 0.0105, omega0: 2.0, plate_exp: 2.0, range_max: 4.0}
reward: {win: 1.0, loss: -1.0, draw: 0.0, c_shape: 0.5}
sim:    {dt: 0.05, start_yaw_jitter: 0.5236}
```
`SimConfig.load(path)` / `.save(path)`; `SimConfig.hash()` goes into every report and checkpoint.

## 9. Env API
```python
env = SentryEnv(cfg, n_envs, device, seed)
obs = env.reset()                      # dict: actor [N,2,44], critic [N,2,60], hidden-state reset mask
obs, rew, done, info = env.step(actions)   # actions [N,2,5] in each robot's canonical frame
# rew [N,2], done [N] (auto-reset; info['outcome'] [N] valid where done, info['hp'] [N,2], info['shots'], info['hits'])
env.state_snapshot() -> dict of cpu tensors for the replay recorder
```
Both robots act simultaneously; the policy and bots produce `[N,5]` for their side. A helper `play_match(agent_red, agent_blue, n, seed)` runs n matches and returns outcomes + optional trajectory log.
