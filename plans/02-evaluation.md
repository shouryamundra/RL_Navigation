# 02 — Evaluation plan (v0)

## Agents under evaluation
- Scripted bots (`sentry_sim/bots.py`), all operating on the **actor observation only** (same information as the policy; no cheating):
  - `static` — stays at its start pose, sweeps the turret ±FoV around the direction of the opponent's start zone, fires whenever the opponent is visible and roughly in front of the turret.
  - `charger` — drives straight at the last-known opponent position (opponent start-zone center until first sighting), steering around bunkers with the ray distances; aims at the opponent with a lead when visible; fires when visible and the aim error is small.
  - `cover_camper` — moves to a waypoint behind the bunker nearest its start zone (opposite side from the enemy start), peeks out when `time_since_seen` is large, retreats behind cover for 2 s after being hit; fires when visible.
  - `random_walker` — random velocity resampled every 1 s, random turret sweep, fires when visible. Sanity floor: anything should beat it.
- `policy:<checkpoint>` — a trained recurrent PPO checkpoint.
- `mirror` — during training only: the current policy against itself.

## Metrics per (agent A, agent B, colour) cell
Played as `n_matches` (default 500) matches per colour assignment with a fixed seed list, so reruns reproduce exactly.
- **Win rate** (wins / n, draws count 0; also reported: draw rate) with a **Wilson 95 % CI**.
- Kill rate (matches ending by 0 HP), mean match length.
- Mean final HP difference (A − B).
- Shots fired, hits landed, hit rate, mean firing range, mean time opponent visible — the behavior fingerprint.
- Pooled over both colours: the headline number, plus the per-colour split (to catch frame bugs: a symmetric bot pair must show symmetric results within CI).

## Outputs
`scripts/run_eval.py --agents static charger cover_camper random_walker policy:ckpt.pt --n 500 --seed 0 --out reports/eval_<date>.md`
- Markdown round-robin matrix (row beats column, pooled win rate ± CI), one matrix per colour, and a fingerprint table.
- `reports/*.json` with the raw per-match outcomes (for later re-analysis).
- `--render k` additionally writes k replay gifs per cell to `reports/replays/`.

## Replay
`scripts/render_replay.py --red charger --blue policy:ckpt.pt --seed 3 --out reports/replays/x.gif`
Top-down matplotlib animation at 10 fps: field, bunkers, start zones, robots as discs with a turret line (length ∝ FoV cone drawn faintly), HP bars, ammo, time, visible-flag, shot markers (hit = filled, miss = hollow), trail of last 2 s. Also a `--still t` option for a single frame (used in tests and the report).

## Training-time evaluation
Every `eval_every` updates: policy vs each scripted bot (both colours, 256 matches each, fixed seeds) and vs the 3 most recent pool checkpoints. Logged: win rate + CI, fingerprint, episode return, entropy, KL, value loss, env-steps/s. A checkpoint is added to the self-play pool when it beats the newest pool member ≥ 55 % pooled (or every `pool_every` updates, whichever first).

## Acceptance gates for the deliverables
- **D1 (env):** all rules tests pass; a 1024-env smoke run of 2400 steps with random actions finishes with no NaNs; `static` vs `static` in symmetric poses gives ~50 % per colour (within CI) — proves the canonical-frame flip.
- **D2 (bots + eval):** the round-robin runs in one command; `charger` and `cover_camper` beat `random_walker` and `static` with the 95 % CI lower bound > 50 %; per-colour results for each bot vs itself are symmetric within CI.
- **D3 (PPO):** from scratch vs `static` only: ≥ 80 % win rate within the first training hour on one GPU (or the equivalent CPU budget); then vs the mixed pool: pooled win rate vs every scripted bot ≥ 70 % with CI lower bound > 60 %.
- **D4 (report):** `reports/v0-report.md` with the round-robin matrix, 3–5 replay gifs, a list of observed behaviors (with the fingerprint numbers that back each claim), and the ordered list of fidelity additions.

## Statistics
Wilson score interval, z = 1.96. With n = 500 and p̂ = 0.7, the CI is ±0.04; with n = 1000 (both colours pooled) ±0.028. Draws are counted as non-wins in the win rate and reported separately.
