# Crowd navigation in MATLAB

A differential-drive robot crosses real pedestrian crowds (ETH/UCY
recordings). Machine learning predicts where people will walk next, and
the planner uses those predictions to slow down, swerve, and keep a
comfortable distance. Course project for EGN3113, built in phases (see
[PHASES.md](PHASES.md)).

## Quick start (MATLAB or MATLAB Online)

```matlab
cd crowd-nav-matlab
startup            % adds src/ folders to the path
run_tests          % quick checks, < 30 s
demo_phase1        % clean data, run the robot, animate, save video
demo_phase2        % same crowd without vs with Kalman prediction
train_predictors   % train the 5 networks (Deep Learning Toolbox, ~30 min)
demo_phase3        % LSTM + DWA vs Kalman + pure pursuit in a dense crowd
```

Phase 1 needs **no toolboxes**. Later phases use the Deep Learning
Toolbox (LSTM, CNN), Statistics and Machine Learning Toolbox (GP), and
optionally Model Predictive Control Toolbox (nlmpc).

## Layout

| Folder | What is in it |
|---|---|
| `data/raw/` | The 6 ETH/UCY files: frame, ped_id, x, y (m), 2.5 Hz |
| `src/data/` | Loading, windowing (8 observed + 12 future steps) and the leak-free split |
| `src/learn/` | Features, network definitions, learned predictor wrapper |
| `src/predict/` | Constant velocity, Kalman filter, Social Force, ADE/FDE evaluation |
| `src/sim/` | Crowd replay, diff-drive robot, cost map, A*, pure pursuit, metrics, animation |
| `demos/` | One demo per phase |
| `experiments/` | Batch runs that produce the numbers for the report |
| `tests/` | `run_tests.m` |
| `results/` | Generated videos, CSVs and .mat files (not committed) |

## Data

`data/raw/` holds the standard ETH/UCY files as distributed with the
Trajectron++ repository (biwi_eth, biwi_hotel, crowds_zara01/02,
students001/003); the same files are in the course SharePoint folder
`Pedestrian_Trajectory_Data/raw`. The original datasets are by
Pellegrini et al. (ETH, ICCV 2009) and Lerner et al. (UCY, 2007).

`build_dataset` rebuilds the splits from raw instead of using the
course's `processed/` files, because those had a train/val leak:

| | train | val | test (ETH) | val windows sharing frames with train |
|---|---|---|---|---|
| Old split (last 20% of windows) | 27,036 | 6,761 | 364 | 978 |
| `build_dataset` (one split time per scene) | 28,022 | 4,634 | 364 | **0** |

Windows that straddle a scene's split time (1,141) are dropped.
Near-static windows (moving < 0.5 m in 8 s; 58% of Hotel, 44% of
Zara2) are flagged in `isStatic`; `'DropStaticTrain', true` removes
them from training only.

## Phase 1 robot

Every 0.4 s, people's current positions become circles of radius
pedestrian + robot + 0.2 m, plus a soft "personal space" cost out to
0.8 m beyond that. A* (8-connected, octile heuristic) plans on a
0.25 m grid; pure pursuit follows the path and a governor slows the
robot as the nearest person gets within 1 m. The crowd is replayed, so
people do not react to the robot. This reactive, no-prediction robot is
the baseline every later phase is compared with.

### Phase 1 baseline results

`exp_phase1_baseline`: 20 episodes per scene (10 start times × both
directions), 60 s limit, robot top speed 1.0 m/s. Success = reached the
goal with no contact. Clearance is edge to edge, so negative means
contact.

| Scene | Success | Reached goal | Collisions / episode | Min clearance (m) | Time to goal (s) | Stopped |
|---|---|---|---|---|---|---|
| biwi_eth | 10% | 100% | 2.7 | -0.40 | 20.1 | 11% |
| biwi_hotel | 35% | 100% | 1.4 | -0.15 | 14.8 | 12% |
| crowds_zara01 | 30% | 100% | 1.1 | -0.14 | 17.2 | 11% |
| crowds_zara02 | 30% | 100% | 1.4 | -0.11 | 19.2 | 11% |
| students001 | 5% | 45% | 6.7 | -0.50 | 37.9 | 40% |
| students003 | 0% | 90% | 6.9 | -0.52 | 39.8 | 37% |

Planning only around where people are *now* almost always reaches the
goal but touches someone in most episodes, and in the dense
university scenes it freezes about 40% of the time. Because the crowd
is replayed, people never step aside, so the robot can only avoid them
by knowing where they are heading. That is the gap the predictors in
Phases 2 to 4 are meant to close.

## Phase 2: tracking and classical predictors

`src/predict/` has three predictors behind one interface,
`Y = fn(H, 12, 0.4)` (H = everyone's last 8 positions):

- **cv**: constant velocity from the last step.
- **kalman**: a constant-velocity Kalman filter (`kalman_cv.m`) smooths the
  noisy track and rolls forward; it also gives a covariance for Phase 4.
- **sf**: Social Force (Helbing & Molnar 1995). Everyone is simulated
  together, keeping their filtered velocity and pushing away from people ahead of them.

Kalman noise q and the Social Force strength A and range B were tuned on
**val only** (`tune_phase2`).

**Prediction accuracy** (`exp_phase2_prediction`, metres, ADE / FDE):

| | val | val, moving only | ETH test |
|---|---|---|---|
| cv | 0.440 / 0.972 | 0.562 / 1.245 | 1.075 / 2.282 |
| kalman | 0.443 / 0.977 | 0.565 / 1.250 | 1.071 / 2.277 |
| sf | 0.461 / 0.993 | 0.569 / 1.240 | 1.056 / 2.238 |

All three come out within about 5% of each other. Constant velocity is
a famously strong baseline on ETH/UCY (Schöller et al. 2020 report about
the same ETH numbers). The learned models in Phase 3 have to beat these numbers.

**Using predictions in the planner.** `make_obstacle_fcn('kalman')` turns
predictions into obstacles. A predicted position τ seconds ahead
becomes a hard obstacle only where the robot could actually meet the person
(|distance − v_max·τ| < 1 m); elsewhere it adds a soft cost. Predictions up
to 3.2 s ahead are used.

**Navigation** (`exp_navigation(name)`: the same 120 episodes as Phase 1 for every predictor):

| Predictor | Success | Reached | Collisions / ep | Min clearance (m) | Time (s) | Stopped | Personal-space intrusion (s) | Plan (ms) |
|---|---|---|---|---|---|---|---|---|
| none (Phase 1) | 18% | 89% | 3.38 | -0.31 | 23.2 | 20% | 10.2 | 65 |
| cv | 33% | 91% | 2.08 | -0.08 | 24.2 | 15% | 6.5 | 72 |
| kalman | 33% | 90% | 2.27 | -0.08 | 24.4 | 15% | 6.7 | 71 |
| sf | 34% | 91% | 2.05 | -0.05 | 23.9 | 14% | 6.2 | 72 |

Adding any prediction nearly doubles success and cuts collisions by about
a third, at about the same travel time. The three predictors are close
to each other, so in this setting knowing *that* people move matters
more than predicting *exactly how*. Zara1 reaches 80% success, while the dense
university scenes (students001/003) stay at 0–10%. They are the hard case for
the later phases.

## Phase 3: learned predictors and a smarter controller

### Networks (`src/learn/`, trained with `train_predictors` in MATLAB)

Every network sees a person's last 8 positions in their own **heading
frame** (origin at the person, x axis along their walking direction) and
outputs a **correction to constant velocity** for the next 12 steps
(residual learning). Training uses the clean train split plus
left/right mirrored copies (56,044 samples). Early stopping keeps the
best validation epoch.

| Network | What it looks at | Learnable params |
|---|---|---|
| `mlp` | the 7 step displacements, flattened | 38,040 |
| `lstm` | the 7 steps as a sequence (LSTM, 64 units) | 28,568 |
| `tcn` | temporal CNN: 1-D convolutions over the 7 steps | 62,968 |
| `sgcnn` | **social-grid CNN**: a 2-D CNN on a 16×16 grid (0.5 m cells) of neighbours' positions and velocities around the person, joined with their own motion | 58,488 |
| `tf` | small transformer: self-attention over the 7 steps | 45,528 |

`show_architectures` draws each network to `results/arch_*.png`;
`deepNetworkDesigner(predictor_net('sgcnn'))` opens one in Deep Network
Designer.

**Prediction accuracy** (ADE / FDE in metres; ETH is never seen in training):

| | val | val, moving only | ETH test |
|---|---|---|---|
| cv | 0.440 / 0.972 | 0.562 / 1.245 | 1.075 / 2.282 |
| kalman | 0.443 / 0.977 | 0.565 / 1.250 | 1.071 / 2.277 |
| sf | 0.461 / 0.993 | 0.569 / 1.240 | 1.056 / 2.238 |
| mlp | 0.431 / 0.938 | 0.541 / 1.181 | 1.030 / 2.091 |
| **lstm** | 0.440 / 0.958 | 0.548 / 1.196 | **1.003 / 2.045** |
| tcn | 0.430 / 0.936 | 0.541 / 1.180 | 1.021 / 2.058 |
| **sgcnn** | **0.429 / 0.934** | **0.529 / 1.158** | 1.044 / 2.155 |
| tf | 0.433 / 0.944 | 0.543 / 1.186 | 1.008 / 2.044 |

Every network beats every classical predictor. The FDE gain is about
10% on ETH. The social-grid CNN is best on the scenes it was trained on
(it uses the neighbours), while the LSTM and transformer transfer best
to the unseen ETH scene.

### DWA local controller (`dwa_control.m`, `cfg.controller = 'dwa'`)

Pure pursuit with a speed governor can only slow down, so it stops in
people's way. The Dynamic Window Approach tries 77 (v, w) commands,
including slow reversing, within the acceleration limits. It rolls each
one out for 2 s and checks it against where people are **predicted** to be
at each moment. It picks the command with the best progress, clearance,
speed and smoothness that keeps clear of everyone.

### Navigation (same 120 episodes as before)

A contact is **at fault** when the robot was moving toward the person at the
moment of contact (closing speed > 0.1 m/s). Otherwise the person walked
into it, which the replayed crowd does because it never yields.

| Predictor | Controller | Success | Reached | Collisions / ep | At-fault / ep | Min clearance (m) | Time (s) | Stopped | Univ. success / coll. |
|---|---|---|---|---|---|---|---|---|---|
| none | pp | 18% | 89% | 3.38 | 0.00 | -0.31 | 23.2 | 20% | 2% / 6.8 |
| none | dwa | 23% | 94% | 2.35 | 0.24 | -0.21 | 24.5 | 1% | 5% / 4.1 |
| cv | pp | 33% | 91% | 2.08 | 0.00 | -0.08 | 24.2 | 15% | 2% / 4.3 |
| cv | dwa | 48% | 90% | 1.06 | 0.33 | 0.01 | 24.1 | 1% | 22% / 1.7 |
| kalman | pp | 33% | 90% | 2.27 | 0.00 | -0.08 | 24.4 | 15% | 2% / 4.8 |
| kalman | dwa | 47% | 92% | 0.98 | 0.32 | 0.01 | 24.6 | 1% | 20% / 1.6 |
| sf | pp | 34% | 91% | 2.05 | 0.00 | -0.05 | 23.9 | 14% | 5% / 4.2 |
| sf | dwa | 47% | 90% | 1.00 | 0.29 | 0.02 | 23.9 | 1% | 20% / 1.6 |
| tcn | pp | 33% | 92% | 2.12 | 0.00 | -0.08 | 24.5 | 14% | 5% / 4.5 |
| lstm | pp | 35% | 92% | 2.41 | 0.00 | -0.09 | 24.7 | 15% | 5% / 5.2 |
| mlp | dwa | 48% | 90% | 1.04 | 0.33 | 0.01 | 24.0 | 1% | 18% / 1.8 |
| lstm | dwa | 47% | 93% | 0.99 | 0.30 | 0.01 | 24.5 | 1% | 20% / 1.6 |
| tcn | dwa | 44% | 92% | 1.11 | 0.34 | 0.00 | 23.9 | 1% | 15% / 1.9 |
| sgcnn | dwa | 43% | 89% | 1.10 | 0.31 | 0.00 | 23.6 | 1% | 10% / 1.9 |
| tf | dwa | 45% | 93% | 1.03 | 0.32 | 0.01 | 25.0 | 1% | 20% / 1.7 |

What this shows:

1. **The controller matters more than the predictor.** With any predictor,
   DWA instead of pure pursuit roughly halves contacts (≈2.1 → ≈1.0 per
   episode). It lifts success from about 33% to 47%, and in the dense university
   scenes from 2–5% to about 20%. It almost never stands still.
2. **DWA trades waiting for moving.** Pure pursuit is never at fault (it
   stops before touching anyone), but people walk into it. DWA keeps
   moving, so about 0.3 contacts per episode are its own. Phase 5's reactive
   crowd will show which behaviour real people prefer.
3. **Better prediction accuracy does not give safer navigation here.**
   The networks are 5–10% more accurate than constant velocity, yet
   navigation is within noise of it. This matches Poddar et al. (2023), whose
   MPC drove the same with constant velocity as with a deep predictor. The
   planner only uses predictions up to 3.2 s ahead, where everyone is close
   to constant velocity anyway. Phase 4 (uncertainty) asks whether *knowing
   how wrong* a prediction may be helps more than making it more accurate.

Learned-predictor runs need the trained models, so run them in MATLAB:
`exp_phase3_navigation`. Learned predictors fall back to constant velocity
for people seen for fewer than 8 frames, because the networks were never
trained on padded histories.

## Phase 4: how wrong can a prediction be?

Code in `src/uncertainty/`. Every predictor gets a circle around each
predicted position that should contain the true position 90% of the time.

### Methods

* **Split conformal prediction** (`conformal_calibrate`, `get_conformal`).
  Take the prediction errors on the validation set. For each horizon, the
  radius is the ⌈(n+1)(1−α)⌉-th smallest error. If test data looks like
  validation data, this radius has a guaranteed 1−α coverage, whatever the
  predictor (CV, Kalman, LSTM, CNN...). Two variants:
  `plain` (one radius per horizon) and `speed` (radius grows with the
  person's speed: r = (0.5 + speed)·q).
* **Adaptive conformal inference (ACI)** (`aci_offline`). Runs through the
  test scene in time order. Each time an outcome becomes known and the
  circle missed, the level is lowered so circles grow; each hit lets them
  shrink. Coverage then tracks 90% even when the test scene differs.
* **Model-based uncertainty.** The Kalman filter's own covariance, and a
  **Gaussian process** predictor (`train_gp`, `predict_gp`, 24 ARD GPs on
  1000 training windows) that gives a predictive standard deviation. A 90%
  circle is 2.146·σ.

### Do the circles contain the truth? (`exp_phase4_uncertainty`, α = 0.1)

Coverage on the ETH test scene (never seen in training or calibration),
mean radius in metres at 1.6 / 3.2 / 4.8 s ahead. Validation coverage of
the conformal methods is 90.0% by construction.

| Predictor | Method | ETH coverage | Radius 1.6 / 3.2 / 4.8 s |
|---|---|---|---|
| cv | conformal plain | 57% | 0.48 / 1.25 / 2.18 |
| cv | conformal speed | 64% | 0.65 / 1.74 / 3.03 |
| cv | ACI plain | 90% | 2.39 / 6.04 / 11.9 |
| cv | ACI speed | 91% | 3.65 / 9.25 / 13.9 |
| kalman | conformal speed | 64% | 0.65 / 1.75 / 3.04 |
| kalman | Kalman covariance | 98% | 1.84 / 4.85 / 8.69 |
| sf | conformal speed | 65% | 0.68 / 1.76 / 3.06 |
GP_ROWS

What this shows:

1. **The 90% guarantee breaks under distribution shift.** People in ETH walk
   about twice as fast as in the validation scenes (median 1.01 vs
   0.51 m/s), and validation has many people standing still. So radii
   calibrated on validation cover only 57–65% on ETH. Scaling the radius
   with speed helps (57% → 64%) but does not close the gap.
2. **ACI gets the coverage back but the circles become huge.** ETH errors
   are heavy-tailed, so 90% coverage needs radii of several metres. With a
   smaller step size (γ = 0.003) ACI gives 85% coverage with 1.6 m at 1.6 s.
   There is a real trade-off between coverage and how much room the robot
   has left.
3. **Model-based uncertainty is over-cautious.** The Kalman covariance
   covers 98% with circles 3–4 times bigger than conformal ones.

### Does uncertainty make the robot safer? (DWA, 120 episodes)

`make_obstacle_fcn('cv', struct('alpha', 0.1, 'cpMode', 'speed'))`
inflates every predicted person by its conformal radius, both in the A*
cost map and in DWA's clearance check.

| Predictor | Uncertainty | Success | Reached | Collisions / ep | At-fault / ep | Median min clearance (m) |
|---|---|---|---|---|---|---|
| cv | none | 47% | 90% | 1.06 | 0.33 | -0.02 |
| cv | conformal 90% plain | 44% | 76% | 0.88 | 0.38 | 0.04 |
| cv | conformal 90% speed | 47% | 80% | 0.86 | 0.42 | 0.04 |
| kalman | none | 47% | 93% | 0.98 | 0.32 | -0.02 |
| kalman | conformal 90% speed | 49% | 76% | 0.82 | 0.41 | 0.06 |
| kalman | conformal 70% speed | 48% | 81% | 0.97 | 0.39 | 0.06 |

Conformal margins cut contacts by about 15–20% (1.06 → 0.86 per episode)
and keep the robot further from people. But the larger circles block more
of the space, so the robot reaches the goal less often within the time
limit (90% → 76–80%), and overall success stays about the same. The extra
at-fault contacts come from the robot squeezing between inflated circles.
Phase 5 (space-time A*, MPC and a crowd that reacts) is where these
margins should pay off, because a planner that reasons over time can wait
or detour instead of getting boxed in.

`demo_phase4` shows CV + DWA with and without conformal margins on the
busiest part of ETH. The orange rings are the 1.6 / 3.2 / 4.8 s conformal
circles.
