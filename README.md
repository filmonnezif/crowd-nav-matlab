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
```

Phase 1 needs **no toolboxes**. Later phases use the Deep Learning
Toolbox (LSTM, CNN), Statistics and Machine Learning Toolbox (GP), and
optionally Model Predictive Control Toolbox (nlmpc).

## Layout

| Folder | What is in it |
|---|---|
| `data/raw/` | The 6 ETH/UCY files: frame, ped_id, x, y (m), 2.5 Hz |
| `src/data/` | Loading, windowing (8 observed + 12 future steps) and the leak-free split |
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
