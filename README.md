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
run_tests          % 6 quick checks, < 30 s
demo_phase1        % clean data, run the robot, animate, save MP4
```

Phase 1 needs **no toolboxes**. Later phases use the Deep Learning
Toolbox (LSTM, CNN), Statistics and Machine Learning Toolbox (GP), and
optionally Model Predictive Control Toolbox (nlmpc).

## Layout

| Folder | What is in it |
|---|---|
| `data/raw/` | The 6 ETH/UCY files: frame, ped_id, x, y (m), 2.5 Hz |
| `src/data/` | Loading, windowing (8 observed + 12 future steps) and the leak-free split |
| `src/sim/` | Crowd replay, diff-drive robot, cost map, A*, pure pursuit, metrics, animation |
| `demos/` | `demo_phase1.m` (more demos per phase) |
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
university scenes it freezes about 40% of the time. Most contacts are
people walking into the robot's path from the side, which it could
only avoid by knowing where they are heading. That is the gap the
predictors in Phases 2 to 4 are meant to close.
