# Phases

Each phase ends with something that runs in MATLAB and a row in the
results table. The paper compares every layer's classical (course)
version with a learned one.

## Phase 1 — Data and simulator  ✅ done
- Leak-free train/val/test split from raw ETH/UCY; static windows flagged
- Crowd replay with interpolation between 0.4 s frames
- Differential-drive robot, occupancy/cost grid, A*, pure pursuit, speed governor
- Metrics: success, collisions, min clearance, time, path ratio, freezing, personal-space intrusions
- `demo_phase1` animation + MP4; `exp_phase1_baseline` over 6 scenes × 20 episodes

## Phase 2 — Tracking and classical predictors  ✅ done
- Kalman filter (constant-velocity model) to estimate velocity from noisy positions
- Predictors: constant velocity, Kalman rollout, Social Force
- ADE/FDE on the clean val and ETH test sets
- Plug predictions into the planner (`cfg.obstacleFcn`) and re-run the baseline experiment

## Phase 3 — Learned predictors  ✅ done (+ DWA controller)
- LSTM encoder–decoder on relative displacements
- Temporal 1D CNN on the 8 observed velocities (course CNN)
- Social-grid 2D CNN: occupancy grid of neighbours around each person
- Small transformer (if time allows)
- Same ADE/FDE table and the same navigation experiment

## Phase 4 — Uncertainty  ✅ done
- Gaussian process regression baseline
- Split conformal prediction around every predictor: a per-horizon radius with a coverage guarantee
- Planner inflates each predicted position by its conformal radius
- Adaptive conformal (ACI) for distribution shift; Kalman covariance vs conformal
- `demo_phase4` draws the uncertainty rings

## Phase 5 — Better planning and a reactive crowd  ✅ done
- Space-time A* over predicted positions (time as the third axis), exact DP over the time-layered grid
- Sampling MPC (cross-entropy method) that can slow down or swerve; nlmpc optional in MATLAB
- Reactive Social Force crowd so people respond to the robot, plus a "people pushed" metric

## Phase 6 — Demo  ✅ done
- One animated scene: predictions, uncertainty circles, robot plan
- Predictor switch (CV / Kalman / LSTM / CNN) for side-by-side comparison
- VideoWriter export for the presentation
- `demo_phase6`: side-by-side video, stills, summary CSV and an interactive viewer (`view_episodes`)

## Phase 7 — Experiments and paper
- Full grid: predictors × uncertainty on/off × planners × scenes
- Prediction accuracy vs navigation safety (does a better ADE give fewer collisions?)
- Write-up
