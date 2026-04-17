=========================================================================
Warehouse Mobile Robot Navigation Through Crowded Aisles
Author:  Sahil Kanojia
=========================================================================


HOW TO RUN
==========

1. Open MATLAB R2021b or later

2. Navigate to the folder containing warehouse_environment.m

3. In the MATLAB Command Window type:
      warehouse_environment
   and press Enter

4. The simulation runs all four controllers automatically:
      Figure 1 - PID Baseline (runs first)
      Figure 2 - NL-MPC Conservative (runs second)
      Figure 3 - NL-MPC Aggressive (runs third)
      Figure 4 - Linear MPC Successive Linearisation (runs fourth)
      Figure 5 - All Controllers Comparison (appears at the end)

5. Results are printed to the Command Window after each controller finishes

6. The comparison figure is saved automatically as:
      topic1_all_controllers.png


DEPENDENCIES
============

- MATLAB R2021b or later
- Optimization Toolbox (required for fmincon and quadprog)
- No additional toolboxes required
- No external libraries or packages needed


EXPECTED RESULTS
================

Controller                   Time(s)  Collisions  Violations  ms/step
-------------------------------------------------------------------
PID Baseline                  40.7        80           0        < 0.1
NL-MPC Conservative (N=10)    26.1         9           0         9.09
NL-MPC Aggressive   (N=12)    25.6         0           0         9.80
Linear MPC SL       (N=10)    34.1        19          20         1.14


FILE STRUCTURE
==============

warehouse_environment.m   - Main simulation file (all 4 controllers)
README.txt                - This file


CONTROLLERS IMPLEMENTED
=======================

Section 4a - PID Pure-Pursuit Baseline
   Kp_v = 1.0, Kp_omega = 2.5, lookahead = 0.5 m
   No obstacle avoidance. Reactive clipping only.

Section 4b - Nonlinear MPC Conservative
   Solver:  fmincon (SQP algorithm)
   N = 10,  Q = 10,  R = 0.1,  D_SAFE = 0.3 m
   Full nonlinear robot dynamics used for prediction.
   Nonlinear obstacle avoidance constraints.

Section 4c - Nonlinear MPC Aggressive
   Solver:  fmincon (SQP algorithm)
   N = 12,  Q = 40,  R = 1.0,  D_SAFE = 0.2 m
   Warm-started with minimum forward velocity of 0.4 m/s.
   Longer horizon detects worker patrol cycles.

Section 4d - Linear MPC Successive Linearisation
   Solver:  quadprog
   N = 10,  Nc = 1,  Q = 50,  R = 0.5,  D_SAFE = 0.1 m
   Linearised Jacobian matrices A_k and B_k updated at each
   horizon step around predicted heading theta_k.
   Stop-and-turn fallback for heading errors > 0.6 rad.


HELPER FUNCTIONS (bottom of file)
==================================

robot_dynamics()            - Discrete-time unicycle kinematics
update_dynamic_obstacles()  - Worker patrol logic
check_collision()           - Circle vs rectangle and circle collision
get_obstacle_distances()    - Distance to all obstacles for MPC
mpc_cost()                  - NL-MPC objective function for fmincon
mpc_constraints()           - NL-MPC obstacle constraints for fmincon


KNOWN LIMITATIONS
=================

1. Dynamic obstacles (workers) are treated as quasi-static within
   the MPC prediction horizon. Worker positions are not predicted
   forward in time. This is a known limitation documented in the paper.

2. Linear MPC exhibits corner overshoot at 90-degree turns due to
   cos(theta)=0 in the linearised B matrix at theta=pi/2.
   This is a structural limitation of linearised models for
   non-holonomic systems, documented in Section IV of the paper.

3. No terminal cost or terminal constraints in the MPC formulation.
   This may affect theoretical stability guarantees.


APPROXIMATE RUN TIME
====================

PID:                    ~1 minute
NL-MPC Conservative:    ~5 minutes
NL-MPC Aggressive:      ~6 minutes
Linear MPC SL:          ~4 minutes
Total:                  ~16 minutes


NOTES
=====

- Environment is reset to identical initial conditions before each
  controller run for fair comparison.

- All controllers use the same reference path defined in Section 3.

- The comparison figure is saved as topic1_all_controllers.png in
  the same directory as the script.

=========================================================================
