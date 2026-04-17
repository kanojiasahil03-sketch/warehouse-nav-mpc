%% Topic 1 Starter Code: Warehouse Mobile Robot Navigation Through Crowded Aisles
% =========================================================================
% This file provides:
%   1. Differential-drive robot kinematic model (Section 4, robot_dynamics())
%   2. Warehouse environment with walls, shelves (static), and moving workers
%   3. A simple PID baseline controller for path following (no obstacle avoidance)
%   4. Collision detection and obstacle distance functions
%   5. Visualisation of the robot navigating the warehouse
%
% =========================================================================
% YOUR TASKS — What you must implement:
% =========================================================================
%
%   CONTROLLER 1 (baseline, already provided):
%     PID pure-pursuit controller — see Section 4 simulation loop
%
%   CONTROLLER 2 (you must implement — Week 2-3: MPC):
%     Model Predictive Control with obstacle-avoidance constraints
%     - Use the kinematic model in robot_dynamics() as your prediction model
%     - Add input constraints: |v| <= V_MAX, |omega| <= OMEGA_MAX
%     - Add obstacle avoidance: use get_obstacle_distances() for constraint formulation
%     - Tip: Use fmincon for nonlinear MPC, or linearise for linear MPC
%
%   WHERE TO ADD YOUR CODE:
%     Option A: Add a new section after Section 4 that re-runs the simulation
%               with your MPC controller replacing the PID block (lines 113-148)
%     Option B: Create a separate function my_mpc_controller(state, env, ref_path, ...)
%               and call it in a second simulation loop
%
%   METRICS TO COMPARE (PID vs MPC):
%     - Collision count (how many timesteps the robot overlaps an obstacle)
%     - Constraint violations (how often |v| or |omega| exceeds limits before clipping)
%     - Path completion time (seconds to reach the goal)
%     - Computational cost per step (use tic/toc)
%
%   EXPECTED BASELINE PID RESULTS (verify these before starting your MPC):
%     - Total time:            ~29.8 s
%     - Path length:           ~26.4 m
%     - Collisions:            ~97  (PID has NO obstacle avoidance!)
%     - Constraint violations: 0
%     - These numbers confirm the starter code is running correctly.
%
% Run: >> warehouse_environment
% =========================================================================
clear; clc; close all;

%% ========================================================================
% 1. PARAMETERS
% =========================================================================
V_MAX     = 0.8;     % m/s   — warehouse safety limit
OMEGA_MAX = 1.5;     % rad/s — turning rate limit
DT        = 0.05;    % simulation time step (s)
ROBOT_R   = 0.25;    % robot collision radius (m)

%% ========================================================================
% 2. WAREHOUSE ENVIRONMENT SETUP
% =========================================================================
env.width  = 12.0;  % metres
env.height = 8.0;

% Static obstacles: shelving racks [x_corner, y_corner, width, height]
env.shelves = [
    2.0, 1.0, 0.5, 2.5;   % Shelf row 1, bottom
    2.0, 4.5, 0.5, 2.5;   % Shelf row 1, top
    5.0, 1.0, 0.5, 2.5;   % Shelf row 2, bottom
    5.0, 4.5, 0.5, 2.5;   % Shelf row 2, top
    8.0, 1.0, 0.5, 2.5;   % Shelf row 3, bottom
    8.0, 4.5, 0.5, 2.5;   % Shelf row 3, top
];

% Dynamic obstacles: workers/forklifts patrolling aisles
env.dyn_obs(1).radius = 0.3;
env.dyn_obs(1).path   = [3.5, 0.5; 3.5, 7.5];  % walks up aisle 1
env.dyn_obs(1).speed  = 0.3;  % m/s
env.dyn_obs(1).pos    = [3.5, 0.5];
env.dyn_obs(1).dir    = 1;  % +1 forward, -1 backward along path
env.dyn_obs(1).idx    = 1;  % current path waypoint index

env.dyn_obs(2).radius = 0.3;
env.dyn_obs(2).path   = [6.5, 7.5; 6.5, 0.5];  % walks down aisle 2
env.dyn_obs(2).speed  = 0.4;
env.dyn_obs(2).pos    = [6.5, 7.5];
env.dyn_obs(2).dir    = 1;
env.dyn_obs(2).idx    = 1;

% % ========================================================================
% 3. REFERENCE PATH (pre-planned warehouse route)
% =========================================================================
waypoints = [
    0.5,  4.0;    % Start
    1.5,  4.0;    % Enter aisle
    3.5,  4.0;    % Aisle 1 middle
    3.5,  7.2;    % Up aisle 1 — slightly higher
    6.5,  7.2;    % Cross to aisle 2 — slightly higher
    6.5,  0.8;    % Down aisle 2 — slightly lower
    9.5,  0.8;    % Cross to aisle 3 — slightly lower
    9.5,  4.0;    % Aisle 3 middle
    11.0, 4.0;    % Goal
];


% Interpolate between waypoints to create a dense path
ref_path = [];
for i = 1:size(waypoints,1)-1
    s = waypoints(i,:);
    e = waypoints(i+1,:);
    d = norm(e - s);
    n_pts = max(round(d / 0.1), 2);
    t_vals = linspace(0, 1, n_pts);
    for j = 1:length(t_vals)-1
        ref_path = [ref_path; s + t_vals(j)*(e - s)];
    end
end
ref_path = [ref_path; waypoints(end,:)];

%% ========================================================================
% 4a. SIMULATION — PID Controller
% =========================================================================
state     = [0.5; 4.0; 0.0];
path_idx  = 1;
lookahead = 0.5;

Kp_v     = 1.0;
Kp_omega = 2.5;

T_MAX   = 120.0;
n_steps = round(T_MAX / DT);
traj    = zeros(n_steps+1, 3);
ctrls   = zeros(n_steps, 2);
traj(1,:) = state';

collisions            = 0;
constraint_violations = 0;

% --- Setup live figure ---
fig = figure('Position', [100 100 800 600]);
ax_map = axes('Parent', fig);
hold(ax_map, 'on'); axis(ax_map, 'equal');
xlim(ax_map, [-0.5, env.width+0.5]);
ylim(ax_map, [-0.5, env.height+0.5]);

for step = 1:n_steps
    env = update_dynamic_obstacles(env, DT);

    px = state(1); py = state(2); theta = state(3);

    % Advance path_idx: skip points already passed
    while path_idx < size(ref_path,1)
        d = norm(ref_path(path_idx,:) - [px, py]);
        if d >= lookahead
            break;
        end
        path_idx = path_idx + 1;
    end
    target = ref_path(min(path_idx, size(ref_path,1)),:);

    % Heading and distance errors
    desired_theta = atan2(target(2)-py, target(1)-px);
    heading_err   = desired_theta - theta;
    heading_err   = mod(heading_err + pi, 2*pi) - pi;
    dist_err      = norm(target - [px, py]);

    % P control
    v     = Kp_v * dist_err * cos(heading_err);
    omega = Kp_omega * heading_err;

    % Count violations before clipping
    if abs(v) > V_MAX || abs(omega) > OMEGA_MAX
        constraint_violations = constraint_violations + 1;
    end

    % Clip
    v     = max(-V_MAX, min(V_MAX, v));
    omega = max(-OMEGA_MAX, min(OMEGA_MAX, omega));
    ctrls(step,:) = [v, omega];

    % Propagate
    state = robot_dynamics(state, [v; omega], DT);
    traj(step+1,:) = state';

    % --- Live animation ---
    cla(ax_map);
    hold(ax_map, 'on');

    % Draw walls
    plot(ax_map, [0 env.width env.width 0 0], [0 0 env.height env.height 0], 'k-', 'LineWidth', 2);

    % Draw shelves
    for i = 1:size(env.shelves,1)
        rectangle('Parent', ax_map, 'Position', env.shelves(i,:), ...
            'FaceColor', [0.6 0.3 0.1], 'EdgeColor', [0.4 0.2 0.05]);
    end

    % Draw reference path
    plot(ax_map, ref_path(:,1), ref_path(:,2), 'g--', 'LineWidth', 1);

    % Draw trajectory so far
    plot(ax_map, traj(1:step+1,1), traj(1:step+1,2), 'b-', 'LineWidth', 1.5);

    % Draw dynamic obstacles
    for i = 1:length(env.dyn_obs)
        th = linspace(0, 2*pi, 30);
        fill(ax_map, env.dyn_obs(i).pos(1) + env.dyn_obs(i).radius*cos(th), ...
             env.dyn_obs(i).pos(2) + env.dyn_obs(i).radius*sin(th), ...
             'r', 'FaceAlpha', 0.5);
    end

    % Draw robot as circle
    th = linspace(0, 2*pi, 30);
    fill(ax_map, state(1) + ROBOT_R*cos(th), state(2) + ROBOT_R*sin(th), ...
         'b', 'FaceAlpha', 0.7);

    % Draw robot heading arrow
    quiver(ax_map, state(1), state(2), ...
           0.4*cos(state(3)), 0.4*sin(state(3)), 'w', 'LineWidth', 2);

    axis(ax_map, 'equal');
    xlim(ax_map, [-0.5, env.width+0.5]);
    ylim(ax_map, [-0.5, env.height+0.5]);
    title(ax_map, sprintf('Warehouse Navigation — PID  |  t = %.1f s  |  Collisions: %d', step*DT, collisions));
    xlabel(ax_map, 'x (m)'); ylabel(ax_map, 'y (m)');

    % Use get_obstacle_distances (removes unused warning)
    dists = get_obstacle_distances(state(1:2)', env);

    drawnow limitrate;
    % Collision check
    if check_collision(state(1:2), env, ROBOT_R)
        collisions = collisions + 1;
    end

    % Goal check
    if norm(state(1:2)' - ref_path(end,:)) < 0.2
        fprintf('Goal reached at t = %.1f s\n', step*DT);
        traj  = traj(1:step+1,:);
        ctrls = ctrls(1:step,:);
        break;
    end
end

%% ========================================================================
% 4b. SIMULATION — Nonlinear MPC Conservative (fmincon)
% =========================================================================

% Reset environment
env.dyn_obs(1).pos = [3.5, 0.5];
env.dyn_obs(1).dir = 1;
env.dyn_obs(1).idx = 1;
env.dyn_obs(2).pos = [6.5, 7.5];
env.dyn_obs(2).dir = 1;
env.dyn_obs(2).idx = 1;

% MPC parameters
N         = 10;       % prediction horizon (config 1)
Q         = 10;       % path tracking weight
R         = 0.1;      % control effort weight
D_SAFE    = 0.3;      % safety margin beyond robot radius

% Reset state
state_mpc  = [0.5; 4.0; 0.0];
path_idx_m = 1;

T_MAX_MPC   = 120.0;
n_steps_mpc = round(T_MAX_MPC / DT);
traj_mpc    = zeros(n_steps_mpc+1, 3);
ctrls_mpc   = zeros(n_steps_mpc, 2);
traj_mpc(1,:) = state_mpc';

collisions_mpc            = 0;
constraint_violations_mpc = 0;
comp_time_mpc             = zeros(n_steps_mpc, 1);

% Setup live figure
fig2 = figure('Position', [100 100 800 600]);
ax2  = axes('Parent', fig2);

for step = 1:n_steps_mpc
    env = update_dynamic_obstacles(env, DT);

    px = state_mpc(1);
    py = state_mpc(2);
    theta = state_mpc(3);

    % Advance path index
    while path_idx_m < size(ref_path,1)
        d = norm(ref_path(path_idx_m,:) - [px, py]);
        if d >= 0.5; break; end
        path_idx_m = path_idx_m + 1;
    end

    % Build reference trajectory over horizon N
    ref_horizon = zeros(N, 2);
    for k = 1:N
        idx = min(path_idx_m + k - 1, size(ref_path,1));
        ref_horizon(k,:) = ref_path(idx,:);
    end

    % Get obstacle distances for constraints
    obs_dists = get_obstacle_distances([px, py], env);

    % Initial guess — use last control input
    if step == 1
        u0 = zeros(2*N, 1);
    else
        u0 = repmat(ctrls_mpc(step-1,:)', N, 1);
    end

    % fmincon options
    opts = optimoptions('fmincon', ...
        'Display', 'none', ...
        'MaxIterations', 50, ...
        'MaxFunctionEvaluations', 2000, ...
        'Algorithm', 'sqp');

    % Input bounds: [v; omega] for each step in horizon
    lb = repmat([-V_MAX; -OMEGA_MAX], N, 1);
    ub = repmat([ V_MAX;  OMEGA_MAX], N, 1);

    % Solve MPC optimisation
    tic;
    u_opt = fmincon(...
        @(u) mpc_cost(u, state_mpc, ref_horizon, N, Q, R, DT), ...
        u0, [], [], [], [], lb, ub, ...
        @(u) mpc_constraints(u, state_mpc, obs_dists, N, DT, ROBOT_R, D_SAFE), ...
        opts);
    comp_time_mpc(step) = toc;

    % Extract first control input
    v_mpc     = u_opt(1);
    omega_mpc = u_opt(2);

    % Count violations before clipping
    if abs(v_mpc) > V_MAX || abs(omega_mpc) > OMEGA_MAX
        constraint_violations_mpc = constraint_violations_mpc + 1;
    end

    % Clip and apply
    v_mpc     = max(-V_MAX, min(V_MAX, v_mpc));
    omega_mpc = max(-OMEGA_MAX, min(OMEGA_MAX, omega_mpc));
    ctrls_mpc(step,:) = [v_mpc, omega_mpc];

    state_mpc = robot_dynamics(state_mpc, [v_mpc; omega_mpc], DT);
    traj_mpc(step+1,:) = state_mpc';

    % Collision check
    if check_collision(state_mpc(1:2), env, ROBOT_R)
        collisions_mpc = collisions_mpc + 1;
    end

    % Live animation
    cla(ax2); hold(ax2, 'on');
    plot(ax2, [0 env.width env.width 0 0], [0 0 env.height env.height 0], 'k-', 'LineWidth', 2);
    for i = 1:size(env.shelves,1)
        rectangle('Parent', ax2, 'Position', env.shelves(i,:), ...
            'FaceColor', [0.6 0.3 0.1], 'EdgeColor', [0.4 0.2 0.05]);
    end
    plot(ax2, ref_path(:,1), ref_path(:,2), 'g--', 'LineWidth', 1);
    plot(ax2, traj_mpc(1:step+1,1), traj_mpc(1:step+1,2), 'r-', 'LineWidth', 1.5);
    for i = 1:length(env.dyn_obs)
        th = linspace(0, 2*pi, 30);
        fill(ax2, env.dyn_obs(i).pos(1) + env.dyn_obs(i).radius*cos(th), ...
             env.dyn_obs(i).pos(2) + env.dyn_obs(i).radius*sin(th), 'r', 'FaceAlpha', 0.5);
    end
    th = linspace(0, 2*pi, 30);
    fill(ax2, state_mpc(1) + ROBOT_R*cos(th), state_mpc(2) + ROBOT_R*sin(th), 'g', 'FaceAlpha', 0.7);
    quiver(ax2, state_mpc(1), state_mpc(2), 0.4*cos(state_mpc(3)), 0.4*sin(state_mpc(3)), 'w', 'LineWidth', 2);
    axis(ax2, 'equal');
    xlim(ax2, [-0.5, env.width+0.5]); ylim(ax2, [-0.5, env.height+0.5]);
    title(ax2, sprintf('Linear MPC N=10  |  t=%.1fs  |  Collisions: %d', step*DT, collisions_mpc));
    xlabel(ax2, 'x (m)'); ylabel(ax2, 'y (m)');
    drawnow limitrate;

    % Goal check
    if norm(state_mpc(1:2)' - ref_path(end,:)) < 0.2
        fprintf('MPC Goal reached at t = %.1f s\n', step*DT);
        traj_mpc  = traj_mpc(1:step+1,:);
        ctrls_mpc = ctrls_mpc(1:step,:);
        comp_time_mpc = comp_time_mpc(1:step);
        break;
    end
end

% MPC Metrics
total_time_mpc   = size(ctrls_mpc,1) * DT;
diffs_mpc        = diff(traj_mpc(:,1:2));
path_length_mpc  = sum(sqrt(sum(diffs_mpc.^2, 2)));
ctrl_effort_mpc  = sum(ctrls_mpc(:,1).^2 + ctrls_mpc(:,2).^2) * DT;
avg_comp_mpc     = mean(comp_time_mpc) * 1000;

fprintf('\n==================================================\n');
fprintf('  Linear MPC (N=10) Results\n');
fprintf('==================================================\n');
fprintf('  Total time:            %.1f s\n', total_time_mpc);
fprintf('  Path length:           %.2f m\n', path_length_mpc);
fprintf('  Collisions:            %d\n', collisions_mpc);
fprintf('  Constraint violations: %d\n', constraint_violations_mpc);
fprintf('  Control effort:        %.2f\n', ctrl_effort_mpc);
fprintf('  Avg compute time:      %.2f ms/step\n', avg_comp_mpc);
fprintf('==================================================\n\n');

%% ========================================================================
% 4c.SIMULATION — Nonlinear MPC Aggressive (fmincon)
% =========================================================================


% Reset environment
env.dyn_obs(1).pos = [3.5, 0.5];
env.dyn_obs(1).dir = 1;
env.dyn_obs(1).idx = 1;
env.dyn_obs(2).pos = [6.5, 7.5];
env.dyn_obs(2).dir = 1;
env.dyn_obs(2).idx = 1;

% MPC parameters — identical to N=10 except N and label
N         = 12;
Q         = 40;
R         = 1;
D_SAFE    = 0.2;

% Reset state
state_mpc  = [0.5; 4.0; 0.0];
path_idx_m = 1;

T_MAX_MPC   = 120.0;
n_steps_mpc = round(T_MAX_MPC / DT);
traj_mpc20    = zeros(n_steps_mpc+1, 3);
ctrls_mpc20   = zeros(n_steps_mpc, 2);
traj_mpc20(1,:) = state_mpc';

collisions_mpc20            = 0;
constraint_violations_mpc20 = 0;
comp_time_mpc20             = zeros(n_steps_mpc, 1);

% Setup live figure
fig3 = figure('Position', [100 100 800 600]);
ax3  = axes('Parent', fig3);

for step = 1:n_steps_mpc
    env = update_dynamic_obstacles(env, DT);

    px    = state_mpc(1);
    py    = state_mpc(2);
    theta = state_mpc(3);

    % Advance path index
    while path_idx_m < size(ref_path,1)
        d = norm(ref_path(path_idx_m,:) - [px, py]);
        if d >= 0.5; break; end
        path_idx_m = path_idx_m + 1;
    end

    % Build reference trajectory over horizon N
    ref_horizon = zeros(N, 2);
    for k = 1:N
        idx = min(path_idx_m + k - 1, size(ref_path,1));
        ref_horizon(k,:) = ref_path(idx,:);
    end

    % Get obstacle distances
    obs_dists = get_obstacle_distances([px, py], env);

    % Force forward initial guess for N=20
    if step == 1
        u0 = zeros(2*N, 1);
        for k = 1:N
            u0(2*k-1) = V_MAX * 0.5;  % always start with forward motion
            u0(2*k)   = 0;
        end
    else
        u0 = zeros(2*N, 1);
        for k = 1:N
            u0(2*k-1) = max(ctrls_mpc20(step-1, 1), V_MAX * 0.4);
            u0(2*k)   = ctrls_mpc20(step-1, 2);
        end
    end

    % fmincon options — same as N=10
    opts = optimoptions('fmincon', ...
        'Display', 'none', ...
        'MaxIterations', 50, ...
        'MaxFunctionEvaluations', 2000, ...
        'Algorithm', 'sqp');

    % Input bounds
    lb = repmat([-V_MAX; -OMEGA_MAX], N, 1);
    ub = repmat([ V_MAX;  OMEGA_MAX], N, 1);

    % Solve
    tic;
    u_opt = fmincon(...
        @(u) mpc_cost(u, state_mpc, ref_horizon, N, Q, R, DT), ...
        u0, [], [], [], [], lb, ub, ...
        @(u) mpc_constraints(u, state_mpc, obs_dists, N, DT, ROBOT_R, D_SAFE), ...
        opts);
    comp_time_mpc20(step) = toc;

    % Extract first control input
    v_mpc     = u_opt(1);
    omega_mpc = u_opt(2);

    % Count violations
    if abs(v_mpc) > V_MAX || abs(omega_mpc) > OMEGA_MAX
        constraint_violations_mpc20 = constraint_violations_mpc20 + 1;
    end

    % Clip and apply
    v_mpc     = max(-V_MAX, min(V_MAX, v_mpc));
    omega_mpc = max(-OMEGA_MAX, min(OMEGA_MAX, omega_mpc));
    ctrls_mpc20(step,:) = [v_mpc, omega_mpc];

    state_mpc = robot_dynamics(state_mpc, [v_mpc; omega_mpc], DT);
    traj_mpc20(step+1,:) = state_mpc';

    % Collision check
    if check_collision(state_mpc(1:2), env, ROBOT_R)
        collisions_mpc20 = collisions_mpc20 + 1;
    end

    % Live animation
    cla(ax3); hold(ax3, 'on');
    plot(ax3, [0 env.width env.width 0 0], [0 0 env.height env.height 0], 'k-', 'LineWidth', 2);
    for i = 1:size(env.shelves,1)
        rectangle('Parent', ax3, 'Position', env.shelves(i,:), ...
            'FaceColor', [0.6 0.3 0.1], 'EdgeColor', [0.4 0.2 0.05]);
    end
    plot(ax3, ref_path(:,1), ref_path(:,2), 'g--', 'LineWidth', 1);
    plot(ax3, traj_mpc20(1:step+1,1), traj_mpc20(1:step+1,2), 'r-', 'LineWidth', 1.5);
    for i = 1:length(env.dyn_obs)
        th = linspace(0, 2*pi, 30);
        fill(ax3, env.dyn_obs(i).pos(1) + env.dyn_obs(i).radius*cos(th), ...
             env.dyn_obs(i).pos(2) + env.dyn_obs(i).radius*sin(th), 'r', 'FaceAlpha', 0.5);
    end
    th = linspace(0, 2*pi, 30);
    fill(ax3, state_mpc(1) + ROBOT_R*cos(th), state_mpc(2) + ROBOT_R*sin(th), 'g', 'FaceAlpha', 0.7);
    quiver(ax3, state_mpc(1), state_mpc(2), 0.4*cos(state_mpc(3)), 0.4*sin(state_mpc(3)), 'w', 'LineWidth', 2);
    axis(ax3, 'equal');
    xlim(ax3, [-0.5, env.width+0.5]); ylim(ax3, [-0.5, env.height+0.5]);
    title(ax3, sprintf('NL-MPC Aggressive (N=12)  |  t=%.1fs  |  Collisions: %d', ...
          step*DT, collisions_mpc20));
    xlabel(ax3, 'x (m)'); ylabel(ax3, 'y (m)');
    drawnow limitrate;
    % Goal check
    if norm(state_mpc(1:2)' - ref_path(end,:)) < 0.2
        fprintf('MPC N=20 Goal reached at t = %.1f s\n', step*DT);
        traj_mpc20    = traj_mpc20(1:step+1,:);
        ctrls_mpc20   = ctrls_mpc20(1:step,:);
        comp_time_mpc20 = comp_time_mpc20(1:step);
        break;
    end
end

total_time_mpc20_val  = size(ctrls_mpc20,1) * DT;
diffs_mpc20           = diff(traj_mpc20(:,1:2));
path_length_mpc20_val = sum(sqrt(sum(diffs_mpc20.^2, 2)));
ctrl_effort_mpc20_val = sum(ctrls_mpc20(:,1).^2 + ctrls_mpc20(:,2).^2) * DT;
avg_comp_mpc20_val    = mean(comp_time_mpc20) * 1000;

fprintf('\n==================================================\n');
fprintf('  NL-MPC Aggressive Results\n');
fprintf('==================================================\n');
fprintf('  Total time:            %.1f s\n', total_time_mpc20_val);
fprintf('  Path length:           %.2f m\n', path_length_mpc20_val);
fprintf('  Collisions:            %d\n', collisions_mpc20);
fprintf('  Constraint violations: %d\n', constraint_violations_mpc20);
fprintf('  Control effort:        %.2f\n', ctrl_effort_mpc20_val);
fprintf('  Avg compute time:      %.2f ms/step\n', avg_comp_mpc20_val);
fprintf('==================================================\n\n');

% Aliases for visualisation
traj_na         = traj_mpc20;
ctrls_na        = ctrls_mpc20;
collisions_na   = collisions_mpc20;
total_time_na   = total_time_mpc20_val;

%% ========================================================================
% 4d. SIMULATION — Linear MPC N=10 (Successive Linearisation)
% =========================================================================

% Reset environment
env.dyn_obs(1).pos = [3.5, 0.5];
env.dyn_obs(1).dir = 1;
env.dyn_obs(1).idx = 1;
env.dyn_obs(2).pos = [6.5, 7.5];
env.dyn_obs(2).dir = 1;
env.dyn_obs(2).idx = 1;

% Parameters
N_l10      = 10;   % prediction horizon
Nc_l10     = 1;    % control horizon (shorter = less aggressive)
Q_l10      = 50;
R_l10      = 0.5;
D_SAFE_l10 = 0.1;  % softened safety margin

% Reset
robot_state_l10   = [0.5; 4.0; 0.0];
path_idx_l10      = 1;
T_MAX_L10         = 120.0;
n_steps_l10       = round(T_MAX_L10 / DT);
traj_l10          = zeros(n_steps_l10+1, 3);
ctrls_l10         = zeros(n_steps_l10, 2);
traj_l10(1,:)     = robot_state_l10';
collisions_l10            = 0;
constraint_violations_l10 = 0;
comp_time_l10             = zeros(n_steps_l10, 1);

fig4 = figure('Position', [100 100 800 600]);
ax4  = axes('Parent', fig4);

for step = 1:n_steps_l10
    env = update_dynamic_obstacles(env, DT);
    px    = robot_state_l10(1);
    py    = robot_state_l10(2);
    theta = robot_state_l10(3);

    % Advance path index
    while path_idx_l10 < size(ref_path,1)
        d = norm(ref_path(path_idx_l10,:) - [px, py]);
        if d >= 0.5; break; end
        path_idx_l10 = path_idx_l10 + 1;
    end

    % Reference horizon
    ref_horizon = zeros(N_l10, 2);
    for k = 1:N_l10
        idx = min(path_idx_l10 + k - 1, size(ref_path,1));
        ref_horizon(k,:) = ref_path(idx,:);
    end

    % Nominal velocity
    if step == 1
        v_nom = 0.4;
    else
        v_nom = max(abs(ctrls_l10(step-1,1)), 0.3);
    end

    % =============================================
    % SUCCESSIVE LINEARISATION
    % Re-linearise at each horizon step
    % =============================================
    nx = 3; nu = 2;
    x_pred = robot_state_l10;

    A_seq = zeros(nx, nx, N_l10);
    B_seq = zeros(nx, nu, N_l10);

    for k = 1:N_l10
        theta_k = x_pred(3);

        % Linearise around predicted heading
        A_seq(:,:,k) = [1, 0, -v_nom*sin(theta_k)*DT;
                        0, 1,  v_nom*cos(theta_k)*DT;
                        0, 0,  1];
        B_seq(:,:,k) = [cos(theta_k)*DT, 0;
                        sin(theta_k)*DT, 0;
                        0,               DT];

        % Propagate predicted state
        if step == 1
            u_pred = [v_nom; 0];
        else
            u_pred = ctrls_l10(step-1,:)';
        end
        x_pred = A_seq(:,:,k) * x_pred + B_seq(:,:,k) * u_pred;
    end

    % Build Phi and Gamma
    Phi   = zeros(nx*N_l10, nx);
    Gamma = zeros(nx*N_l10, nu*N_l10);

    Phi_k = eye(nx);
    for k = 1:N_l10
        Phi_k = A_seq(:,:,k) * Phi_k;
        Phi((k-1)*nx+1:k*nx, :) = Phi_k;
    end

    for col = 1:N_l10
        A_prod = eye(nx);
        for row = col:N_l10
            if row > col
                A_prod = A_seq(:,:,row) * A_prod;
            end
            Gamma((row-1)*nx+1:row*nx, (col-1)*nu+1:col*nu) = ...
                A_prod * B_seq(:,:,col);
        end
    end

    % Control horizon constraint — fix inputs after Nc steps
    % Build selection matrix to map Nc inputs to N inputs
    S = zeros(nu*N_l10, nu*Nc_l10);
    for k = 1:N_l10
        col_idx = min(k, Nc_l10);
        S((k-1)*nu+1:k*nu, (col_idx-1)*nu+1:col_idx*nu) = eye(nu);
    end
    Gamma_c = Gamma * S;

    % Cost matrices
    Q_blk = zeros(nx*N_l10);
    for k = 1:N_l10
        Q_blk((k-1)*nx+1,(k-1)*nx+1) = Q_l10;
        Q_blk((k-1)*nx+2,(k-1)*nx+2) = Q_l10;
    end
    Q_blk(end-2,end-2) = Q_l10 * 5;
    Q_blk(end-1,end-1) = Q_l10 * 5;

    R_blk = R_l10 * eye(nu*Nc_l10);

    % Reference vector
    X_ref = zeros(nx*N_l10, 1);
    for k = 1:N_l10
        X_ref((k-1)*nx+1) = ref_horizon(k,1);
        X_ref((k-1)*nx+2) = ref_horizon(k,2);
    end

    % QP cost
    H     = Gamma_c' * Q_blk * Gamma_c + R_blk;
    H     = (H + H') / 2;
    f_vec = Gamma_c' * Q_blk * (Phi * robot_state_l10 - X_ref);

    % Input bounds for control horizon
    lb_u = repmat([-V_MAX; -OMEGA_MAX], Nc_l10, 1);
    ub_u = repmat([ V_MAX;  OMEGA_MAX], Nc_l10, 1);

    % Obstacle constraints
    obs_dists = get_obstacle_distances([px, py], env);
    A_obs = [];
    b_obs = [];
    for i = 1:length(obs_dists)
        obs = obs_dists{i};
        if strcmp(obs.type, 'wall'); continue; end
        if obs.dist < 1.5
            if strcmp(obs.type, 'dynamic')
                obs_px = obs.data.pos(1);
                obs_py = obs.data.pos(2);
            else
                shelf  = obs.data;
                obs_px = max(shelf(1), min(px, shelf(1)+shelf(3)));
                obs_py = max(shelf(2), min(py, shelf(2)+shelf(4)));
            end
            diff_vec = [px-obs_px; py-obs_py];
            if norm(diff_vec) < 1e-6; continue; end
            n_vec    = diff_vec / norm(diff_vec);
            a_row    = zeros(1, nu*Nc_l10);
            a_row(1) = -(n_vec(1)*cos(theta) + ...
                         n_vec(2)*sin(theta))*DT;
            A_obs(end+1,:) = a_row;
            % Softened constraint
            b_obs(end+1) = -(ROBOT_R + D_SAFE_l10 - obs.dist) + 0.1;
        end
    end

    % Solve QP
    qp_opts = optimoptions('quadprog', 'Display', 'none', ...
                           'MaxIterations', 200);
    tic;
    if isempty(A_obs)
        u_opt = quadprog(H, f_vec, [], [], [], [], ...
                         lb_u, ub_u, [], qp_opts);
    else
        u_opt = quadprog(H, f_vec, A_obs, b_obs, [], [], ...
                         lb_u, ub_u, [], qp_opts);
    end
    comp_time_l10(step) = toc;

    % Extract first control from control horizon
    if isempty(u_opt)
        % FIX 2: proper angle wrapping for fallback
        des_th = atan2(ref_horizon(1,2)-py, ref_horizon(1,1)-px);
        h_err  = atan2(sin(des_th-theta), cos(des_th-theta));
        v_cmd     = 0.4;
        omega_cmd = max(-OMEGA_MAX, min(OMEGA_MAX, 3.0*h_err));
    else
        v_cmd     = u_opt(1);
        omega_cmd = u_opt(2);
    end

    % FIX 2: proper angle wrapping
    des_th    = atan2(ref_horizon(1,2)-py, ref_horizon(1,1)-px);
    h_err_now = atan2(sin(des_th-theta), cos(des_th-theta));

    % Large heading error — turn in place
    if abs(h_err_now) > 0.6
        v_cmd     = 0.0;
        omega_cmd = max(-OMEGA_MAX, min(OMEGA_MAX, 3.0*h_err_now));

    % Low speed — push forward
    elseif abs(v_cmd) < 0.2
        v_cmd     = 0.4;
        omega_cmd = max(-OMEGA_MAX, min(OMEGA_MAX, 3.0*h_err_now));
    end

    % Count violations
    if abs(v_cmd) > V_MAX || abs(omega_cmd) > OMEGA_MAX
        constraint_violations_l10 = constraint_violations_l10 + 1;
    end

    % Clip
    v_cmd     = max(-V_MAX, min(V_MAX, v_cmd));
    omega_cmd = max(-OMEGA_MAX, min(OMEGA_MAX, omega_cmd));
    ctrls_l10(step,:) = [v_cmd, omega_cmd];

    % Propagate
    robot_state_l10    = robot_dynamics(robot_state_l10, ...
                                        [v_cmd; omega_cmd], DT);
    traj_l10(step+1,:) = robot_state_l10';

    % Collision check
    if check_collision(robot_state_l10(1:2), env, ROBOT_R)
        collisions_l10 = collisions_l10 + 1;
    end

    % Animation
    cla(ax4); hold(ax4, 'on');
    plot(ax4, [0 env.width env.width 0 0], ...
         [0 0 env.height env.height 0], 'k-', 'LineWidth', 2);
    for i = 1:size(env.shelves,1)
        rectangle('Parent', ax4, 'Position', env.shelves(i,:), ...
            'FaceColor', [0.6 0.3 0.1], 'EdgeColor', [0.4 0.2 0.05]);
    end
    plot(ax4, ref_path(:,1), ref_path(:,2), 'g--', 'LineWidth', 1);
    plot(ax4, traj_l10(1:step+1,1), traj_l10(1:step+1,2), ...
         'b-', 'LineWidth', 1.5);
    for i = 1:length(env.dyn_obs)
        th = linspace(0, 2*pi, 30);
        fill(ax4, env.dyn_obs(i).pos(1) + env.dyn_obs(i).radius*cos(th), ...
             env.dyn_obs(i).pos(2) + env.dyn_obs(i).radius*sin(th), ...
             'r', 'FaceAlpha', 0.5);
    end
    th = linspace(0, 2*pi, 30);
    fill(ax4, robot_state_l10(1) + ROBOT_R*cos(th), ...
         robot_state_l10(2) + ROBOT_R*sin(th), 'b', 'FaceAlpha', 0.7);
    quiver(ax4, robot_state_l10(1), robot_state_l10(2), ...
           0.4*cos(robot_state_l10(3)), ...
           0.4*sin(robot_state_l10(3)), 'w', 'LineWidth', 2);
    axis(ax4, 'equal');
    xlim(ax4, [-0.5, env.width+0.5]);
    ylim(ax4, [-0.5, env.height+0.5]);
    title(ax4, sprintf('Linear MPC SL N=10  |  t=%.1fs  |  Collisions: %d', ...
          step*DT, collisions_l10));
    xlabel(ax4, 'x (m)'); ylabel(ax4, 'y (m)');
    drawnow limitrate;

    % Goal check
    if norm(robot_state_l10(1:2)' - ref_path(end,:)) < 0.3
        fprintf('Linear MPC N=10 Goal reached at t = %.1f s\n', step*DT);
        traj_l10      = traj_l10(1:step+1,:);
        ctrls_l10     = ctrls_l10(1:step,:);
        comp_time_l10 = comp_time_l10(1:step);
        break;
    end
end

% Metrics
total_time_l10  = size(ctrls_l10,1) * DT;
diffs_l10       = diff(traj_l10(:,1:2));
path_len_l10    = sum(sqrt(sum(diffs_l10.^2,2)));
effort_l10      = sum(ctrls_l10(:,1).^2 + ctrls_l10(:,2).^2)*DT;
avg_comp_l10    = mean(comp_time_l10)*1000;

fprintf('\n==================================================\n');
fprintf('  Linear MPC N=10 - Successive Linearisation\n');
fprintf('==================================================\n');
fprintf('  Total time:            %.1f s\n', total_time_l10);
fprintf('  Path length:           %.2f m\n', path_len_l10);
fprintf('  Collisions:            %d\n', collisions_l10);
fprintf('  Constraint violations: %d\n', constraint_violations_l10);
fprintf('  Control effort:        %.2f\n', effort_l10);
fprintf('  Avg compute time:      %.2f ms/step\n', avg_comp_l10);
fprintf('==================================================\n\n');


%% ========================================================================
% 5. METRICS
% =========================================================================
total_time  = size(ctrls,1) * DT;
diffs       = diff(traj(:,1:2));
path_length = sum(sqrt(sum(diffs.^2, 2)));
ctrl_effort = sum(ctrls(:,1).^2 + ctrls(:,2).^2) * DT;


fprintf('\n==================================================\n');
fprintf('  PID Controller Results\n');
fprintf('==================================================\n');
fprintf('  Total time:            %.1f s\n', total_time);
fprintf('  Path length:           %.2f m\n', path_length);
fprintf('  Collisions:            %d\n', collisions);
fprintf('  Constraint violations: %d\n', constraint_violations);
fprintf('  Control effort:        %.2f\n', ctrl_effort);
fprintf('==================================================\n\n');

% Aliases for visualisation
traj_nc         = traj_mpc;
ctrls_nc        = ctrls_mpc;
collisions_nc   = collisions_mpc;
total_time_nc   = total_time_mpc;


%% ========================================================================
% 6. VISUALISATION — All Controllers Comparison
% =========================================================================
figure('Position', [50 50 1920 1080]);

colors = {'b', 'r', 'm', 'c'};
names  = {'PID', 'NL-MPC Conservative', 'NL-MPC Aggressive', 'Linear MPC SL'};

trajs   = {traj,      traj_nc,      traj_na,      traj_l10};
ctrlss  = {ctrls,     ctrls_nc,     ctrls_na,     ctrls_l10};
cols_n  = {collisions, collisions_nc, collisions_na, collisions_l10};
times_n = {size(ctrls,1)*DT, total_time_nc, total_time_na, total_time_l10};

% --- Row 1: Trajectory maps ---
for c = 1:4
    subplot(2,4,c); hold on;
    xlim([-0.5, env.width+0.5]);
    ylim([-0.5, env.height+0.5]);
    axis equal;
    title(names{c}, 'FontSize', 9, 'FontWeight', 'bold');
    xlabel('x (m)'); ylabel('y (m)');

    plot([0 env.width env.width 0 0], ...
         [0 0 env.height env.height 0], 'k-', 'LineWidth', 2);

    for i = 1:size(env.shelves,1)
        rectangle('Position', env.shelves(i,:), ...
            'FaceColor', [0.6 0.3 0.1], 'EdgeColor', [0.4 0.2 0.05]);
    end

    plot(ref_path(:,1), ref_path(:,2), 'g--', 'LineWidth', 1);
    plot(trajs{c}(:,1), trajs{c}(:,2), 'Color', colors{c}, 'LineWidth', 2);
    plot(trajs{c}(1,1), trajs{c}(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(trajs{c}(end,1), trajs{c}(end,2), 'r*', 'MarkerSize', 10);

    for i = 1:length(env.dyn_obs)
        th = linspace(0, 2*pi, 30);
        fill(env.dyn_obs(i).pos(1) + env.dyn_obs(i).radius*cos(th), ...
             env.dyn_obs(i).pos(2) + env.dyn_obs(i).radius*sin(th), ...
             'r', 'FaceAlpha', 0.4);
    end

    text(0.2, 7.6, sprintf('Collisions: %d', cols_n{c}), ...
         'FontSize', 8, 'Color', 'r', 'FontWeight', 'bold');
    text(0.2, 7.0, sprintf('Time: %.1fs', times_n{c}), ...
         'FontSize', 8, 'Color', 'b');
end

% --- Row 2: Control inputs ---
for c = 1:4
    subplot(2,4,4+c); hold on;
    t_vec = (0:size(ctrlss{c},1)-1) * DT;
    plot(t_vec, ctrlss{c}(:,1), 'b-', 'LineWidth', 1.2, 'DisplayName', 'v (m/s)');
    plot(t_vec, ctrlss{c}(:,2), 'r-', 'LineWidth', 1.2, 'DisplayName', '\omega (rad/s)');
    yline( V_MAX,    'b--', 'Alpha', 0.4, 'HandleVisibility', 'off');
    yline(-V_MAX,    'b--', 'Alpha', 0.4, 'HandleVisibility', 'off');
    yline( OMEGA_MAX, 'r--', 'Alpha', 0.4, 'HandleVisibility', 'off');
    yline(-OMEGA_MAX, 'r--', 'Alpha', 0.4, 'HandleVisibility', 'off');
    xlabel('Time (s)'); ylabel('Control input');
    title(sprintf('%s — Inputs', names{c}), 'FontSize', 9);
    legend('FontSize', 7, 'Location', 'northeast');
    grid on;
    ylim([-1.8, 1.8]);
end

sgtitle('Warehouse Navigation — Controller Comparison', ...
        'FontSize', 12, 'FontWeight', 'bold');

exportgraphics(gcf, 'topic1_all_controllers.png', 'Resolution', 300);
fprintf('Comparison plot saved to topic1_all_controllers.png\n');


%% ========================================================================
% HELPER FUNCTIONS
% =========================================================================

function next_state = robot_dynamics(state, u, dt)
    % Discrete-time kinematic model of a differential-drive robot.
    % state = [px; py; theta], u = [v; omega]
    px = state(1); py = state(2); theta = state(3);
    v = u(1); omega = u(2);

    px_new    = px + v * cos(theta) * dt;
    py_new    = py + v * sin(theta) * dt;
    theta_new = theta + omega * dt;
    theta_new = mod(theta_new + pi, 2*pi) - pi;  % wrap to [-pi,pi]

    next_state = [px_new; py_new; theta_new];
end

function env = update_dynamic_obstacles(env, dt)
    % Move each dynamic obstacle along its path (patrol back and forth).
    for i = 1:length(env.dyn_obs)
        obs = env.dyn_obs(i);
        next_idx = obs.idx + obs.dir;
        if next_idx >= 1 && next_idx <= size(obs.path,1)
            target = obs.path(next_idx,:);
        else
            target = obs.path(obs.idx,:);
        end

        dir_vec = target - obs.pos;
        d = norm(dir_vec);

        if d < 0.1
            % Reached waypoint, reverse direction
            obs.dir = -obs.dir;
            new_idx = obs.idx + obs.dir;
            if new_idx >= 1 && new_idx <= size(obs.path,1)
                obs.idx = new_idx;
            end
        else
            obs.pos = obs.pos + (dir_vec / d) * obs.speed * dt;
        end
        env.dyn_obs(i) = obs;
    end
end

function collided = check_collision(robot_pos, env, robot_r)
    % Check if the robot collides with any obstacle or wall.
    px = robot_pos(1); py = robot_pos(2);
    collided = false;

    % Wall collision
    if px - robot_r < 0 || px + robot_r > env.width || ...
       py - robot_r < 0 || py + robot_r > env.height
        collided = true; return;
    end

    % Shelf collision (rectangle vs circle)
    for i = 1:size(env.shelves,1)
        sx = env.shelves(i,1); sy = env.shelves(i,2);
        sw = env.shelves(i,3); sh = env.shelves(i,4);
        closest_x = max(sx, min(px, sx + sw));
        closest_y = max(sy, min(py, sy + sh));
        d = sqrt((px - closest_x)^2 + (py - closest_y)^2);
        if d < robot_r
            collided = true; return;
        end
    end

    % Dynamic obstacle collision (circle vs circle)
    for i = 1:length(env.dyn_obs)
        d = norm(robot_pos' - env.dyn_obs(i).pos);
        if d < robot_r + env.dyn_obs(i).radius
            collided = true; return;
        end
    end
end

function distances = get_obstacle_distances(robot_pos, env)
    % Compute distance from the robot to every obstacle.
    % Returns: Nx3 cell array {distance, type_string, data}
    % Useful for MPC constraint formulation.
    px = robot_pos(1); py = robot_pos(2);
    distances = {};

    % Shelves
    for i = 1:size(env.shelves,1)
        sx = env.shelves(i,1); sy = env.shelves(i,2);
        sw = env.shelves(i,3); sh = env.shelves(i,4);
        cx = max(sx, min(px, sx + sw));
        cy = max(sy, min(py, sy + sh));
        d = sqrt((px-cx)^2 + (py-cy)^2);
        distances{end+1} = struct('dist', d, 'type', 'shelf', 'data', env.shelves(i,:));
    end

    % Dynamic obstacles
    for i = 1:length(env.dyn_obs)
        d = norm(robot_pos' - env.dyn_obs(i).pos) - env.dyn_obs(i).radius;
        distances{end+1} = struct('dist', d, 'type', 'dynamic', 'data', env.dyn_obs(i));
    end

    % Walls
    distances{end+1} = struct('dist', px, 'type', 'wall', 'data', 'left');
    distances{end+1} = struct('dist', env.width - px, 'type', 'wall', 'data', 'right');
    distances{end+1} = struct('dist', py, 'type', 'wall', 'data', 'bottom');
    distances{end+1} = struct('dist', env.height - py, 'type', 'wall', 'data', 'top');
end


function cost = mpc_cost(u, state, ref_horizon, N, Q, R, dt)
    % MPC cost: path tracking + control effort
    cost = 0;
    x = state;
    for k = 1:N
        v     = u(2*k-1);
        omega = u(2*k);
        % Propagate
        x = robot_dynamics(x, [v; omega], dt);
        % Tracking error to reference point
        err = x(1:2) - ref_horizon(k,:)';
        cost = cost + Q * (err' * err) + R * (v^2 + omega^2);
    end
end

function [c, ceq] = mpc_constraints(u, state, obs_dists, N, dt, robot_r, d_safe)
    ceq = [];
    c   = [];
    
    % Only use obstacles that are actually reachable — dist < 2.0m
    relevant = {};
    for i = 1:length(obs_dists)
        if obs_dists{i}.dist < 1.0
            relevant{end+1} = obs_dists{i};
        end
    end
    
    % Cap at 4 closest obstacles
    n_obs = min(4, length(relevant));
    if n_obs == 0; return; end
    
    x = state;
    for k = 1:N
        v     = u(2*k-1);
        omega = u(2*k);
        x     = robot_dynamics(x, [v; omega], dt);
        for i = 1:n_obs
            obs = relevant{i};
            if strcmp(obs.type, 'dynamic')
            obs_pos = obs.data.pos;
            d = norm(x(1:2)' - obs_pos) - obs.data.radius;
            c(end+1) = (robot_r + d_safe) - d;   % ADD THIS LINE
        
            elseif strcmp(obs.type, 'shelf')
                shelf = obs.data;
                cx = max(shelf(1), min(x(1), shelf(1)+shelf(3)));
                cy = max(shelf(2), min(x(2), shelf(2)+shelf(4)));
                d = norm([x(1)-cx, x(2)-cy]);
                c(end+1) = (robot_r + d_safe) - d - 0.05;
            end
        end
    end
end