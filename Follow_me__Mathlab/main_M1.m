clear; clc; close all;
% coppelia path 
addpath('C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/programming/zmqRemoteApi/clients/matlab');

% API connection between coppeliasim and matlab
client = RemoteAPIClient();
sim = client.require('sim');
client.setStepping(true);

% Scene objects
drone_h = sim.getObject('/Quadcopter');
bill_h  = sim.getObject('/Bill');
obs_handles = [
    sim.getObject('/Tree1');
    sim.getObject('/Tree2');
    sim.getObject('/Bench')
    ];
num_obs = length(obs_handles);

% Vehicle's parameters
quad.m      = 4.34;
quad.g      = 9.81;
quad.J      = diag([0.0820, 0.0845, 0.1377]);
quad.T_max  = 2 * quad.m * quad.g;  % Thrust-to-weight ratio = 2
quad.kR     = 3.5;
quad.kOmega = 1.1;

Kp = 1.5;
Kd = 2.6;

% distance between bill and quadrotors (NED Convention)
dist_behind = 2.5;
height_des  = 1.6;

% CBF parameters
cbf_params.mu            = 0.4; % for Cristofaro theorem it should be <0.46
cbf_params.r_drone       = 0.45;   
cbf_params.d_margin      = 0.25;   % safety margin [m] 
cbf_params.delta1_offset = 0.30;

%starting simulation
sim.startSimulation();
dt = sim.getSimulationTimeStep();
N  = round(45.0 / dt);

err_follow_hist = zeros(1, N);
min_dist_hist   = zeros(1, N);
t_hist          = (0:N-1) * dt;

p_bill_init = cell2mat(sim.getObjectPosition(bill_h, -1))';
[p_target_prev, ~, ~, ~] = sim2ned(p_bill_init, zeros(3,1), eye(3), zeros(3,1));
dp_target_filt = zeros(3, 1);


for k = 1:N
    % Position and velocity of the quadrator + conversion 
    p_drone_sim  = cell2mat(sim.getObjectPosition(drone_h, -1))';
    [dp_drone_sim, omega_sim] = sim.getObjectVelocity(drone_h);
    dp_drone_sim = cell2mat(dp_drone_sim)';
    omega_sim    = cell2mat(omega_sim)';
    
    % quadrator rotation matrix for inclination
    M_drone      = reshape(cell2mat(sim.getObjectMatrix(drone_h, -1)), 4, 3)';
    R_sim        = M_drone(1:3, 1:3);

    % Position and velocity of the bill + conversion
    p_bill_sim   = cell2mat(sim.getObjectPosition(bill_h, -1))';
    eul_bill_sim = cell2mat(sim.getObjectOrientation(bill_h, -1));

    % NED conversion required
    [p, dp, R, Omega] = sim2ned(p_drone_sim, dp_drone_sim, R_sim, omega_sim);
    [p_target, ~, ~, ~] = sim2ned(p_bill_sim, zeros(3,1), eye(3), zeros(3,1));
    dp_target_raw  = (p_target - p_target_prev) / dt;
    dp_target_filt = 0.85 * dp_target_filt + 0.15 * dp_target_raw;
    p_target_prev  = p_target;

    yaw_bill_sim = eul_bill_sim(3);
    dir_bill_ned = [sin(yaw_bill_sim); cos(yaw_bill_sim); 0];
    % Obstacles position and NED conversion 
    P_obs_ned = zeros(3, num_obs);
    dists_3d  = zeros(1, num_obs);
    for i = 1:num_obs
        p_obs_sim       = cell2mat(sim.getObjectPosition(obs_handles(i), -1))';
        [p_obs_i, ~, ~, ~] = sim2ned(p_obs_sim, zeros(3,1), eye(3), zeros(3,1));
        p_obs_i(3) = p(3); % put tree at right height
        P_obs_ned(:, i) = p_obs_i;
        dists_3d(i)     = norm(p - p_obs_i);
    end

    % Desired Viewpoint and yaw in bill direction
    p_d   = p_target - dist_behind * dir_bill_ned + [0; 0; -height_des];
    dp_d  = dp_target_filt;
    ddp_d = [0; 0; 0];

    % target positon wrt quadrotor 
    vec_target = p_target - p;
    psi_d = atan2(vec_target(2), vec_target(1));

    % nominal controller
    u_star = nominal_controller(Kp, Kd, p, dp, p_d, dp_d, ddp_d);

    % nearest obstacle
    [d_closest, idx_closest] = min(dists_3d);
    p_obs_closest = P_obs_ned(:, idx_closest);

    % recall CBF function
    [u_safe, ~] = cbf_static_analytical(p, dp, u_star, p_obs_closest, cbf_params);
    %u_safe=u_star;
    % SO(3) controller
    [T_ned, tau_ned]   = so3_attitude_controller(R, Omega, u_safe, psi_d, quad);
    [T_sim, tau_b_sim] = ned2sim(T_ned, tau_ned);

    F_world_sim   = R_sim * [0; 0; T_sim];
    tau_world_sim = R_sim * tau_b_sim;

    sim.addForceAndTorque(drone_h, F_world_sim', tau_world_sim');
    client.step();

    % saving metrics
    err_follow_hist(k) = norm(p - p_d);
    min_dist_hist(k)   = d_closest - cbf_params.r_drone;
end

sim.stopSimulation();
figure('Name', 'Milestone M1 - Risultati');
subplot(2,1,1);
plot(t_hist, err_follow_hist, 'b-', 'LineWidth', 1.5); grid on;
ylabel('Following Error [m]'); title('Errore di inseguimento');

subplot(2,1,2);
plot(t_hist, min_dist_hist, 'r-', 'LineWidth', 1.5); hold on;
yline(0, 'k--', 'Collisione'); grid on;
xlabel('Tempo [s]'); ylabel('Distanza min [m]'); title('Distanza minima dagli ostacoli');

