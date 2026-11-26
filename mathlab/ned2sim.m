function [T_sim, tau_sim] = ned2sim(T_ned, tau_ned)
% NED2SIM conversion (roll, pitch, yaw) from NED to CoppeliaSim EDU

Pb = [1  0  0;
    0 -1  0;
    0  0 -1];

% La magnitudine della spinta T è invariata (in CoppeliaSim agisce lungo +z_b)
T_sim   = T_ned;

% torque around pitch and yaw change sign
tau_sim = Pb' * tau_ned;
end