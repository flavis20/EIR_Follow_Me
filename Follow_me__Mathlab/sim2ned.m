function [p_ned, dp_ned, R_ned, Omega_ned] = sim2ned(p_sim, dp_sim, R_sim, Omega_sim)
% SIM2NED conversion from ENU to NED
% all vectors in input and output are 3x1 column.

% Orthogonal transformation matrix with det = +1
Pw = [0  1  0;
    1  0  0;
    0  0 -1];   % ENU -> NED (z down)

Pb = [1  0  0;
    0 -1  0;
    0  0 -1];   % Body: x forward, y right, z down

p_ned     = Pw * p_sim;
dp_ned    = Pw * dp_sim;
R_ned     = Pw * R_sim * Pb';
Omega_ned = Pb * (R_sim' * Omega_sim);
end