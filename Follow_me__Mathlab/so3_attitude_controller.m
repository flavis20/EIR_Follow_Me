function [T, tau] = so3_attitude_controller(R, Omega, u_safe, psi_d, quad_params)
% SO3_ATTITUDE_CONTROLLER Geometric tracking controller on SO(3)
% R           : rotation matrix 3x3 body -> NED
% Omega       : angular velocity 3x1 in frame body NED [rad/s]
% u_safe      : desired accelleration 3x1 in NED [m/s^2]
% psi_d       : desired yaw [rad]
% quad_params : struct with m, g, J, T_max, kR, kOmega

e3 = [0; 0; 1];
m  = quad_params.m;
g  = quad_params.g;
J  = quad_params.J;

% desired forces vector in NED (m*ddp = m*g*e3 - T*R*e3)
F_d = m * (g * e3 - u_safe);

% Thrust projected along the drone's current vertical axis (R*e3)
T = dot(F_d, R * e3);
T = max(0, min(quad_params.T_max, T));   % clamping saturation [0, 2*m*g]

% desired rotation matrix R_d = [b1d, b2d, b3d]
if norm(F_d) > 1e-6
    b3d = F_d / norm(F_d);
else
    b3d = e3;
end

b1c = [cos(psi_d); sin(psi_d); 0];
b2d = cross(b3d, b1c);
b2d = b2d / norm(b2d);
b1d = cross(b2d, b3d);

R_d = [b1d, b2d, b3d];

% Trim error on SO(3) via vee map
E_R = 0.5 * (R_d' * R - R' * R_d);
e_R = [E_R(3, 2); E_R(1, 3); E_R(2, 1)];

% angular velocity error
e_Omega = Omega;

% control couple with gyroscopic cancellation
tau = -quad_params.kR * e_R - quad_params.kOmega * e_Omega + cross(Omega, J * Omega);
end