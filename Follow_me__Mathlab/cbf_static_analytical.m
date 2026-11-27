function [u_safe, h_val] = cbf_static_analytical(p, dp, u_star, p_obs, params)
% CBF_STATIC_ANALYTICAL closed law for static obstacles (point obstacle)
% p, dp, u_star, p_obs : column vectors 3x1 in NED
% params               : struct with mu, r_drone, d_margin, delta1_offset

% safety distance delta for obstacle
d_min  = params.r_drone + params.d_margin;
delta  = d_min^2;
delta1 = delta + params.delta1_offset;   % delta1 > delta

% distance vector from obstacel -> quadrotor and function h
diff  = p - p_obs;
dist2 = diff' * diff;
h_val = dist2 + params.mu * (diff' * dp);

% belonging condition to D_track
in_Du_star  = (diff' * (params.mu * u_star + 2 * dp)) > 0;
in_D_delta1 = h_val > delta1;

if in_Du_star || in_D_delta1
    % nominal tracking
    u_safe = u_star;
else
    % safety filter: radial braking + tangential sliding
    Pi_parallel = (diff * diff') / dist2;
    Pi_perp     = eye(3) - Pi_parallel;

    u_safe = -(2 / params.mu) * Pi_parallel * dp + Pi_perp * u_star;
end
end