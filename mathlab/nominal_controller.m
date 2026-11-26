function u_star = nominal_controller(Kp,Kd,p, dp, p_d,dp_d, ddp_d)
% NOMINAL CONTROLLER compute desired accelleration u*
% Kp,Kd > 0

    e_p = p_d - p;
    e_dp = dp_d - dp;

    u_star = ddp_d + Kp * e_p + Kd * e_dp;
end
