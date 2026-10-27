function xd_dot = ref_model(xd, psi_ref)
% 3rd-order reference model for heading, Fossen (2021) eq. (15.143)
%   psi_d/psi_ref(s) = w^3 / ((s + w)(s^2 + 2*zeta*w*s + w^2))
%
%   xd     = [psi_d; r_d; r_d_dot]   (rad, rad/s, rad/s^2)
%   psi_ref = heading setpoint       (rad)

    w_ref    = 0.03;    % reference model natural frequency (rad/s)
    zeta_ref = 1;       % relative damping ratio

    psi_d = xd(1);
    r_d   = xd(2);
    a_d   = xd(3);      % desired yaw acceleration

    xd_dot = [ r_d
               a_d
               w_ref^3*(psi_ref - psi_d) - (2*zeta_ref + 1)*w_ref^2*r_d ...
                                         - (2*zeta_ref + 1)*w_ref*a_d ];
end