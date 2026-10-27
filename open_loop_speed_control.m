function n_c = open_loop_speed_control(U_ref)
% Open-loop (feedforward) speed control from the linear surge model, Problem 3(e)
%   steady state: -Xu*U = (1 - t)*T   ->   T_d = -Xu*U_ref/(1 - t)
%   thrust model: T = rho*D^4*KT*|n|*n ->   n_d = sqrt(T_d/(rho*D^4*KT))

    % parameters (same as ship.m)
    m     = 17.0677e6;
    Xudot = -8.9830e5;
    T1    = 20;                     % surge time constant used in ship.m
    Xu    = -(m - Xudot)/T1;        % linear surge damping
    t_thr = 0.05;                   % thrust deduction number
    rho   = 1025;
    Dia   = 3.3;

    persistent KT
    if isempty(KT)
        [KT, ~] = wageningen(0, 1.5, 0.65, 4);   % same as in ship.m
    end

    T_d = -Xu * U_ref / (1 - t_thr);             % desired thrust (N)
    n_c = sign(T_d) * sqrt(abs(T_d) / (rho * Dia^4 * KT));   % desired shaft speed
end