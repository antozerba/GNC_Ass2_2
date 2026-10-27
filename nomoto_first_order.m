function [T,K] = nomoto_first_order(u_d)

    %% Task 2.b
    % ---- Parameters (same as ship.m) ----
    m     = 17.0677e6;   Iz = 2.1732e10;   xg = -3.7;   L = 161;
    Xudot = -8.9830e5;   Yvdot = -5.1996e6;   Yrdot = 9.3677e5;
    Nvdot = Yrdot;       Nrdot = -2.4283e10;
    rho   = 1025;
    
    % rudder coefficients (copied from ship.m)
    b_r = 2; AR = 8; 
    lambda = b_r^2/AR;
    CN = 6.13*lambda/(lambda + 2.25);
    aH = 0.75; xH = -0.4*L; xR = -0.5*L;
    Y_delta = 0.25*(1 + aH)*rho*AR*CN;
    N_delta = 0.25*(xR + aH*xH)*rho*AR*CN;
    
    % linear damping (as in ship.m)
    Yv = -(m - Yvdot)/20;
    Nr = -(Iz - Nrdot)/10;
    
    % ---- Sway-yaw model: M*nu_r_dot + N*nu_r = b*delta ----
    M = [ m - Yvdot,     m*xg - Yrdot
    m*xg - Nvdot,  Iz - Nrdot  ]; %Mrb + Madd
    
    CRB_star = [0  m*u_d
        0  m*xg*u_d];
    CA_star  = [0                    -Xudot*u_d
        (Xudot - Yvdot)*u_d  -Yrdot*u_d];
    D_lin    = -diag([Yv Nr]);
    N = CRB_star + CA_star + D_lin;
    b = 2*u_d*[-Y_delta; -N_delta];   
    
    %----- state space model -----
    Minv = invQR(M);
    
    A = -Minv*N;
    B = Minv*b;
    C = [0 1];         % r since we want rudder -> r as tf
    D = 0;             % rudder is already on the state 
    
    % ---- Transfer function r/delta ----
    [num, den] = ss2tf(A, B, C, D);
    H = tf(num, den);  
    
    %% Task 2.c
    % --- Finding the T constants -------
    p  = roots(den);                 % poles
    z  = roots(num);                 % zero
    Tp = sort(-1./p, 'descend');
    T1 = Tp(1);
    T2 = Tp(2);
    T3 = -1/z;
    K  = num(end)/den(end);          % gain for s = 0
    T  = T1 + T2 - T3;               % 1st-order Nomoto time constant
    
    fprintf('T1 = %.2f s\nT2 = %.2f s\nT3 = %.2f s\n', T1, T2, T3);
    fprintf('K  = %.4e 1/s\nT  = %.2f s\n', K, T);
end