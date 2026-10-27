function [Kp,Kd,Ki] =  pid_desing_nomoto(w_b, zeta, T, K)
    
    %% Task 3.e
    % PID pole placement for the Nomoto model, Fossen (2021) Alg. 15.1
    w_n   = w_b / sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2));
    m_nom = T/K;
    d_nom = 1/K;
    Kp = m_nom * w_n^2;
    Kd = 2*zeta*w_n*m_nom - d_nom;
    Ki = (w_n/10) * Kp;

end


