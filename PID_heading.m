function delta_c = PID_heading(e_psi, e_r, e_int, Kp, Kd, Ki)
    delta_c = -Kp*e_psi - Kd*e_r - Ki*e_int;
end