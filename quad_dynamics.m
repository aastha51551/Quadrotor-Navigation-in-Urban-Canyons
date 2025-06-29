function x_next = quad_dynamics(x, u, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw)
    % State unpacking
    pos = x(1:3);
    vel = x(4:6);
    angles = x(7:9);
    omega = x(10:12);
    phi = angles(1); theta = angles(2); psi = angles(3);

    % Rotation matrix
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

    % Relative velocity
    v_rel = vel - vw;

    % Forces
    thrust = [0; 0; k_f * sum(u)];
    F_drag = -0.5 * rho * A_drag * C_d * norm(v_rel) * v_rel;
    F_total = R * thrust + F_drag + [0; 0; -m*g];

    % Torques
    tau = [L*k_f*(u(2) - u(4));
           L*k_f*(u(3) - u(1));
           k_m*(u(1) - u(2) + u(3) - u(4))];

    % Euler kinematics
    R_euler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
               0 cos(phi)           -sin(phi);
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    eul_dot = R_euler * omega;

    % State update
    x_next = zeros(12,1);
    x_next(1:3) = pos + vel * dt;
    x_next(4:6) = vel + F_total / m * dt;
    x_next(7:9) = angles + eul_dot * dt;
    x_next(10:12) = omega + (J \ (tau - cross(omega, J * omega))) * dt;
end
