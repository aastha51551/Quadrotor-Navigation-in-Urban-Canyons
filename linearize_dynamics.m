function [A, B, H] = linearize_dynamics(x0, u0, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw)
    n = length(x0);
    m_in = length(u0);
    d = length(vw);  % disturbance dimension (3 for wind)
    eps = 1e-3;

    A = zeros(n, n);
    B = zeros(n, m_in);
    H = zeros(n, d);  % disturbance matrix

    fx = quad_dynamics(x0, u0, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw);

    % A matrix
    for i = 1:n
        dx = zeros(n, 1);
        dx(i) = eps;
        f_plus = quad_dynamics(x0 + dx, u0, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw);
        A(:, i) = (f_plus - fx) / (eps);
    end

    % B matrix
    for i = 1:m_in
        du = zeros(m_in, 1);
        du(i) = eps;
        f_plus = quad_dynamics(x0, u0 + du, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw);
        B(:, i) = (f_plus - fx) / (eps);
    end

    % H matrix: derivative w.r.t wind
    for i = 1:d
        dw = zeros(d,1);
        dw(i) = eps;
        f_plus = quad_dynamics(x0, u0, dt, m, g, J, L, k_f, k_m, rho, A_drag, C_d, vw + dw);
        H(:, i) = (f_plus - fx) / (eps);
    end
end
