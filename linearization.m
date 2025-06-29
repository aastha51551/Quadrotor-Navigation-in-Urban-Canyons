m = 1.26;                      % mass [kg]
g = 9.81;                      % gravity [m/s^2]
J = diag([0.0347, 0.0457, 0.0977]); % inertia matrix [kg*m^2]
L = 0.225;                     % arm length [m]
k_f = 2.98;                 % thrust coefficient
k_m = 1.14e-7;                 % moment (torque) coefficient
C_d = 0.2;                     % drag coefficient
rho = 1.225;                   % air density
a = 0.05;                      % cross-sectional area
x0 = [0; 0; 10;   % position
      0; 0; 0;    % velocity
      0; deg2rad(5); 0;   % angles (phi, theta, psi)
      0; 0; 0];   % body rates

% Call the same wind extraction from before
[~, alt_idx] = min(abs(altitude - 10));
[~, time_idx] = min(abs(t - 2));
vw = squeeze(V_wind(alt_idx, time_idx, :));

% Reuse previous code to compute total thrust T
theta = deg2rad(5);
R = [cos(theta), 0, sin(theta); 0 1 0; -sin(theta), 0, cos(theta)];
F_drag = -0.5 * rho * a * C_d * norm([5; 0; 0] - vw) * ([5; 0; 0] - vw);
T_total = dot(-F_drag - [0; 0; -m*g], R(:,3));
omega_eq = sqrt(T_total / (4 * k_f));
u0 = repmat(omega_eq^2, 4, 1);  

% Compute A and B
[A, B, H] = linearize_dynamics(x0, u0, dt, m, g, J, L, k_f, k_m, rho, a, C_d, vw);
C=eye(12);

assignin('base','A',A);
assignin('base','B',B);
assignin('base','H',H);
assignin('base','C',C);
