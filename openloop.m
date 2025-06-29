% Parameters
m = 1.26;                      % mass [kg]
g = 9.81;                      % gravity [m/s^2]
J = diag([0.0347, 0.0457, 0.0977]); % inertia matrix [kg*m^2]
L = 0.225;                     % arm length [m]
k_f = 2.98;                 % thrust coefficient
k_m = 1.14e-7;                 % moment (torque) coefficient
C_d = 0.2;                     % drag coefficient
rho = 1.225;                   % air density
A = 0.05;                      % cross-sectional area

% Time settings
dt = 0.01;
t_sim = 0:dt:10;
N = length(t_sim);

ref_z = linspace(10, 30, N);                    
ref_x = linspace(0, 0, N);     
ref_y = linspace(0, 0, N);     
traj = [ref_x; ref_y; ref_z]; 

% Initial state
x = zeros(12, N); % [x y z xdot ydot zdot phi theta psi p q r]
x(:,1) = [0; 0; 10; 0; 0; 0; 0;deg2rad(5); 0; 0; 0; 0.1];


omega_hover = (sqrt((m * g) / (4 * k_f)))*ones(4,1);


% Simulation loop
for i = 1:N-1
    % State components
    pos = x(1:3,i);
    vel = x(4:6,i);
    angles = x(7:9,i);
    omega = x(10:12,i);
    
    x(8,i) = deg2rad(5);
    % Get wind at current altitude (nearest neighbor)
    [~, alt_idx] = min(abs(altitude - pos(3)));
    alt_idx = min(max(alt_idx, 1), size(V_wind,1));
    vw = squeeze(V_wind(alt_idx, i, :));
    
    % Relative velocity (in world frame)
    v_rel = vel+vw ;
    

    % Rotation matrix (body to world)
    phi = angles(1); theta = angles(2); psi = angles(3);
    R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta)];
    
    
   
    
    % Forces
    
    F_drag = -0.5*rho*A*C_d*norm(v_rel)*v_rel;
   

    F_gravity = [0; 0; -m * g];

    
    T_vec = -F_drag - F_gravity;
    thrust = dot(R(:,3), T_vec);
    omega_r=sqrt(thrust/(4*k_f))*ones(4,1)*1.1;
    if (t_sim < 1)
        thrust_body = [0; 0; k_f*sum(omega_hover.^2)];
    else
        thrust_body = [0; 0; k_f*sum(omega_r.^2)];
    end
    F_total = R * thrust_body + F_drag + [0; 0; -m*g];

    % Torques
    tau = [
        L*k_f*(omega_r(2)^2 - omega_r(4)^2);
        L*k_f*(omega_r(3)^2 - omega_r(1)^2);
        k_m*(omega_r(1)^2 - omega_r(2)^2 + omega_r(3)^2 - omega_r(4)^2)
    ];

    % State integration
    x(1:3,i+1) = pos + v_rel * dt;
    x(4:6,i+1) = v_rel+ (F_total / m) * dt;
    x(7:9,i+1) = angles + euler_kinematics(angles, omega) * dt;
    x(10:12,i+1) = omega + (J \ (tau - cross(omega, J * omega))) * dt;
end

% Euler kinematics function
function angle_rates = euler_kinematics(angles, omega)
    phi = angles(1); theta = angles(2);
    if abs(cos(theta)) < 1e-5
        theta = sign(theta)*(pi/2 - 1e-5);
    end
    R_euler = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
               0 cos(phi)           -sin(phi);
               0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    angle_rates = R_euler * omega;
end
% Time vector
T = t_sim;

% Plot X, Y, Z position over time
figure;
subplot(3,1,1);
plot(T, x(1,:), 'r', 'LineWidth', 1);
hold on;
plot(T,traj(1,:),'b','LineWidth', 1.5);
ylabel('X [m]'); title('Position Over Time (Open loop)'); grid on;

subplot(3,1,2);
plot(T, x(2,:), 'g', 'LineWidth', 2);
hold on;
plot(T,traj(2,:),'b','LineWidth', 1.5);
ylabel('Y [m]'); grid on;

subplot(3,1,3);
plot(T, x(3,:), 'k', 'LineWidth', 2);
hold on;
plot(T,traj(3,:),'b','LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Z [m]'); grid on;

figure;
subplot(3,1,1);
plot(T, x(4,:), 'r', 'LineWidth', 2);
ylabel('Xv'); title('Velocity Over Time (Open loop)'); grid on;

subplot(3,1,2);
plot(T, x(5,:), 'g', 'LineWidth', 2);
ylabel('Yv'); grid on;

subplot(3,1,3);
plot(T, x(6,:), 'b', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Zv'); grid on;

figure;
plot3(x(1,:),x(2,:),x(3,:), 'm', 'LineWidth', 2);
hold on;
plot3(traj(1,:),traj(2,:),traj(3,:),'b','LineWidth', 1.5);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trajectory of drone (Open loop)');
hold off;


info_z = stepinfo(x(3,:), T, traj(3,end));
disp(info_z);
