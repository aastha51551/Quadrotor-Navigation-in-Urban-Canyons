dt = 1;
T=10;
t_sim = 0:dt:T;
Ns = length(t_sim);

X_0=[0; 0; 10; 0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0.1];
Xs=X_0;
Us=[1.01;0;0;0];
t = (0:Ns-1) * dt;

v_x = zeros(1,Ns);
v_y = zeros(1,Ns);
v_z = zeros(1,Ns);


x_cap=zeros(12,Ns);
d_cap=zeros(3,Ns);
X = zeros(12, Ns);
X(:, 1) = X_0;
U = zeros(4, Ns);
e=zeros(12,Ns);

sys_poles = eig(phi);

desired_observer_poles = 0.8 * ones(size(phi,1),1); 
L_T = place(phi', C', desired_observer_poles)';
L = L_T';  


idx_t = round(t_sim/dt) + 1;  
for k = 1:Ns-1
    
    current_altitude = X(3, k);  
    [~, idx_alt_dynamic] = min(abs(altitude - current_altitude));
    v_x(k) = V_wind(idx_alt_dynamic, idx_t(k), 1);
    v_y(k) = V_wind(idx_alt_dynamic, idx_t(k), 2);
    v_z(k) = V_wind(idx_alt_dynamic, idx_t(k), 3);

    D(:,k) = [v_x(k); v_y(k); v_z(k)];
   
end
D(:,Ns)=D(:,Ns-1);

for k = 1:Ns-1
    if  (k<100)
        U(:,k)=Us;
    else
        U(:,k)=ones(4,1)*1.0129;
    end

end

for k = 1:Ns-1
    X(:, k+1) = phi * X(:, k) + gamma_U * U(:, k)+ gamma_D*D(:,k); 
end

x_cap_0=Xs;
x_cap(:,1)=x_cap_0;

for k=1:Ns-1
    e(:,k)=C*(X(:,k)- x_cap(:,k));
    d_cap(:,k) = pinv(C * gamma_D) * e(:,k);
    x_cap(:,k+1)=phi*x_cap(:,k)+gamma_U*U(:,k)+ gamma_D*d_cap(:,k)+ L*e(:,k);
end



figure('Name', 'Observer State Estimation', 'Position', [100 100 600 400]);

% Plot for State 1
subplot(3, 4, 1);
plot(t_sim, X(1, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(1, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 1');
legend('Actual', 'Predicted');

% Plot for State 2
subplot(3, 4, 2);
plot(t_sim, X(2, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(2, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 2');
legend('Actual', 'Predicted');

% Plot for State 3
subplot(3, 4, 3);
plot(t_sim, X(3, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(3, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 3');
legend('Actual', 'Predicted');

% Plot for State 4
subplot(3, 4, 4);
plot(t_sim, X(4, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(4, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 4');
legend('Actual', 'Predicted');

% Plot for State 5
subplot(3, 4, 5);
plot(t_sim, X(5, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(5, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 5');
legend('Actual', 'Predicted');

% Plot for State 6
subplot(3, 4, 6);
plot(t_sim, X(6, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(6, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 6');
legend('Actual', 'Predicted');

% Plot for State 7
subplot(3, 4, 7);
plot(t_sim, X(7, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(7, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 7');
legend('Actual', 'Predicted');

% Plot for State 8
subplot(3, 4, 8);
plot(t_sim, X(8, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(8, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 8');
legend('Actual', 'Predicted');

% Plot for State 9
subplot(3, 4, 9);
plot(t_sim, X(9, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(9, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 9');
legend('Actual', 'Predicted');

% Plot for State 10
subplot(3, 4, 10);
plot(t_sim, X(10, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(10, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 10');
legend('Actual', 'Predicted');

% Plot for State 11
subplot(3, 4, 11);
plot(t_sim, X(11, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(11, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 11');
legend('Actual', 'Predicted');

% Plot for State 12
subplot(3, 4, 12);
plot(t_sim, X(12, :), 'b', 'LineWidth', 1.5); % Actual state
hold on;
plot(t_sim, x_cap(12, :), 'r--', 'LineWidth', 1.5); % Predicted state
xlabel('Time (s)');
ylabel('State 12');
legend('Actual', 'Predicted');

figure('Name', 'Disturbance estimation', 'Position', [100 100 600 400]);
subplot(3,1,1);
plot(t_sim, D(1,:), 'b', 'LineWidth',1.5); hold on;
plot(t_sim, d_cap(1,:), 'r--', 'LineWidth',1.5);
title('Disturbance - X Axis');
legend('True Disturbance', 'Estimated Disturbance');

subplot(3,1,2);
plot(t_sim, D(2,:), 'b', 'LineWidth',1.5); hold on;
plot(t_sim, d_cap(2,:), 'r--', 'LineWidth',1.5);
title('Disturbance - Y Axis');
legend('True Disturbance', 'Estimated Disturbance');

subplot(3,1,3);
plot(t_sim, D(3,:), 'b', 'LineWidth',1.5); hold on;
plot(t_sim, d_cap(3,:), 'r--', 'LineWidth',1.5);
title('Disturbance - Z Axis');
legend('True Disturbance', 'Estimated Disturbance');

assignin('base','x_cap',x_cap);
assignin('base','d_cap',d_cap);
