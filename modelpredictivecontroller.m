dt = 1;
T = 10;
t_sim = 0:dt:T;
Ns = length(t_sim);

ref_z = linspace(10, 27, Ns);
ref_x = linspace(0, 0, Ns);
ref_y = linspace(0, 0, Ns);
traj = [ref_x; ref_y; ref_z];

Np = 5;
Nc = 3;

wx = eye(12);
wu = 0.1 * eye(4);
Wx = kron(eye(Np), wx);
Wu = kron(eye(Np), wu);

Sx = [];
Su = [];
for i = 1:Np
    Sx = [Sx; phi^i];
    Su_i_row = [];
    for j = 1:Np
        if j <= i
            Su_i_row = [Su_i_row phi^(i-j) * gamma_U];
        else
            Su_i_row = [Su_i_row zeros(12,4)];
        end
    end
    Su = [Su; Su_i_row];
end

Lambda = kron(eye(Np), eye(4));
for i = 2:Np
    Lambda((i-1)*4+1:i*4, (i-2)*4+1:(i-1)*4) = -eye(4);
end

Lambda0 = zeros(Np*4, 4);
Lambda0(1:4,1:4) = eye(4);

Aeq = zeros(4*(Np-Nc),4*Np);
beq = zeros(4*(Np-Nc),1);
for i = 1:Np-Nc
    Aeq((i-1)*4+1:i*4, 4*(Nc-1)+1:4*Nc) = -eye(4);
    Aeq((i-1)*4+1:i*4, 4*(Nc-1+i)+1:4*(Nc+i)) = eye(4);
end

X = zeros(12, Ns);
X_0 = [0; 0; 10; 0; 0; 0; 0; deg2rad(5); 0; 0; 0; 0.1];
X(:,1) = X_0;

U = zeros(4, Ns);
u_prev = zeros(4,1);

for k=1:Ns-1

    x0 = x_cap(:,k);
    d0 = d_cap(:,k);

    H = 2*(Su' * Wx * Su + Wu);
    f = 2*(Su' * Wx * Sx * x0);

    options = optimoptions('quadprog','Display','off');
    [Delta_U,~,exitflag] = quadprog(H, f, Aeq, beq, [], [], [], [], [], options);

    

    delta_u0 = Delta_U(1:4);
    u_k = u_prev + delta_u0;
    u_prev = u_k;
    U(:,k) = u_k;
    X(:,k+1) = phi * x0+ gamma_U * U(:,k) + gamma_D * d0;
end

a = sum((X(1,:) - traj(1,:)).^2);
disp("SSE pos x (MPC)");
disp(a);
b = sum((X(2,:) - traj(2,:)).^2);
disp("SSE pos y (MPC)");
disp(b);
c = sum((X(3,:) - traj(3,:)).^2);
disp("SSE pos z (MPC)");
disp(c);

u=sum((U - Ui));
disp("SSMV (MPC)");
disp(u);

figure('Name', 'MPC', 'Position', [100 100 600 400]);
plot3(X(1,:),X(2,:),X(3,:), 'm', 'LineWidth', 2);
hold on;
plot3(traj(1,:),traj(2,:),traj(3,:), 'b', 'LineWidth', 1.5);
grid on;
xlabel('X (×10^{-4} m)');
ylabel('Y (×10^{-9} m)');
zlabel('Z (m)');
title('Trajectory of drone');
hold off;

figure('Name', 'MPC (Scaled)', 'Position', [100 100 600 400]);
plot3(X(1,:),X(2,:),X(3,:), 'm', 'LineWidth', 2);
hold on;
plot3(traj(1,:),traj(2,:),traj(3,:), 'b--', 'LineWidth', 1.5);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([0 10]);
ylim([0 10]);
zlim([10 30]);
title('Trajectory of drone');
hold off;
