dt = 1;
T=10;
t_sim = 0:dt:T;
Ns = length(t_sim); 

ref_z = linspace(10, 27, Ns);                    
ref_x = linspace(0, 0, Ns);     
ref_y = linspace(0, 0, Ns);     
traj = [ref_x; ref_y; ref_z]; 

G_inf = pinv(gamma_U) * (phi - eye(size(phi)));

U=zeros(4,Ns);
Us=[1.01;0;0;0];

X=zeros(12,Ns);
X_0=[0; 0; 10; 0; 0; 0; 0;deg2rad(5); 0; 0; 0; 0.1];
Xs=X_0;
X(:, 1) = X_0;

Wx = eye(12);
Wu = 0.1 * eye(4);

for k = 1:Ns
    if  (k<100)
        Ui(:,k)=Us;
    else
        Ui(:,k)=ones(4,1)*1.0129;
    end

end

for i = 1:Ns-1
    U(:, i+1) = -G_inf * (X(:, i));
    X(:, i+1) = phi * x_cap(:, i) + gamma_U * U(:, i)+gamma_D*d_cap(:,i);
end

a = sum((X(1,:) - traj(1,:)).^2);
disp("SSE pos x (LQOC)");
disp(a);
b = sum((X(2,:) - traj(2,:)).^2);
disp("SSE pos y (LQOC)");
disp(b);
c = sum((X(3,:) - traj(3,:)).^2);
disp("SSE pos z (LQOC)");
disp(c);

u=sum((U - Ui));
disp("SSMV (LQOC)");
disp(u);


figure('Name', 'LQOC', 'Position', [100 100 600 400]);
plot3(X(1,:),X(2,:),X(3,:), 'm', 'LineWidth', 2);
hold on;
plot3(traj(1,:),traj(2,:),traj(3,:),'b','LineWidth', 1.5);
grid on;
xlabel('X (×10^{-4} m)');
ylabel('Y (×10^{-9} m)');
zlabel('Z (m)');
title('Trajectory of drone');
hold off;

figure('Name', 'LQOC (Scaled)', 'Position', [100 100 600 400]);
plot3(X(1,:),X(2,:),X(3,:), 'm', 'LineWidth', 2);
hold on;
plot3(traj(1,:),traj(2,:),traj(3,:),'b--','LineWidth', 1.5);
grid on;
xlabel('X m)');
ylabel('Y m)');
zlabel('Z (m)');
xlim([0 10]);
ylim([0 10]);
zlim([10 30]);
title('Trajectory of drone');
hold off;
