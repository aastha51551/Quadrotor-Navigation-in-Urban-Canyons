
altitude = linspace(0, 500, 100);
V_mean = 2 + 0.02 * altitude;  
sigma_w = 0.4 * V_mean; % Wind turbulence intensity
L_w = 10 * ones(size(altitude)); % Turbulence length scale

dt = 0.01; 
T = 10; 
t = 0:dt:T; 

V_wind = zeros(length(altitude), length(t), 3); % [x, y, z] components

for j = 1:length(altitude)
    for i = 2:length(t)
        dW = randn(3,1) * sqrt(dt); % 3D white noise
        
        V_wind(j,i,1) = V_wind(j,i-1,1)*exp(-dt/L_w(j)) + sigma_w(j)*sqrt(1-exp(-2*dt/L_w(j)))*dW(1);
        V_wind(j,i,2) = V_wind(j,i-1,2)*exp(-dt/L_w(j)) + sigma_w(j)*sqrt(1-exp(-2*dt/L_w(j)))*dW(2);
        V_wind(j,i,3) = V_wind(j,i-1,3)*exp(-dt/L_w(j)) + sigma_w(j)*sqrt(1-exp(-2*dt/L_w(j)))*dW(3);
    end
end

figure;
surf(t, altitude, V_wind(:,:,1), 'EdgeColor', 'none'); 
xlabel('Time (s)'); ylabel('Altitude (m)'); zlabel('Wind X (m/s)');
title('Dryden Turbulence - X Component');
colorbar; view(2);

figure;
surf(t, altitude, V_wind(:,:,2), 'EdgeColor', 'none'); 
xlabel('Time (s)'); ylabel('Altitude (m)'); zlabel('Wind Y (m/s)');
title('Dryden Turbulence - Y Component');
colorbar; view(2);

figure;
surf(t, altitude, V_wind(:,:,3), 'EdgeColor', 'none'); 
xlabel('Time (s)'); ylabel('Altitude (m)'); zlabel('Wind Z (m/s)');
title('Dryden Turbulence - Z Component');
colorbar; view(2);

figure('Name', 'Wind Velocity over time across altitudes', 'Position', [100 100 600 400]);
plot(t,V_wind(:,:,3),'b');

assignin('base',"V_wind",V_wind)


