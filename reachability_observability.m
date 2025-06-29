Ts=0.1;
phi = expm(A*Ts);
gamma_U = (integral(@(t) expm(A*t)*B,0,Ts,'ArrayValued',true));
gamma_D = integral(@(t) expm(A*t)*H,0,Ts,'ArrayValued',true);

n = size(phi,1);  
R = gamma_U;
for i = 1:n-1
    R = [R, phi^i * gamma_U];
end
rank_R = rank(R);
disp(['Rank of reachability matrix: ', num2str(rank_R)]);
if (rank_R==n)
    disp("Reachable");
end

W = C;
for j = 1:n-1
    W = [W, C*phi^j ];
end
rank_W = rank(W);
disp(['Rank of Observability matrix: ', num2str(rank_W)]);
if (rank_W==n)
    disp("Observable");
end

assignin('base','phi',phi);
assignin('base','gamma_U',gamma_U);
assignin('base','gamma_D',gamma_D);

