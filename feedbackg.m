
n = size(phi, 1);
R = gamma_U;
for i = 1:n-1
    R = [R, phi^i * gamma_U];
end

lambda = 0.4;
alpha_lambda = (lambda .^ (n-1:-1:0))';

R_inv = pinv(R);

G_0 = R_inv * alpha_lambda;
G=([G_0(1:12,:) G_0(13:24,:) G_0(25:36,:) G_0(37:48,:)])*1e-7;
G=G';

assignin('base','G',G);
