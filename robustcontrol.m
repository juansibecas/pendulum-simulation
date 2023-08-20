%% Robustez, estabilidad y comportamiento dinamico del controlador

% LQR

[K_rob, S, Pcl] = lqr(A, B, Q, R);

closedsys_rob = ss(A-B*K_rob, B, C, D);

N = 5;
disk = zeros(N, 2);
for i=1:N
   aux =  diskmargin(closedsys_rob, -1+i);
   disk(i, :) = aux.GainMargin;
end

figure(1)
diskmarginplot([disk(1,:);disk(2,:);disk(3,:);disk(4,:);disk(5,:)])

figure(2)
pzmap(closedsys_rob)

figure(3)
nyquist(closedsys_rob)

figure(4)
rlocus(closedsys_rob)

figure(5)
bode(closedsys_rob)