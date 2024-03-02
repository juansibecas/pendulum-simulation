%% Rechazo a perturbaciones
%% Monte Carlo

F_v = L2*(1:10)/10; %Nm
n_samples = length(F_v);

for i=1:n_samples
    F = F_v(i);
    out = sim('stabilization_control');
    theta = out.animation.signals.values(:,1);
    time = out.animation.time;
    plot(time, theta)
    hold on
end
grid on
set(findall(gcf, 'Type', 'Line'), 'linewidth', 2)
xlabel('Tiempo(s)')
ylabel('Theta(rad)')