%% alpha  0.9 -> 0.9999 => 0.99 - 0.995 es optimo

alpha_v = [0.9 0.92 0.94 0.96 0.98 0.99 0.992 0.994 0.996 0.998 0.999 0.9995 0.9999 0.99999];
N = length(alpha_v);
theta_rms = zeros(1,N);

for i = 1:N
    alpha = alpha_v(i);
    
    out = sim('stabilization_control');
    
    theta = out.animation.signals.values(:,1);
    time = out.animation.time;

    theta_handle = @(t) interp1(time, theta, t, 'linear');

    theta_resampled = theta_handle(0:0.001:10);
    figure(1)
    hold on
    fplot(@(t) theta_handle(t))
    
    
    
    N_th = length(theta_resampled);

    theta_rms(i) = sqrt(sum(theta_resampled.^2)/N_th);
end
legend('1', '2', '3', '4', '5','6','7','8','9', '10', '11', '12', '13', '14')

figure(2)
plot(alpha_v, theta_rms)
