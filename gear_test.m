%% Pruebas para determinar la relacion de transmisión

gear_v = 1:0.1:5;
warning('off', 'all')
N = length(gear_v);
height_reached = zeros(1, N);
time2reach0 = zeros(1,N);
tic
for i = 1:N
    gear = gear_v(i);
    Kt = gear*0.876;        % N*m/A (Constante de torque)
    Ke = gear*Kt;           % V/(rad/s) (Constante de FCEM)
    out = sim('stabilization_control');
    
    theta = out.animation.signals.values(:,1);
    time = out.animation.time;
    
    theta_wrapped = atan2(sin(theta), cos(theta));

    theta_handle = @(t) interp1(time, theta_wrapped, t, 'linear');
    
    resampling_t = 0:0.001:10;
    N_th = length(resampling_t);
    theta_resampled = theta_handle(resampling_t);
    
    end_pos = sum(theta_resampled(N_th-100:N_th))/N_th;
    
    time2reach0_aux = resampling_t(find(abs(theta_resampled) <= 0.02,1));
    if isempty(time2reach0_aux)
        time2reach0(i) = 0; 
    else
        time2reach0(i) = time2reach0_aux; 
    end
    
%      figure(1)
%      hold on
%      plot(time, theta)

    height_reached(i) = min(abs(theta_resampled));
    if mod(i,100)==0
       toc
       disp(i)
       disp('de')
       disp(N)
    end
end

figure(2)
plot(gear_v, height_reached, 'linewidth', 2)
xlabel('gear')
ylabel('Ángulo alcanzado(rad)')

figure(3)
plot(gear_v, time2reach0, 'linewidth', 2)
xlabel('gear')
ylabel('Tiempo de elevación(s)')