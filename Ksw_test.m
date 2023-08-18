%% Pruebas para determinar la ganancia del controlador de balanceo
%% Ksw entre 0.03 y 0.2. El valor que se elija cambia la frecuencia 
%% del movimiento y la rapidez para lograr la elevacion
%% Ksw = 0.17 es mas rapido

Ksw_v = -10:0.1:10;
warning('off', 'all')
N = length(Ksw_v);
height_reached = zeros(1, N);
time2reach0 = zeros(1,N);
tic
for i = 1:N
    Ksw = Ksw_v(i);
    
    out = sim('swingup_control');
    
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
plot(Ksw_v, height_reached)

figure(3)
plot(Ksw_v, time2reach0)