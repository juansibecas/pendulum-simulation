%% Simulation Data

time = out.animation.time;
N = length(time);

theta = out.animation.signals.values(:,1);
wp = out.animation.signals.values(:,2);
wm = out.animation.signals.values(:,3);
ia = out.animation.signals.values(:,4);
phi = zeros(1,N);
for i = 2:N
   dt = time(i) - time(i-1);
   phi(i) = phi(i-1) + wm(i-1)*dt;
end


%% Function Handles
theta_handle = @(t) interp1(time, theta, t, 'linear');
wp_handle = @(t) interp1(time, wp, t, 'linear');
wm_handle = @(t) interp1(time, wm, t, 'linear');
ia_handle = @(t) interp1(time, ia, t, 'linear');
phi_handle = @(t) interp1(time, phi, t, 'linear');


%% Animation Handles
xp = @(t) L2*sin(theta_handle(t));
yp = @(t) L2*cos(theta_handle(t));

xw = @(t) xp(t) + L2/2 * cos(phi_handle(t));
yw = @(t) yp(t) + L2/2 * sin(phi_handle(t));

th = 0:pi/50:2*pi;
xunit = @(t) L2/2 * cos(th) + xp(t);
yunit = @(t) L2/2 * sin(th) + yp(t);


%% Animation
figure(1)
hold on
axis equal
grid on
fanimator(@(t) plot([0 xp(t)],[0 yp(t)],'r-', lineWidth=2));
fanimator(@(t) plot([xw(t) xp(t)],[yw(t) yp(t)],'g', lineWidth=2));
fanimator(@(t) text(0.1,0.1,"Timer: "+num2str(t,2)))
fanimator(@(t) plot(xunit(t), yunit(t), 'g', lineWidth=2));



%% Plot
figure(2)
hold on
grid on
fplot(@(t) theta_handle(t))
fplot(@(t) phi_handle(t))
legend('pendulum', 'wheel')

%% Respuesta completa de los 4 estados

figure(3)
subplot(2, 2, 1);
fplot(@(t) theta_handle(t), 'linewidth', 2)
xlabel('Tiempo(s)')
ylabel('theta(rad)')
grid on

subplot(2, 2, 2);
fplot(@(t) wp_handle(t), 'color', '#f58231', 'linewidth', 2)
xlabel('Tiempo(s)')
ylabel('wp(rad/s)')
grid on

subplot(2, 2, 3);
fplot(@(t) wm_handle(t), 'color', '#e6194B', 'linewidth', 2)
xlabel('Tiempo(s)')
ylabel('wm(rad/s)')
grid on

subplot(2, 2, 4);
fplot(@(t) ia_handle(t), 'color', '#3cb44b', 'linewidth', 2)
xlabel('Tiempo(s)')
ylabel('Ia(A)')
grid on






