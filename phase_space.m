%% Simulation Data

tic
x0_sw = [-pi 0 0 0];
out = sim('inverted_pendulum');
toc

time = out.sim_sw_real.time;

theta = out.sim_sw_real.signals.values(:,1);
wp = out.sim_sw_real.signals.values(:,2);
wm = out.sim_sw_real.signals.values(:,3);

%--------------------------------------------------------------------------
figure(1);
plot3(theta, wm, wp);
