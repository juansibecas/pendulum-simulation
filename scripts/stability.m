%% Puntos estabilizables en diagrama de fases
n_theta = 18;
n_wp = 2;
n_wm = 30;
IT = n_theta*n_wm*n_wp; %numero total de simulaciones

theta_v = linspace(0, pi/2, n_theta);
wp_v = linspace(0, -5, n_wp);
wm_v = linspace(0, -30, n_wm);

x0 = [0 0 0 0];

theta_end = zeros(1, IT); %donde termino cada simulacion
theta_full = cell(IT, 1);
ics = zeros(IT, 3); %condiciones iniciales de cada iteracion

tic
%% Condiciones Iniciales
it = 0;
for i=1:n_theta
    x0(1) = theta_v(i);
    for j=1:n_wp
        x0(2) = wp_v(j);
        for k=1:n_wm
            x0(3) = wm_v(k);
            it = it + 1;
            ics(it, :) = x0(1:3);
        end
    end
end
toc

%% Barrido de Parametros
tic
for i=1:IT
    x0(1:3) = ics(i,:);
    out = sim('stabilization_control'); 
    theta = out.phase_stability.signals.values(:,1);
    end_pos = sum(theta(end-100:end))/100;
    theta_end(i) = end_pos;
    theta_full(i) = {theta};
    if mod (i, 100) == 0
       toc
    end
end

%% Analisis de condiciones iniciales estabilizables

stabilized_ics = zeros(1, 3); %Condiciones iniciales desde las que se logro estabilizar

%Cada grupo corresponde a una velocidad angular inicial distinta del
%pendulo
groups_v = ones(1, 1); %grupo al que pertenece cada elemento de stabilized_ics
group_size = IT/n_wp; %cantidad de elementos en cada grupo

%Condiciones iniciales desde las que se logro estabilizar y grupo al que
%pertenece cada una
for i=1:IT
    if abs(theta_end(i)) < 0.1
        stabilized_ics(end+1, :) = ics(i, :);
        groups_v(end+1) = ceil(i/group_size);
    end
end

%cell array, cada una tiene el conjunto de condiciones iniciales
%correspondiente a ese grupo
groups = cell(n_wp,1);

%colores para graficar en diagrama de fase, %%%%automatizar colores para
%cualquier cantidad de grupos
colors = ['#0072BD'; '#D95319'; '#EDB120'; '#7E2F8E'; '#77AC30'; '#4DBEEE'];

for i=1:n_wp
    idxs = groups_v == i; %indices de cada grupo
    group = stabilized_ics(idxs, logical([1 0 1])); %extrae las condiciones iniciales (theta, wm) de cada grupo
    groups(i) = {group};
    group_wp = wp_v(i);
    
    %% Scatter (theta, wm) de cada grupo
    figure(1)
    hold on
    scatter3(group(:,1), group(:,2), group_wp*ones(1,length(group(:,1))), 'MarkerEdgeColor', colors(i, :))
    
    %% Regresion lineal de cada grupo
    p = polyfit(group(:,1), group(:,2), 1);
    theta_min = min(group(:,1));
    theta_max = max(group(:,1));
    group_theta = linspace(theta_min, theta_max);
    group_wm = polyval(p, group_theta);
    plot3(group_theta, group_wm, group_wp*ones(1,length(group_theta)), 'color', colors(i, :))
    
    %% Grafico energia-theta de cada grupo
%     figure(2)
%     hold on
%     group_energy = 1 + cos(group_theta) + 0.5*(group_wm/wn).^2; %+ 0.5*Jw/(mp*g*L2)*(group_wp).^2;
%     plot3(group_theta, group_energy, group_wp*ones(1,length(group_theta)), 'color', colors(i, :))
end




