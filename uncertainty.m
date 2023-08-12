%% Data

g = 9.81;       % m/s^2 (Gravedad)

% Sistema Mecánico
mw = ureal('mw', 0.250, 'percent', 50); 	% Kg (Masa de la Rueda) - Brida
ma = ureal('ma', 0.060, 'percent', 50);     % Kg (Masa del brazo)
mm = ureal('mm', 0.05, 'percent', 50);      % Kg (Masa del motor) - En el extremo
mp = ma + mm;   % Kg (Masa del péndulo)

r1 = ureal('r1', 0.09, 'percent', 10);                      % m (Radio interno brida)
r2 = ureal('r2', 0.13, 'percent', 10);                      % m (Radio externo brida)
L2 = 0.2;                       % m (Longitud del Péndulo)
L1 = (ma * L2/2 + mm * L2)/mp;  % m (Distancia de la articulación pasiva al centro de gravedad)

J_wheel = mw * r2^2;                            % Kg*m^2 (Momento de inercia de la brida)
Jp = ma * L2^2 / 12;                            % Kg*m^2 (Momento de inercia del brazo)
Jp_O = Jp + ma * (L2/2)^2 + (mm + mw) * L2^2;   % Kg*m^2 (Momento de inercia del péndulo respecto a la articulación pasiva) 
bp = ureal('bp', 0.038, 'percent', 100);                                     % N*s/m (Coeficiente de fricción en la articulación pasiva)

% DC Motor Model (RS 440 329)
J_motor = 2.38e-5;  % kg*m^2 (Momento de inercia del rotor)
V_nom = 24;         % V (Tensión nominal)
bm = ureal('bm', 0.018, 'percent', 20);         % N*s/m (Coeficiente de fricción del rotor)
La = ureal('La', 3.2e-3, 'percent', 20);        % H (Inductancia del motor)
Ra = ureal('Ra', 40.8, 'percent', 20);          % Ohm (Resistencia del bobinado)
Kt = ureal('Kt', 1, 'percent', 20);             % N*m/A (Constante de torque)
Ke = ureal('Ke', 1.66e-2, 'percent', 20);       % V*s (Constante de FCEM)


% Abreviaciones
beta = g*(mp*L1 + mw * L2);
Jw = J_motor + J_wheel;
J1 = Jw + Jp_O;
Tr = Jw/J1;                 % Razón de transferencia de velocidad angular

%% Sistema con incertidumbre

A_unc = [0          1       0       0;
         beta/J1   -bp/J1  bm/J1   -Kt/J1;
         0          0       -bm/Jw  Kt/Jw;
         0          0       -Ke/La  -Ra/La];

B_unc = [0 0 0 1/La]';

% Inclinómetro
C_unc = [1 0 0 0];

D_unc = 0;

opensys_unc = ss(A_unc, B_unc, C_unc, D_unc);

%% Control

R_unc = 1;    % Costo de entrada

% Regla de Bryson - Costo de cada estado

Q_unc = [0.01   0     0   0;
         0   0.01     0   0;
         0   0     1   0;
         0   0     0   1];

% LQR

[K_unc, S, Pcl] = lqr(A, B, Q_unc, R_unc);

closedsys_unc = ss(A_unc-B_unc*K_unc, B_unc, C_unc, D_unc);

%% Margin

opt = robOptions('Display', 'on', 'Sensitivity', 'on');
%[StabilityMargin, wcu] = robstab(closedsys_unc, opt);

%% Monte Carlo

Time = 0:0.001:10;
U = zeros(size(Time'));
n_samples = 100;
samples = usample(closedsys_unc, n_samples);

for i=1:n_samples
    y = lsim(samples(:,:,i), U, Time, x0);
    plot(Time, y)
    hold on
end
grid on
set(findall(gcf, 'type', 'line'),'linewidth', 3);
