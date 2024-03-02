%% Data

g = 9.81;       % m/s^2 (Gravedad)

% Sistema Mecánico
mw = ureal('mw', 0.250, 'percent', 25); 	% Kg (Masa de la Rueda) - Brida
ma = ureal('ma', 0.060, 'percent', 25);     % Kg (Masa del brazo)
mm = ureal('mm', 0.070, 'percent', 20);     % Kg (Masa del motor) - En el extremo
mp = ma + mm;                               % Kg (Masa del péndulo)

r2 = ureal('r2', 0.13, 'percent', 10);      % m (Radio externo brida)
L2 = 0.2;                                   % m (Longitud del Péndulo)
L1 = (ma * L2/2 + mm * L2)/mp;              % m (Distancia de la articulación pasiva al centro de gravedad)

J_wheel = mw * r2^2;                            % Kg*m^2 (Momento de inercia de la brida)
Jp = ma * L2^2 / 12;                            % Kg*m^2 (Momento de inercia del brazo)
Jp_O = Jp + ma * (L2/2)^2 + (mm + mw) * L2^2;   % Kg*m^2 (Momento de inercia del péndulo respecto a la articulación pasiva) 
bp_u = ureal('bp', 0.03, 'percent', 50);        % N*s/m (Coeficiente de fricción en la articulación pasiva)

% DC Motor Model (RS 440 329)
J_motor = 2.23e-7;  % kg*m^2 (Momento de inercia del rotor)
V_nom = 24;         % V (Tensión nominal)
I_nom = 1.06;       % A (Corriente de Arranque)
bm_u = ureal('bm', 8.9457e-06, 'percent', 10);         % N*s/m (Coeficiente de fricción del rotor)
La_u = ureal('La', 1.56e-3, 'percent', 10);        % H (Inductancia del motor)
Ra_u = ureal('Ra', 22.7, 'percent', 10);          % Ohm (Resistencia del bobinado)
Kt_u = ureal('Kt', 22*34.7e-3, 'percent', 10);             % N*m/A (Constante de torque)
Ke_u = ureal('Ke', 22*34.7e-3, 'percent', 10);       % V*s (Constante de FCEM)


% Abreviaciones
beta_u = g*(mp*L1 + mw * L2);
Jw_u = J_motor + J_wheel;
J1_u = Jw_u + Jp_O;
Tr = Jw_u/J1_u;                 % Razón de transferencia de velocidad angular

%% Sistema con incertidumbre

A_unc = [0          1       0       0;
         beta_u/J1_u   -bp_u/J1_u  bm_u/J1_u   -Kt_u/J1_u;
         0          0       -bm_u/Jw_u  Kt_u/Jw_u;
         0          0       -Ke_u/La_u  -Ra_u/La_u];

B_unc = [0 0 0 1/La_u]';

% Inclinómetro
C_unc = [1 0 0 0];

D_unc = 0;

opensys_unc = ss(A_unc, B_unc, C_unc, D_unc);

%% Linear Quadratic Regulator - Control de estabilidad

th_max = 10*pi/180;  % Valor maximo para theta 10 grados
wp_max = 2;          % Velocidad angular maxima pendulo 5rad/s
wm_max = 20;         % Velocidad angular maxima motor 30rad/s

% Regla de Bryson - Costo de cada estado sobre "valor maximo" al cuadrado

R = 1/V_nom^2;    % Costo de entrada

Q_th = 1/th_max^2;
Q_wp = 0.5/wp_max^2;
Q_wm = 0.1/wm_max^2;
Q_ia = 0.1/I_nom^2;

Q = [Q_th   0     0     0;
     0      Q_wp  0     0;   % Se puede agregar un costo cruzado wp*ia para
     0      0     Q_wm  0;   % limitar la potencia
     0      0     0     Q_ia];

[K_unc, S, Pcl] = lqr(A_unc.NominalValue, B_unc.NominalValue, Q, R);

closedsys_unc = ss(A_unc-B_unc*K_unc, B_unc, C_unc, D_unc);

%% Margin

opt = robOptions('Display', 'on', 'Sensitivity', 'on');
%[StabilityMargin, wcu] = robstab(closedsys_unc, opt);

%% Monte Carlo

n_samples = 1000;
samples = usample(closedsys_unc, n_samples);
x0 = [0.3 0 0 0];
F = 0;
for i=1:n_samples
    beta = usample(beta_u);
    bm = usample(bm_u);
    bp = usample(bp_u);
    J1 = usample(J1_u);
    Jw = usample(Jw_u);
    Ke = usample(Ke_u);
    Kt = usample(Kt_u);
    La = usample(La_u);
    Ra = usample(Ra_u);
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
