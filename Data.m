clear
clc
%% Data

g = 9.81;       % m/s^2 (Gravedad)

% Sistema Mecánico
mw = 0.250; 	% Kg (Masa de la Rueda) - Brida
ma = 0.060;     % Kg (Masa del brazo)
mm = 0.05;      % Kg (Masa del motor) - En el extremo
mp = ma + mm;   % Kg (Masa del péndulo)

r1 = 0.09;                      % m (Radio interno brida)
r2 = 0.13;                      % m (Radio externo brida)
L2 = 0.2;                       % m (Longitud del Péndulo)
L1 = (ma * L2/2 + mm * L2)/mp;  % m (Distancia de la articulación pasiva al centro de gravedad)

J_wheel = mw * r2^2;                            % Kg*m^2 (Momento de inercia de la brida)
Jp = ma * L2^2 / 12;                            % Kg*m^2 (Momento de inercia del brazo)
Jp_O = Jp + ma * (L2/2)^2 + (mm + mw) * L2^2;   % Kg*m^2 (Momento de inercia del péndulo respecto a la articulación pasiva) 
bp = 0.038;                                     % N*s/m (Coeficiente de fricción en la articulación pasiva)

% DC Motor Model (RS 440 329)
J_motor = 2.38e-5;  % kg*m^2 (Momento de inercia del rotor)
V_nom = 24;         % V (Tensión nominal)
bm = 0.018;         % N*s/m (Coeficiente de fricción del rotor)
La = 3.2e-3;        % H (Inductancia del motor)
Ra = 40.8;          % Ohm (Resistencia del bobinado)
Kt = 1;             % N*m/A (Constante de torque)
Ke = 1.66e-2;       % V*s (Constante de FCEM)


% Abreviaciones
beta = g*(mp*L1 + mw * L2);
Jw = J_motor + J_wheel;
J1 = Jw + Jp_O;
Tr = Jw/J1;                 % Razón de transferencia de velocidad angular
wn = sqrt(mp*g*L2/Jp_O);    % Frecuencia de pequeñas oscilaciones

%% Representación en espacio de estados

A = [0          1       0       0;
     beta/J1   -bp/J1  bm/J1   -Kt/J1;
     0          0       -bm/Jw  Kt/Jw;
     0          0       -Ke/La  -Ra/La];
 
B = [0 0 0 1/La]';

% Inclinómetro
C_acc = [1 0 0 0];

D_acc = 0;

% Inclinómetro + Encoder en la rueda
C_acc_enc = [1 0 0 0;
             0 0 1 0];

D_acc_enc = zeros(2, 1);

% Inclinómetro + Doble Encoder

C_acc_double_enc = [1 0 0 0;
                    0 0 1 0;
                    0 1 0 0];
                
D_acc_double_enc = zeros(3,1);

%% Controlabilidad

% Inclinómetro
opensys_acc = ss(A, B, C_acc, D_acc);
% Inclinómetro + Encoder en la rueda
opensys_acc_enc = ss(A, B, C_acc_enc, D_acc_enc);
% Inclinómetro + Doble Encoder
opensys_acc_double_enc = ss(A, B, C_acc_double_enc, D_acc_double_enc);

controlability = ctrb(opensys_acc);


%% Observabilidad
observability = zeros(3,1);

observability(1) = rank(obsv(opensys_acc));
observability(2) = rank(obsv(opensys_acc_enc));
observability(3) = rank(obsv(opensys_acc_double_enc));


%% Linear Quadratic Regulator - Control de estabilidad
R = 0.1;    % Costo de entrada

% Regla de Bryson - Costo de cada estado

Q = [1   0     0   0;
     0   0.5   0   0;
     0   0     0.1 0;
     0   0     0   0.1];

% LQR

[K, S, Pcl] = lqr(A, B, Q, R);


%% Observador de Kalman

q = 1e-10;       % Ruido de proceso / Ruido de sensores
  
Qe = B*q*B';

r = 1.54e-5;

Re = [r];

% LQR para observador

% Inclinómetro
[L_t, ~, ~] = lqr(A', C_acc', Qe, Re);

L_acc = L_t';

% Inclinómetro + Encoder en la rueda
[L_t, ~, ~] = lqr(A', C_acc_enc', Qe, Re);

L_acc_enc = L_t';

% Inclinómetro + Doble Encoder
[L_t, ~, ~] = lqr(A', C_acc_double_enc', Qe, Re);

L_acc_double_enc = L_t';

%% Control de balanceo

Ksw = 0.035;

%% Especificación de sensores

% Giroscopio
PSD_gyro = 0.005;       % Power Spectral Density Amplitude [degrees per second/root-Hz] - MPU6050 datasheet
gyro_noise_power = (PSD_gyro*pi/180)^2; % Potencia de ruido blanco
gyro_bias = 0.05;

% Acelerometro
PSD_acc = 400e-6;       % Power Spectral Density Amplitude [micro-g/root-Hz] - MPU6050 datasheet
acc_noise_power = (PSD_acc*g)^2;  % Potencia de ruido blanco
acc_bias = 0;

% IMU dinamica y LPF
fs_IMU = 1000;          % Frecuencia de muestreo
zetad_IMU = sqrt(2)/2;  % Amortiguamiento del IMU
wd_IMU = 30000*2*pi;     % Frecuencia natural del IMU
zetaf_IMU = sqrt(2)/2;  % Amortiguamiento del LPF
wf_IMU = 40*2*pi;      % Frecuencia de corte del LPF
delay_IMU = 5e-3;       % Delay del LPF


%% Simulink

% Condiciones iniciales
x0 = [0.2 0 0 0];
x0_sw = [pi 0 0 0];

% State Space Block
C_FullState = eye(4);
D1 = zeros(4,1);

% Filtro complementario
simulation_dt = 0.001;
alpha = 0.99;

% Altura del acelerometro
ideal_acc_height = 0;
real_acc_height = L1/3;








    