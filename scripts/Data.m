clear
clc
%% Data

g = 5;       % m/s^2 (Gravedad)

% Sistema Mecánico
mw = 0.25; 	% Kg (Masa de la Rueda) - Brida
ma = 0.060;     % Kg (Masa del brazo)
mm = 0.070;       % Kg (Masa del motor) - En el extremo
mp = ma + mm;   % Kg (Masa del péndulo)

r2 = 0.13;                      % m (Radio anillo)
L2 = 0.2;                       % m (Longitud del Péndulo)
L1 = (ma * L2/2 + mm * L2)/mp;  % m (Distancia de la articulación pasiva al centro de gravedad)

J_wheel = mw * r2^2;                            % Kg*m^2 (Momento de inercia de la brida)
Jp = ma * L2^2 / 12;                            % Kg*m^2 (Momento de inercia del brazo)
Jp_O = Jp + ma * (L2/2)^2 + (mm + mw) * L2^2;   % Kg*m^2 (Momento de inercia del péndulo respecto a la articulación pasiva) 

% Supuesto tal que el torque de friccion maximo = 10% del torque maximo del
% motor (0.49N*m) -> 0.049N*m / 10 rad/s = 0.0049 N*m/(rad/s)
bp = 0.03;     % N*m/(rad/s) (Coeficiente de fricción en la articulación pasiva)

% DC Motor Model (Maxon DCX 16 L 24V)
% maxongroup.es/medias/sys_master/root/8992244629534/EN-22-103.pdf
gear = 22;
J_motor = 2.23e-7;              % kg*m^2 (Momento de inercia del rotor)
V_nom = 24;                     % V (Tensión nominal)
I_nom = 1.06;                   % A (Corriente nominal)
Ra = 22.7;                      % Ohm (Resistencia del bobinado)
Kt = gear*34.7e-3;              % N*m/A (Constante de torque)
Ke = gear*34.7e-3;              % V/(rad/s) (Constante de FCEM)
La = 1.56e-3;                  % H (Inductancia del motor)
bm = Kt*0.00805/(6560*2*pi/60);  % N*m/(rad/s) (Coeficiente de fricción del rotor)


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
C = [1 0 0 0];

D = 0;

opensys = ss(A, B, C, D);

%% Controlabilidad
controlability = ctrb(opensys);

%% Observabilidad
observability = obsv(opensys);


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

% LQR

[K, S, Pcl] = lqr(A, B, Q, R);

closedsys = ss(A-B*K, B, C, D);

%% Especificación de sensores

% IMU dinamica y LPF
fs_IMU = 1000;          % Frecuencia de muestreo
wd_IMU = 30000*2*pi;     % Frecuencia natural del IMU


% Giroscopio
PSD_gyro = 0.005;       % Amplitud densidad espectral de potencia [degrees per second/root-Hz] - MPU6050 datasheet
wgyro_LPF = 42*2*pi;      % Frecuencia de corte del LPF
gyro_noise_power = (PSD_gyro*pi/180)^2 * wgyro_LPF/(2*pi); % Potencia de ruido blanco

% Acelerometro
PSD_acc = 400e-6;       % Amplitud densidad espectral de potencia [micro-g/root-Hz] - MPU6050 datasheet
wacc_LPF = 44*2*pi;      % Frecuencia de corte del LPF
acc_noise_power = (PSD_acc*g)^2 * wacc_LPF/(2*pi);  % Potencia de ruido blanco


%% Observador de Kalman

%Torque nominal 0.575
%Torque con variacion de inductancia 0.575
%Son iguales, la incertidumbre en inductancia no cambia nada

Tl = 0;         % Torque de perturbacion (ruido de proceso)

q = 1e-10;       % Ruido de proceso
  
Qe = B*q*B';

r = 5.924e-5;    % Ruido de sensores

Re = [r];

% LQR para observador

% Inclinómetro
[L_t, ~, ~] = lqr(A', C', Qe, Re);

L = L_t';

%% Control de balanceo

Ken = 750;
Kw = 90;
Ku = 0.4;

%% Simulink
F = 0;
theta_c = 0.5; % Angulo de conmutacion de controlador
% Condiciones iniciales
x0 = [0.5 0 0 0];
x0_sw = [pi 0 0 0];

% State Space Block
C_FullState = eye(4);
D1 = zeros(4,1);

% Filtro complementario
alpha = 0.99;

% Altura del acelerometro
ideal_acc_height = 0;
real_acc_height = 0.02; %m

%Frecuencia angular para bloque derivador
w_deriv = (fs_IMU*2*pi)/20;

%Tiempo de muestreo para bloques de ruido blanco
t_noise = 1/fs_IMU;



    