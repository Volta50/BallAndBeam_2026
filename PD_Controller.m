% Control PD para planta Ku/s^2
% Objetivo: hallar Kp y Kd dado ts y zeta

clear; clc;

%% Parámetros del sistema
d = 3;
L = 46.2;

K_bb = -3.613;
Ku   = K_bb*(d/L)   % Ganancia de la planta (ajustar según tu sistema)


ts   = 7;    % Tiempo de asentamiento [s] (criterio 2%)
zeta = 0.95;  % Factor de amortiguamiento

%% Cálculo de parámetros del sistema de 2do orden deseado
% Criterio 2%: ts ≈ 4 / (zeta * wn)
wn = 3.5 / (zeta * ts);

fprintf('=== Parámetros deseados ===\n');
fprintf('wn   = %.4f rad/s\n', wn);
fprintf('zeta = %.4f\n', zeta);
fprintf('ts   = %.4f s (teórico)\n\n', 4/(zeta*wn));

%% FT en lazo cerrado con controlador PD: C(s) = Kp + Kd*s
% G(s) = Ku/s^2,  C(s)*G(s) = Ku*(Kd*s + Kp)/s^2
% Denominador CL: s^2 + Ku*Kd*s + Ku*Kp
%
% Comparando con s^2 + 2*zeta*wn*s + wn^2:
%   Ku*Kp = wn^2      → Kp = wn^2 / Ku
%   Ku*Kd = 2*zeta*wn → Kd = 2*zeta*wn / Ku

Kp = wn^2 / Ku;
Kd = 2 * zeta * wn / Ku;

fprintf('=== Ganancias del controlador PD ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n\n', Kd);

%% Simulación
s  = tf('s');
G  = Ku / s^2;
C  = Kp + Kd * s;
CL = feedback(C * G, 1);

fprintf('=== Polos en lazo cerrado ===\n');
disp(pole(CL));

% Respuesta al escalón
figure;
step(CL);
title(sprintf('Respuesta al escalón — Kp=%.3f, Kd=%.3f', Kp, Kd));
xlabel('Tiempo (s)'); ylabel('Salida');
grid on;

% Métricas reales
info = stepinfo(CL);
fprintf('=== Métricas reales ===\n');
fprintf('ts real    = %.4f s\n', info.SettlingTime);
fprintf('Overshoot  = %.2f %%\n', info.Overshoot);