%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all

%% Пропорционально-интегрирующий петлевой фильтр. 
%% Влияние некорректного значения коэффициента усиления фазового детектора на
%% шумовую полосу, коэффициент демпфирования и резонансную частоту.

%% Коэффициенты Kp и Ki рассчитываются для единичного значения Kd, при заданных значения BL и ksi,
%% BL и ksi пересчитываются с учетом нового Kd.

%% Литература: Gardner Phaselock Technique

Kd_ratio = 2; % отношение исходного и фактического усиления фазового детектора

%% исходные значения 
disp('Expected values: ')
ksi_init = 0.707
BL_init = 10
wn_init = 2 * BL_init / (ksi_init + 0.25/ksi_init)

%% коэффициенты фильтра
Kp = 4*ksi_init*BL_init / (ksi_init + 0.25/ksi_init);   % усиление пропорциональной ветви
Ki = 4*BL_init^2 / (ksi_init + 0.25/ksi_init)^2;        % усиление интегрирующей ветви

%% фактические значения
disp('')
disp('Actual values: ')
wn_actual = sqrt(Kd_ratio * Ki);
ksi_actual = 0.5 * wn_actual * Kp / Ki
BL_actual = 0.5 * wn_actual * (ksi_actual + 0.25 ./ ksi_actual)
wn_actual