%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Моделирование прохождения белого шума через фапч. Дисперсия фазового шума должена быть равена
%% произведению дисперсии шума на шумовую полосу фапч.

%% Литература: Gardner Phaselock Technique

Ts = 1e-3;                % шаг дискретизации
Tsim = 100;               % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

ksi = sqrt(2);            % декремент затухания 
BL = 10;                  % односторонняя шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

kp = 4*ksi*BL_n / (ksi + 0.25/ksi);        % усиление пропорциональной ветви
ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2;        % усиление интегрирующей ветви

%% входной сигнал - новмальный белый шум
noise_var = 1;
indata = randn(1, Nsamp) .* sqrt(noise_var);

%% ошибка по фазе, выход фильтра и фаза NCO
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

for n = 2:Nsamp
    % вычисление ошибки
    err(n) = indata(n)/sqrt(Ts) - nco_phase(n-1);

    % петлевой фильтр
    kp_out = kp * err(n);
    ki_out = ki * err(n) + ki_out_last;
    loop_filter_out(n) = kp_out + ki_out;
    ki_out_last = ki_out;

    % вычисление фазы NCO
    nco_phase(n) = nco_phase(n-1) + loop_filter_out(n); 
endfor

%% дисперсия фазовой ошибки
sim_var = var(nco_phase)

%% расчетное значение дисперсии фазовой ошибки
calc_var = var(indata) * 2 * BL

plot(Time, nco_phase); grid on;
xlabel('time'); ylabel('phase noise')

msg = strcat(num2str(sim_var, 'sim value=%-d '), num2str(calc_var, 'calc value=%-d'));
title(msg)


