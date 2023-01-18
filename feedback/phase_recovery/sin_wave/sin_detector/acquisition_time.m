%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% pull-in time - интервал времени, в течении которого фапч войдет в синхронный режим, 
%% после множества пропусков циклов.
%% Приближенная формула, при условии, что расстройка по частоте будет много больше, чем lock-in range (больше K).
%% Формула: Tp = (delta_w)^2 / (2 * ksi * wn^3) 
%% При ksi = sqrt(2): Tp = 4.2 * (delta_f)^2 / BL^3
%% Коэффициент усиления (Kg) и амплитуда сигнала NCO равны 1.
%% Литература: Gardner Phaselock Technique p.75

%% Другая формула: (pi^2/16)*(delta_w)^2 / (ksi * wn^3) 
%% Best PLL Design, Simulation and Applications. p. 74

Ts = 1e-3;                % шаг дискретизации
Tsim = 50;                 % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

Kd = 1;                   % коэффициент усиления фазового детектора
ksi = sqrt(2);            % декремент затухания 
BL = 5;                  % шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

wn = 2 * BL / (ksi + 0.25/ksi);     % резонансная частота (рад/с)

%% входной сигнал
delta_phase = -90;
delta_w = 100;
indata = delta_w * Time + (delta_phase / pi * 180);

%% ошибка по фазе, выход фильтра и фаза NCO
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви
for n = 2:Nsamp
    % вычисление ошибки
    err(n) = Kd * sin(indata(n) - nco_phase(n-1));

    % петлевой фильтр
    kp_out = kp * err(n);
    ki_out = ki * err(n) + ki_out_last;
    loop_filter_out(n) = kp_out + ki_out;
    ki_out_last = ki_out;

    % вычисление фазы NCO
    nco_phase(n) = nco_phase(n-1) + loop_filter_out(n); 
endfor

%% значение lock-in range
K = Kd * (kp / Ts);
disp(strcat('Lock-in range = ', num2str(K)));


%% значение pull-in time
Tp = delta_w^2 / (2*ksi*wn^3);

plot(Time, err); grid on;
xlabel('time'); ylabel('phase error')
title(num2str(Tp, 'Expected pull-in time =%-d'))

hold on
plot([Tp Tp], [min(err) max(err)]); 