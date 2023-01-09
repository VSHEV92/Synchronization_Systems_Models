%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Упрощенная модель фапч с синусоидальным детектором.
%% Коэффициент усиления и амплитуда сигнала NCO равны 1.

%% Литература: Gardner Phaselock Technique p.30

Ts = 1e-3;                % шаг дискретизации
Tsim = 1;                 % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

ksi = sqrt(2);            % декремент затухания 
BL = 10;                  % шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

%% параметры синусоидального фазового детектора
As = 1;        % амплитуда сигнала
Kd = As / 2;   % коэффициент фазового детектора

%% входной сигнал
delta_phase = 30;  % расстройка по фазе в градусах
delta_w = 10;      % расстройка по частоте в рад/c
indata = Time * delta_w + (delta_phase/180*pi);

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

subplot(4,1,1)
plot(Time, indata); grid on;
xlabel('time'); ylabel('input signal phase')

subplot(4,1,2)
plot(Time, nco_phase); grid on;
xlabel('time'); ylabel('NCO phase')

subplot(4,1,3)
plot(Time, err); grid on;
xlabel('time'); ylabel('phase error')

subplot(4,1,4)
plot(Time, loop_filter_out / Ts); grid on;
xlabel('time'); ylabel('NCO frequency')