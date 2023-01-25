%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% lock-in range - максимальное значение расстройки по частоте между сигналом и генератором  
%% при котором синхронизация будет происходить без пропуска цикла (очень быстро).
%% Приближенная получена по аналогии с фапч первого порядка для синусоидального детектора: 
%% delta_wL = K, где K = Kd * kp * Kg
%% Формула приближенная, наличие пропуска цикла зависит также от текущего состояния фапч
%% Коэффициент усиления (Kg) и амплитуда сигнала NCO равны 1.

%% Литература: Gardner Phaselock Technique p.70

Ts = 1e-4;                % шаг дискретизации
Tsim = 5;                 % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

Kd = 1;                   % коэффициент усиления фазового детектора
ksi = sqrt(2);            % декремент затухания 
BL = 10;                  % шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

%% входной сигнал
delta_phase = -90;
delta_w = 40;
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
end

plot(Time, err); grid on;
xlabel('time'); ylabel('phase error')
title(num2str(Kd * (kp / Ts), 'Expected lock-in range=%-d'))