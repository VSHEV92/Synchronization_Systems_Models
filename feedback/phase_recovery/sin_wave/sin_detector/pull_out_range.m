%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% pull-out range - минимальное значение скачка в расстройке по частоте, 
%% при котором во время переходного процесса появится пропуск циклов.
%% Приближенная и эмпирическая формула получена с помощью фазовых диаграмм: delta_wpo = 1.8 * wn * (1 + ksi)
%% при Kd равном единице.Коэффициент усиления и амплитуда сигнала NCO равны 1.

%% Литература: Gardner Phaselock Technique p.57

Ts = 1e-3;                % шаг дискретизации
Tsim = 5;                 % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

Kd = 1;   % коэффициент усиления фазового детектора
ksi = sqrt(2);            % декремент затухания 
BL = 5;                  % шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

wn = 2 * BL / (ksi + 0.25/ksi);     % резонансная частота (рад/с)
delta_wpo = 1.8 * wn * (1 + ksi);   % ожидаемый pull-out range (рад/с)


%% входной сигнал
delta_w = 28;                            % расстройка по частоте в рад/c
stepTime = 0.3;                         % момент появления скачка
sampleStepTime = round(stepTime/Ts);  % номер отсчета, когда появляется скачек
indata = zeros(1, Nsamp);
for k = sampleStepTime:Nsamp
    indata(k) = indata(k-1) + delta_w * Ts;
end

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
title(num2str(delta_wpo, 'Expected pull-out range=%-d'))



