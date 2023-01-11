%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Модель фапч с синусоидальным детектором для гармонического сигнала.
%% Коэффициент усиления и амплитуда сигнала NCO равны 1.
%% Рассчетное значение дисперсии фазовой ошибки намного ниже фактического. Так как
%% в линейной модели предполагалось полное подавление слагаемого удвоенной частоты
%% на выходе фазового детектора.

%% Важно!!! 
%% Коэффициент усиления фазового детектора, а значит и все параметры фапч,
%% зависит от амплитуды входного сигнала. Совпадение с расчетными значениями при A_s = 1 

%% --------------------------------------------------------------------------
%% Параметры модели
Ts = 1e-4;      % шаг дискретизации
Tsim = 1;       % время моделирования

A_s = 1;                  % амплитуда входного сигнала
w_s = 2000;               % частота входного сигнала (рад/c)
phase0_s = 30;            % начальная фаза входного сигнала (градусы)
SNR = 10;                 % отношение сигнал/шум на входе (дБ)
w_s_change_time = 0.3;    % момент времени изменения частоты сигнала
w_s_new = 2200;           % новое значение частоты сигнала

ksi = 0.707;    % коэффициент демпфирования 
BL = 30;        % шумовая полоса (Гц)
w0_nco = 1800;    % частота NCO при нулевом входном сигнале
Kd = 0.5;       % коэффициент усиления фазового детектора при входном сигнале с единичной амплитудой

%% --------------------------------------------------------------------------
%% Расчет параметров, необходимых для моделирования
Time = 0:Ts:Tsim;         % массив с отчетами времени
Nsamp = length(Time);     % число отсчетов

w_s_change_sample = round(w_s_change_time/Ts); % номер отсчета изменения частоты сигнала
SNR_lin = 10^(SNR/10);                         % ОСШ в разах
noise_power = A_s^2 / 2 / SNR_lin;             % мощности шума (AWGN)
noise_PSD = noise_power * Ts;                  % спектральная плотность мощности шума

BL_n = BL * Ts;                                 % нормированная шумовая полоса
Kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
Ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви

%% --------------------------------------------------------------------------
%% Расчет характеристик фапч 
wn = 2 * BL / (ksi + 0.25/ksi);                     % резонансная частота (рад/с)
delta_wpo = 1.8 * wn * (1 + ksi);                   % ожидаемый pull-out range (рад/с)
wL = Kd * (Kp / Ts);                                % lock-in range (рад/c) 
Tp = (w_s - w0_nco)^2 / (2*ksi*wn^3);               % pull-in time 
expected_phase_noise = noise_PSD * 2 * BL / A_s;    % фазовый шум при линейном приближении

%% --------------------------------------------------------------------------
%% генерирование входного сигнала

% частота входного сигнала
input_w = [w_s * ones(1,w_s_change_sample), w_s_new * ones(1, Nsamp-w_s_change_sample)];

% фаза входного сигнала
input_phase = zeros(1,Nsamp);
input_phase(1) = phase0_s;
for k = 2:Nsamp
    input_phase(k) = input_phase(k-1) + Ts * input_w(k-1);
endfor

% входной гармонический сигнал
input_signal = A_s*sin(input_phase) + randn(1,Nsamp)*sqrt(noise_power);

plot(input_signal)

%% --------------------------------------------------------------------------
%% моделирование работы фапч
%% ошибка по фазе, выход фильтра и фаза NCO
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

for n = 2:Nsamp
    %% фазовый детектор
    err = input_signal * cos(nco_phase(n-1));

    % петлевой фильтр
    kp_out = Kp * err(n);
    ki_out = Ki * err(n) + ki_out_last;
    loop_filter_out(n) = kp_out + ki_out;
    ki_out_last = ki_out;

    % вычисление фазы NCO
    nco_phase(n) = nco_phase(n-1) + loop_filter_out(n) + w0_nco*Ts; 
endfor

%% ----------------------------------------------------
%% вывод рассчетных значений
disp(['Natural frequency (rad/s): ', num2str(wn)])
disp(['Pull-out range (rad/s): ', num2str(delta_wpo)])
disp(['Lock-in range (rad/s): ', num2str(wL)])
disp(['Pull-in time (s): ', num2str(Tp)])
disp(['Expected phase error variace: ', num2str(expected_phase_noise)])

%% вывод графиков
subplot(3,1,1)
plot(Time, input_signal); hold on;
plot(Time, sin(nco_phase));
xlabel('time (sec)'); ylabel('input and nco signals')

subplot(3,1,2)
plot(Time, loop_filter_out/Ts - input_w + w0_nco)
xlabel('time (sec)'); ylabel('frequency error')

subplot(3,1,3)
plot(Time, err)
xlabel('time (sec)'); ylabel('phase error')