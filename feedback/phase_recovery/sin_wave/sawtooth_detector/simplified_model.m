%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Упрощенная модель фапч с sawtooth детектором (встречается при использовании арктангенса).
%% Коэффициент усиления и амплитуда сигнала NCO равны 1.

%% Литература: 
%% Общая модель: Gardner Phaselock Technique p.30
%% pull-out range: Venceslav F. Kroupa Phase Lock Loops and Frequency Synthesis p. 132
%% pull-in time: Venceslav F. Kroupa Phase Lock Loops and Frequency Synthesis p. 141

%% Другие формулы: 
%% pull-out range: Best PLL Design, Simulation and Applications. p. 76
%% delta_wpo = 5.78 * wn * (0.5 + ksi);

%% pull-in time: Best PLL Design, Simulation and Applications. p. 76
%% Tp = (w_s/wn)^2 / (pi^2) / (ksi*wn); 


%% --------------------------------------------------------------------------
%% Параметры модели
Ts = 1e-3;                % шаг дискретизации
Tsim = 6;                 % время моделирования

phase0_s = 30;            % начальная фаза входного сигнала (градусы)
w_s = 200;                % значение частоты сигнала
w_s_change_time = 3;      % момент времени изменения частоты сигнала
w_s_new = 60;             % новое значение частоты сигнала

ksi = sqrt(2);            % декремент затухания 
BL = 10;                  % шумовая полоса
PD_max = pi;              % максимальное значение на выходе детектора
Kd = 1;                   % коэффициент усиления фазового детектора

%% --------------------------------------------------------------------------
%% Расчет параметров, необходимых для моделирования
Time = 0:Ts:Tsim;         % массив с отчетами времени
Nsamp = length(Time);     % число отсчетов

w_s_change_sample = round(w_s_change_time/Ts); % номер отсчета изменения частоты сигнала

BL_n = BL * Ts;           % нормированная шумовая полоса
Kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
Ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви

%% --------------------------------------------------------------------------
%% Расчет характеристик фапч 
wn = 2 * BL / (ksi + 0.25/ksi);                     % резонансная частота (рад/с)
delta_wpo = pi * wn * 1.83 * (0.53 + ksi);          % ожидаемый pull-out range (рад/с)
wL = PD_max * Kd * (Kp / Ts);                       % lock-in range (рад/c) 
Tp = (w_s/wn)^2 * (1.5/pi^2) / (2*ksi*wn);          % pull-in time 

%% --------------------------------------------------------------------------
%% входной сигнал
% частота входного сигнала
input_w = [w_s * ones(1,w_s_change_sample), w_s_new * ones(1, Nsamp-w_s_change_sample)];

% фаза входного сигнала
input_phase = zeros(1,Nsamp);
input_phase(1) = phase0_s;
for k = 2:Nsamp
    input_phase(k) = input_phase(k-1) + Ts * input_w(k-1);
end

%% --------------------------------------------------------------------------
%% моделирование работы фапч
%% ошибка по фазе, выход фильтра и фаза NCO
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

for n = 2:Nsamp
    % вычисление ошибки
    phase_diff = input_phase(n) - nco_phase(n-1);
    sawtooth_wrap = mod(phase_diff + PD_max, 2*PD_max) - PD_max;
    err(n) = Kd * sawtooth_wrap;

    % петлевой фильтр
    kp_out = Kp * err(n);
    ki_out = Ki * err(n) + ki_out_last;
    loop_filter_out(n) = kp_out + ki_out;
    ki_out_last = ki_out;

    % вычисление фазы NCO
    nco_phase(n) = nco_phase(n-1) + loop_filter_out(n); 
end

%% ----------------------------------------------------
%% вывод рассчетных значений
disp(['Natural frequency (rad/s): ', num2str(wn)])
disp(['Pull-out range (rad/s): ', num2str(delta_wpo)])
disp(['Lock-in range (rad/s): ', num2str(wL)])
disp(['Pull-in time (s): ', num2str(Tp)])

%% вывод графиков
subplot(2,1,1)
plot(Time, err); grid on;
xlabel('time'); ylabel('NCO phase')

subplot(2,1,2)
plot(Time, loop_filter_out / Ts); grid on;
xlabel('time'); ylabel('NCO frequency')