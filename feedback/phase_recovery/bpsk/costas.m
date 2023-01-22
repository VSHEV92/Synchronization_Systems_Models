%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Схема Костаса для приема bpsk сигнала. 
%% На самом деле это не совсем схема Костаса. В оригинальной схеме действительный сигнал
%% умножался на квадратурный гармонический сигнал. После умножителей стояли ФНЧ для удаления
%% компонентов нв удвоенной частоте.
%% Здесь входной сигнал комплексный, поэтому компоненты на удвоенной частоте отсутствуют.
%% Поэтому в схеме также нет ФНЧ. Характеристика фазового детектора: 0.5*sin(2*delta_phi).

%% --------------------------------------------------------------------------
%% параметры bpsk сигнала
sample_time = 1e-3;
symbols_number = 1e4;
samples_ber_symbol = 1;
freq_offset = 50;
phase_offset = 30;
SNR = 10;

Nsamp = samples_ber_symbol * symbols_number;    % общее число отсчетов
Time = (0:Nsamp-1) * sample_time;               % массив с отчетами времени

%% параметры фапч
ksi = 0.707;    % коэффициент демпфирования 
BL = 5;         % шумовая полоса (Гц)
Kd = 1;         % коэффициент усиления фазового детектора

%% --------------------------------------------------------------------------
%% Расчет характеристик фапч 
BL_n = BL * sample_time;                        % нормированная шумовая полоса
Kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
Ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви

wn = 2 * BL / (ksi + 0.25/ksi);                     % резонансная частота (рад/с)
delta_wpo = 1.8 * wn * (1 + ksi);                   % ожидаемый pull-out range (рад/с)
wL = Kd * (Kp / sample_time);                       % lock-in range (рад/c) 
Tp = (2*freq_offset)^2 / (2*ksi*wn^3);              % pull-in time 

%% --------------------------------------------------------------------------
%% генерация входного сигнала
[tx_bits, tx_samples] = bpsk_generator(
        sample_time,            % шаг дискретизации
        symbols_number,         % число передаваемых символов
        samples_ber_symbol,     % число отсчетов на символ
        freq_offset,            % расстройка по несущей частоте (рад/c)
        phase_offset,           % расстройка по фазе (градусы)
        SNR                     % отношение сигнал/шум (Eb/N0)
    );

%% --------------------------------------------------------------------------
%% моделирование работы фапч
%% ошибка по фазе, выход фильтра и фаза NCO
detector_input = zeros(1, Nsamp);
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
nco_value = ones(1, Nsamp);
nco_value = ones(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

for n = 2:Nsamp
    %% умножение входного сигнала на согнал NCO
    detector_input(n) = tx_samples(n) * conj(nco_value(n-1));

    %% фазовый детектор
    err(n) = real(detector_input(n)) * imag(detector_input(n));

    % петлевой фильтр
    kp_out = Kp * err(n);
    ki_out = Ki * err(n) + ki_out_last;
    loop_filter_out(n) = kp_out + ki_out;
    ki_out_last = ki_out;

    % вычисление фазы NCO
    nco_phase(n) = nco_phase(n-1) + loop_filter_out(n);
    nco_value(n) = exp(1*i*nco_phase(n));
endfor

%% синзронизация входного сигнала
rx_samples = detector_input;

%% ----------------------------------------------------
%% вывод рассчетных значений
disp(['Natural frequency (rad/s): ', num2str(wn)])
disp(['Pull-out range (rad/s): ', num2str(delta_wpo)])
disp(['Lock-in range (rad/s): ', num2str(wL)])
disp(['Pull-in time (s): ', num2str(Tp)])
disp(['Pull-in time (bpsk symbols): ', num2str(Tp/(samples_ber_symbol * sample_time))])

%% вывод графиков
figure(1)
scatter(real(rx_samples), imag(rx_samples)); grid on;
axis([-2 2 -2 2]); title('BPSK Scatter')

figure(2)
subplot(2,1,1)
plot(Time, loop_filter_out/sample_time - freq_offset); grid on;
xlabel('time (sec)'); ylabel('frequency error')

subplot(2,1,2)
plot(Time, err); grid on;
xlabel('time (sec)'); ylabel('phase error')