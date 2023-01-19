%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Моделирование потерь в отношении сингнал/шум из-за неидеальной синхронизации.

%% -------------------------------------------------------------------------------------
%% функция, реализующая фапч
function [rx_samples, nco_phase] = power_2_sync (
        tx_samples,  % отсчеты сигнала 
        BL_n,        % нормированная шумовая полоса в процентах
        ksi          % коэффициент демпфирования
    )

    Nsamp = length(tx_samples); % общее число отсчетов в разах
    
    BL_n = BL_n / 100;  % нормированная шумовая полоса
    PD_max = pi;        % максимальное значение на выходе детектора
    Kd = 1;             % коэффициент усиления фазового детектора

    %% Расчет характеристик фапч 
    Kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
    Ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви

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
        %% возведение входного сигнала в квадрат
        detector_input(n) = tx_samples(n)^2;

        %% фазовый детектор
        err(n) = Kd * angle(detector_input(n) * conj(nco_value(n-1)));

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
    rx_samples = tx_samples .* exp(-1/2*i*nco_phase);
endfunction

%% --------------------------------------------------------------------------
%% параметры bpsk сигнала
sample_time = 1e-3;
symbols_number = 1e6;
samples_ber_symbol = 1;
freq_offset = 0;
phase_offset = 0;

%% параметры фапч
ksi = 0.707;    % коэффициент демпфирования 
BL_n = 1;      % нормированная шумовая полоса в процентах

%% массив ОСШ и пусты массивы для вероятности ошибки и СКО фвзы
SNR_list = 0:0.5:10;
P_err = [];
phase_mse = [];

%% основной цикл моделирования
for SNR = SNR_list
    
    disp(['SNR = ', num2str(SNR)]);

    %% генерация входного сигнала
    [tx_bits, tx_samples] = bpsk_generator(
            sample_time,            % шаг дискретизации
            symbols_number,         % число передаваемых символов
            samples_ber_symbol,     % число отсчетов на символ
            freq_offset,            % расстройка по несущей частоте (рад/c)
            phase_offset,           % расстройка по фазе (градусы)
            SNR                     % отношение сигнал/шум (Eb/N0)
        );

    %% синхронизация сигнала
    [rx_samples, nco_phase] = power_2_sync (tx_samples, BL_n, ksi);

    %% рассчет СКО фазовой ошибки в градусах
    phase_mse = [phase_mse sqrt(var(nco_phase)) / pi * 180];

    %% демодуляция и подсчет вероятности ошибки
    rx_bits = real(rx_samples) > 0;
    P_err = [P_err sum(tx_bits ~= rx_bits)/symbols_number]; 
endfor


figure(1)
subplot(1,2,1)
semilogy(SNR_list, P_err); grid on; hold on;

%% теоретическая вероятность ошибки
SNR = 0:0.1:10;
SNR_lin = 10.^(SNR/10);
P_err_gold = 0.5 * erfc(sqrt(SNR_lin));
semilogy(SNR, P_err_gold);
xlabel('SNR (dB)'); ylabel('Error probability')
legend('Experimental BER', 'Theoretical BER'); title('BER Test')

subplot(1,2,2)
plot(SNR_list, phase_mse); grid on;
xlabel('SNR (dB)'); ylabel('NCO phase (degrees)'); title('NCO phase MSE')

axes('Visible','off');
title(['Normalized noise bandwidth ', num2str(BL_n), '%'])