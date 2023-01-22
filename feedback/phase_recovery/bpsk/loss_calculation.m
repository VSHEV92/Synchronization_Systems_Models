%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Моделирование потерь в отношении сингнал/шум из-за неидеальной синхронизации.
%% Восстновление несущей с помощью возведения в квадрат
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
    %[rx_samples, nco_phase] = power_2_sync (tx_samples, BL_n, ksi);
    %[rx_samples, nco_phase] = costas_sync (tx_samples, BL_n, ksi);
    [rx_samples, nco_phase] = remodulation_sync (tx_samples, BL_n, ksi);

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