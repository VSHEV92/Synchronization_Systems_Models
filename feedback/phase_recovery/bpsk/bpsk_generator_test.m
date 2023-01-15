clc, clear, close all

sample_time = 0.001;
symbols_number = 100;
samples_ber_symbol = 1;
freq_offset = 0;
phase_offset = 0;
SNR = 6;   

%% -------------------------------------------------
%% формирование созвездия BPSK
[tx_bits, tx_samples] = bpsk_generator(
    sample_time,            % шаг дискретизации
    symbols_number,         % число передаваемых символов
    samples_ber_symbol,     % число отсчетов на символ
    freq_offset,            % расстройка по несущей частоте (рад/c)
    phase_offset,           % расстройка по фазе (градусы)
    SNR                     % отношение сигнал/шум (Eb/N0)
);

figure(1)
scatter(real(tx_samples), imag(tx_samples)); grid on;
axis([-2 2 -2 2]); title('BPSK Scatter')

%% --------------------------------------------------------------------
%% проверка веоятности ошибки

%% экспериментальная вероятность ошибки
sample_time = 0.001;
symbols_number = 1e6;
samples_ber_symbol = 1;
freq_offset = 0;
phase_offset = 0;

P_err = [];
SNR_list = 0:0.5:10;

for SNR = SNR_list
    % формирование данных
    [tx_bits, tx_samples] = bpsk_generator(
        sample_time,            % шаг дискретизации
        symbols_number,         % число передаваемых символов
        samples_ber_symbol,     % число отсчетов на символ
        freq_offset,            % расстройка по несущей частоте (рад/c)
        phase_offset,           % расстройка по фазе (градусы)
        SNR                     % отношение сигнал/шум (Eb/N0)
    );

    % демодуляция данных
    rx_bits = real(tx_samples) > 0;

    % сравнение данных и подсчет ошибок
    number_of_errors = sum(tx_bits ~= rx_bits);
    P_err = [P_err number_of_errors/symbols_number];
end
figure(2)
semilogy(SNR_list, P_err); grid on; hold on;

%% теоретическая вероятность ошибки
SNR = 0:0.1:10;
SNR_lin = 10.^(SNR/10);
P_err_gold = 0.5 * erfc(sqrt(SNR_lin));
semilogy(SNR, P_err_gold);
legend('Experimental BER', 'Theoretical BER'); title('BER Test')