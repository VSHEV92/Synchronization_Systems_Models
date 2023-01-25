%% функция для генерации BPSK сигнала с заданной расстройкой по частоте
%% и фазе и с заданным ОСШ. 
%% Возвращает вектор передаваемых бит и вектор отсчетов сигнала

function [tx_bits, tx_samples] = bpsk_generator(...
    sample_time,...            % шаг дискретизации
    symbols_number,...         % число передаваемых символов
    samples_ber_symbol,...     % число отсчетов на символ
    freq_offset,...            % расстройка по несущей частоте (рад/c)
    phase_offset,...           % расстройка по фазе (градусы)
    SNR...                     % отношение сигнал/шум (Eb/N0)
)

%% Общее число отсчетов
samples_number = symbols_number * samples_ber_symbol;

%% генерация передаваемых бит
tx_bits = rand(1,symbols_number);
tx_bits = floor(tx_bits*2);

%% генерация символов
tx_samples = repmat(tx_bits, samples_ber_symbol, 1);
tx_samples = tx_samples(:).';
tx_samples = tx_samples*2 - 1;

%% расчет дисперсии шума
symbol_amplitude = 1;
SNR_lin = 10^(SNR/10);
Eb = symbol_amplitude^2 * samples_ber_symbol * sample_time;
N0 = Eb / SNR_lin ;
noise_power = N0 / sample_time;

%% добавление шума
noise = sqrt(noise_power/2)*(randn(1, samples_number) + 1i*randn(1,samples_number));
tx_samples = tx_samples + noise;

%% добавление расстройки по частоте и фазе
phase = (1:samples_number) * sample_time;
phase = phase * freq_offset + (phase_offset/180*pi);
tx_samples = tx_samples .* exp(1i * phase);

end