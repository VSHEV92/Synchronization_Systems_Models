%% функция для генерации QPSK сигнала с заданной расстройкой по частоте
%% и фазе и с заданным ОСШ. 
%% Возвращает вектор передаваемых бит и вектор отсчетов сигнала

function [tx_bits, tx_samples] = qpsk_generator(...
    sample_time,...            % шаг дискретизации
    symbols_number,...         % число передаваемых символов
    samples_ber_symbol,...     % число отсчетов на символ
    freq_offset,...            % расстройка по несущей частоте (рад/c)
    phase_offset,...           % расстройка по фазе (градусы)
    SNR...                     % отtx_samplesношение сигнал/шум (Eb/N0)
)

%% Общее число отсчетов
samples_number = symbols_number * samples_ber_symbol;

%% генерация передаваемых бит и символов
tx_bits = randi([0 1], 2, symbols_number);          % матрица из передаваемых бит  
tx_symbols = bin2dec(num2str(tx_bits.')).';         % вектор символов         
tx_bits = reshape(tx_bits, 1, 2 * symbols_number);  % вектор из передаваемых бит  

%% формирование нескольких отсчетов на символ
tx_samples = repmat(tx_symbols, [samples_ber_symbol, 1]);
tx_samples = reshape(tx_samples, 1, samples_number);

%% модуляция симвлов
tx_samples = pskmod(tx_samples, 4, pi/4);

%% добавление шума
% awgn ожидает ОСШ в виде Es/N0, мы передаем Eb/N0. Так как на символ 2 бита, для перевода добавляем 3 дБ 
tx_samples = awgn(tx_samples, SNR+3);

%% добавление расстройки по частоте и фазе
tx_samples = frequencyOffset(tx_samples.', 1/sample_time, freq_offset/(2*pi)).';
tx_samples = tx_samples * exp(1i * phase_offset * 180 / pi);

end