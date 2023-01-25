%% -------------------------------------------------------------------------------------
%% функция, реализующая фапч для приема bpsk с помощью схемы костаса
function [rx_samples, nco_phase] = costas_sync (...
        tx_samples,...  % отсчеты сигнала 
        BL_n,...        % нормированная шумовая полоса в процентах
        ksi...          % коэффициент демпфирования
    )

    Nsamp = length(tx_samples); % общее число отсчетов в разах
    
    BL_n = BL_n / 100;  % нормированная шумовая полоса
    Kd = 1;             % коэффициент усиления фазового детектора

    %% Расчет характеристик фапч 
    Kp = 4*ksi*BL_n / (ksi + 0.25/ksi) / Kd;        % усиление пропорциональной ветви
    Ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2 / Kd;        % усиление интегрирующей ветви

    %% ошибка по фазе, выход фильтра и фаза NCO
    detector_input = zeros(1, Nsamp);
    err = zeros(1, Nsamp);
    nco_phase = zeros(1, Nsamp);
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
        nco_value(n) = exp(1i*nco_phase(n));
    end

    %% синзронизация входного сигнала
    rx_samples = detector_input;
end