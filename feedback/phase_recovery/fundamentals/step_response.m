%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% Переходная характеричтика фапч с пропорционально-интегрирующим петлевым фильтром.
%% Коэффициенты усиления фазового детектора и NCO равны 1.
%% При значении декремента меньше единице появляются колебания. Их период можно рассчитать
%% через значение шумовой полосы и резонансной частоты.

%% Литература: Gardner Phaselock Technique

Ts = 1e-4;                % шаг дискретизации
Tsim = 1;                 % время моделирования
Time = 0:Ts:Tsim;         
Nsamp = length(Time);     % число отсчетов

ksi_list = 0.3:0.2:1.5;   % декремент затухания 
BL = 10;                  % шумовая полоса
BL_n = BL * Ts;           % нормированная шумовая полоса

%% входной сигнал. ступенчатое изменение фазы на 100 градусов
indata = ones(1, Nsamp);
indata = indata * 100 / 180 * pi;

%% ошибка по фазе, выход фильтра и фаза NCO
err = zeros(1, Nsamp);
nco_phase = zeros(1, Nsamp);
loop_filter_out = zeros(1, Nsamp);

%% прошлое значение в интегрирующей ветви
ki_out_last = 0;

%% массив для сохраниения периода колебаний
Tn = [];

for ksi = ksi_list

    wn = 2 * BL / (ksi + 0.25/ksi);     % резонансная частота (рад/с)
    fn = wn / 2 / pi;                   % резонансная частота (Гц/с)
    Tn = [Tn 1/fn];                     % расчет и сохранение периода колебаний

    kp = 4*ksi*BL_n / (ksi + 0.25/ksi);        % усиление пропорциональной ветви
    ki = 4*BL_n^2 / (ksi + 0.25/ksi)^2;        % усиление интегрирующей ветви
    for n = 2:Nsamp
        % вычисление ошибки
        err(n) = indata(n) - nco_phase(n-1);

        % петлевой фильтр
        kp_out = kp * err(n);
        ki_out = ki * err(n) + ki_out_last;
        loop_filter_out(n) = kp_out + ki_out;
        ki_out_last = ki_out;

        % вычисление фазы NCO
        nco_phase(n) = nco_phase(n-1) + loop_filter_out(n); 
    endfor

    hold on;
    plot(Time, nco_phase); grid on;
    xlabel('time'); ylabel('NCO phase')
endfor

%% на легенде указан период колебаний при переходном процессе
legend(cellstr(num2str(Tn', 'Tn=%-d')))

