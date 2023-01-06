%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all; 

%% AЧX и ФЧХ фапч с пропорционально-интегрирующим петлевым фильтром.
%% Частота нормирована к резонансной частоте фильтра.

%% Литература: Gardner Phaselock Technique

ksi_list = 0.5:0.5:3;              % декремент затухания 
w_n = 0.1:0.1:100;                 % нормированная частота

for ksi = ksi_list
ksi
    % КЧХ фапч
    H = (2*ksi*(i*w_n) + 1) ./ ((i*w_n).^2 + 2*ksi*(i*w_n) + 1);

    %% При значении декремента, меньше единицы, появляется горб на резонансной частоте.
    %% При уменьшении декремента горб растет.
    subplot(2,1,1); hold on;
    semilogx(w_n, 20*log10((abs(H)))); grid on;
    xlabel('w_n'); ylabel('Amplitude response')
    title('ksi from 0.5 to 3 step 0.5')

    %% Передаточная функция имеет одни ноль и два полюса, поэтому фаза стремится к 90 градусам
    %% при увеличении частоты
    subplot(2,1,2); hold on;
    semilogx(w_n, angle(H)/pi*180); grid on;
    xlabel('w_n'); ylabel('Phase response')
    title('ksi from 0.5 to 3 step 0.5')
endfor    