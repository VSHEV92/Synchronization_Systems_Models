%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all

%% Пропорционально-интегрирующий петлевой фильтр.
%% Зависимость полосы фапч по уровню 3 дБ, нормированной к резонансной частоте, от значения декркмента затухания.
%% Полоса пропускания растет с увеличение декремента.

%% Литература: Gardner Phaselock Technique p.14

ksi = 0.1:0.1:5;                                                % декремент затухания
w_3dB_n = sqrt(1 + 2*ksi.^2 + sqrt((2*ksi.^2 + 1).^2 + 1));     % полоса фапч по уровню 3 дБ, поделенная на резонансную частоту

plot(ksi, w_3dB_n); grid on;
xlabel('ksi'); ylabel('w_3dB/wn')
