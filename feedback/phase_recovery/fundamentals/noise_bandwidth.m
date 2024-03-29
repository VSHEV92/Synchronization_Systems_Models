%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all

%% Пропорционально-интегрирующий петлевой фильтр. 
%% Зависимость шумовой полосы, нормированной к резонансной частоте, от значения декркмента затухания.
%% Минимальное значение принимается при значении декремента, равном 0.5. Как правило декремент имеет большее
%% значение, то есть шумовая полоса растет при увеличении декремента. 

%% Литература: Gardner Phaselock Technique p.13

ksi = 0.1:0.1:4;                      % декремент затухания
BL_n = 0.5 * (ksi + 0.25 ./ ksi);     % шумовая полоса, поделенная на резонансную частоту

plot(ksi, BL_n); grid on;
xlabel('ksi'); ylabel('BL/wn')
