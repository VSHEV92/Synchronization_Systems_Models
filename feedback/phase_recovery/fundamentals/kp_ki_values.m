%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all

%% Зависимость коэффициентов пропорционально-интегрирующего петлевого фильтра от шумовой полосы.
%% Полоса нормирована к частоте дискретизации. Декремент затухания равен sqrt(2).
%% Усиление фазового детектора и NCO равны 1.

ksi = sqrt(2);                              % декремент затухания 
BL_n = 0.001:0.001:0.2;                     % нормированная шумовая полоса

kp = 4*ksi*BL_n / (ksi + 0.25/ksi);         % усиление пропорциональной ветви
ki = 4*BL_n.^2 / (ksi + 0.25/ksi)^2;      % усиление интегрирующей ветви

subplot(2,1,1)
plot(BL_n, kp); grid on;
xlabel('BL_n'); ylabel('kp')

subplot(2,1,2)
plot(BL_n, ki); grid on;
xlabel('BL_n'); ylabel('ki')