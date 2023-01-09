%% https://github.com/VSHEV92/Synchronization_Systems_Models
clc; clear; close all

%% Зависимость коэффициентов пропорционально-интегрирующего петлевого фильтра от шумовой полосы.
%% Полоса нормирована к частоте дискретизации.
%% Усиление фазового детектора и NCO равны 1.

%% Литература: Gardner Phaselock Technique p.11


%% Зависимость от нормированной шумовой полосы
ksi = sqrt(2);                              % декремент затухания 
BL_n = 0.001:0.001:0.2;                     % нормированная шумовая полоса

kp = 4*ksi*BL_n / (ksi + 0.25/ksi);         % усиление пропорциональной ветви
ki = 4*BL_n.^2 / (ksi + 0.25/ksi)^2;        % усиление интегрирующей ветви

figure(1)
subplot(2,1,1)
plot(BL_n, kp); grid on;
xlabel('BL_n'); ylabel('kp')
title('ksi is equal sqrt(2)')

subplot(2,1,2)
plot(BL_n, ki); grid on;
xlabel('BL_n'); ylabel('ki')


%% Зависимость от декремента затухания
ksi = 0.1:0.01:3;               % декремент затухания 
BL_n = 0.1;                     % нормированная шумовая полоса

kp = 4*ksi*BL_n ./ (ksi + 0.25./ksi);         % усиление пропорциональной ветви
ki = 4*BL_n^2 ./ (ksi + 0.25./ksi).^2;      % усиление интегрирующей ветви

figure(2)
subplot(2,1,1)
plot(ksi, kp); grid on;
xlabel('ksi'); ylabel('kp')
title('BL_n is equal 0.1')

%% максимальное значение ki при знаении декркмента, равного единице
subplot(2,1,2)
plot(ksi, ki); grid on;
xlabel('ksi'); ylabel('ki')