clc; clear; close all;

%% Definizione dei parametri del sistema

delta_t = 0.1;   % Passo temporale (s)
N = 1000;         % Orizzonte temporale (numero di passi)
g = 9.81;        % Accelerazione gravitazionale (m/s^2)
m = 1000;        % Massa del sistema (kg)
beta = 40;       % Coefficiente di attrito

% Matrici del modello dinamico discreto x[k+1] = A*x[k] + B*u[k]
A = [1, 0, delta_t, 0, 0;
     0, 1, 0, delta_t, delta_t^2/2;
     0, 0, 1 - beta*delta_t/m, 0, 0;
     0, 0, 0, 1, delta_t;
     0, 0, 0, 0, 1];

B = [delta_t^2/(2*m), 0;
     0, delta_t^2/(2*m);
     delta_t/m, 0;
     0, delta_t/m;
     0, 0];

C = eye(5);   % Osserviamo tutti gli stati
D = zeros(5,2);  % Nessun legame diretto tra input e output

% Stato iniziale
x0 = [-9000; 5000; 277; 0; -g];

% Parametri dei disturbi
Wx_max = 15;      % Massima raffica longitudinale [m/s]
Wy_max = 15;      % Massima raffica trasversale [m/s]
p_wind = 0.3;          % Probabilità di una raffica (30% del tempo)

% Salvataggio delle variabili per l'uso in altri script
save('data/system_parameters.mat', 'delta_t', 'N', 'g', 'm', 'beta', 'A', 'B', 'C', 'D', 'x0', 'Wx_max', 'Wy_max', 'p_wind');
disp('Parametri del sistema salvati in "data/system_parameters.mat".');

%% Definizione della traiettoria ottima tramite Linear Quadratic Regulator (LQR)
addpath('functions');  % Se la cartella è nella stessa directory dello script principale
% Definizione delle matrici di costo per il controllo LQR
Q = eye(5);       % Penalizzazione sugli stati (nessuna penalizzazione intermedia)
R = eye(2);         % Penalizzazione sul controllo (identità 2x2)
Qf = 1e10 * eye(5);  % Penalizzazione sullo stato finale (può essere modificata)

% Esecuzione del controllo ottimo LQR a orizzonte finito
[x, u, J] = solve_lqr(A, B, Q, R, Qf, x0, N);

% Salvataggio della traiettoria ottima per l'MPC
x_ref = x; % Memorizza la traiettoria come riferimento per l'MPC
save('data/reference_trajectory.mat', 'x_ref'); % Salva il file per uso futuro
disp('Traiettoria di riferimento salvata come data/reference_trajectory.mat.');

% Plot della traiettoria ottima
figure;

% Posizione x nel tempo
subplot(2,3,2);
plot(1:N+1, x(1,:), 'k.-'); hold on;
xlabel('Tempo (k)');
ylabel('Posizione x');
title('Evoluzione di x');
grid on;

% Posizione y nel tempo
subplot(2,3,5);
plot(1:N+1, x(2,:), 'r.-'); hold on;
xlabel('Tempo (k)');
ylabel('Posizione y');
title('Evoluzione di y');
grid on;

% Velocità x nel tempo
subplot(2,3,3);
plot(1:N+1, x(3,:), 'b.-'); hold on;
xlabel('Tempo (k)');
ylabel('Velocità v_x');
title('Evoluzione di v_x');
grid on;

% Velocità y nel tempo
subplot(2,3,6);
plot(1:N+1, x(4,:), 'g.-'); hold on;
xlabel('Tempo (k)');
ylabel('Velocità v_y');
title('Evoluzione di v_y');
grid on;

% Controllo ottimo u_x e u_y
subplot(2,3,4);
plot(1:N, u(1,:), 'b.-'); hold on;
plot(1:N, u(2,:), 'r.-');
xlabel('Tempo (k)');
ylabel('Forza di controllo');
title('Forza di controllo ottima');
legend('u_x', 'u_y');
grid on;

% Traiettoria y in funzione di x
subplot(2,3,1)
plot(x(1,:), x(2,:), 'k.-'); hold on;
xlabel('Posizione x');
ylabel('Posizione y');
title('Traiettoria y in funzione di x');
grid on;

exportgraphics(gcf, 'results/lqr_control.png', 'Resolution', 300);

disp('Simulazione completata.');
