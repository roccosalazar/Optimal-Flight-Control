%% Script: mpc_control.m
% Esegue il controllo MPC con disturbo atmosferico (vento)

clc; clear; close all;

%% Caricamento del sistema e del modello MPC
addpath('functions'); 
load('data/system_parameters.mat');  % Matrici del sistema
load('data/mpc_object.mat');         % Oggetto MPC
load('data/reference_trajectory.mat'); % Traiettoria di riferimento

% Numero di passi di simulazione
num_steps = N;

% Stato iniziale
x = x0;
X_hist = zeros(size(A,1), num_steps+1);
X_hist(:,1) = x;
U_hist = zeros(size(B,2), num_steps);
Wind_hist = zeros(2, num_steps);  % Salva la storia del vento

% Creazione dello stato iniziale del controllore MPC
mpc_state = mpcstate(mpc_obj);  % Inizializza lo stato MPC

% Simulazione in loop chiuso con MPC
disp('Inizio della simulazione MPC con disturbo atmosferico...');
for k = 1:num_steps
    % Stato corrente
    xk = X_hist(:,k);
    
    % Definizione dell'output desiderato (riferimento per MPC)
    y_ref = x_ref(:,min(k, size(x_ref,2)));  % Usa la traiettoria di riferimento
    
    % Generazione del vento casuale
    wind = noise_generator(Wx_max, Wy_max, p_wind);
    Wind_hist(:,k) = wind;
    
    % Risoluzione del problema di controllo MPC con stato aggiornato
    u_mpc = mpcmove(mpc_obj, mpc_state, xk, y_ref);  
    
    % Aggiornamento dello stato
    xk(1) = A(1,:) * xk + B(1,:) * u_mpc + wind(1)*delta_t; 
    xk(2) = A(2,:) * xk + B(2,:) * u_mpc + wind(2)*delta_t;
    xk(3) = A(3,:) * xk + B(3,:) * u_mpc + wind(1);  % Aggiorna v_x con vento
    xk(4) = A(4,:) * xk + B(4,:) * u_mpc + wind(2);  % Aggiorna v_y con vento
    xk(5) = A(5,:) * xk + B(5,:) * u_mpc;           % Aggiorna accelerazione
    
    % Salvataggio delle variabili per analisi
    X_hist(:,k+1) = xk;
    U_hist(:,k) = u_mpc;
end
disp('Simulazione MPC completata.');

%% Salvataggio dei risultati
save('data/mpc_results.mat', 'X_hist', 'U_hist', 'Wind_hist');
disp('Risultati MPC salvati in "data/mpc_results.mat".');

%% Visualizzazione dei risultati
figure;
subplot(2,3,2);
plot(1:num_steps+1, X_hist(1,:), 'k.-'); hold on;
xlabel('Tempo (k)'); ylabel('Posizione x'); title('Evoluzione di x'); grid on;

subplot(2,3,5);
plot(1:num_steps+1, X_hist(2,:), 'r.-'); hold on;
xlabel('Tempo (k)'); ylabel('Posizione y'); title('Evoluzione di y'); grid on;

subplot(2,3,3);
plot(1:num_steps+1, X_hist(3,:), 'b.-'); hold on;
xlabel('Tempo (k)'); ylabel('Velocità v_x'); title('Evoluzione di v_x'); grid on;

subplot(2,3,6);
plot(1:num_steps+1, X_hist(4,:), 'g.-'); hold on;
xlabel('Tempo (k)'); ylabel('Velocità v_y'); title('Evoluzione di v_y'); grid on;

subplot(2,3,4);
plot(1:num_steps, U_hist(1,:), 'b.-'); hold on;
plot(1:num_steps, U_hist(2,:), 'r.-');
xlabel('Tempo (k)'); ylabel('Forza di controllo');
title('Controllo MPC');
legend('u_x', 'u_y'); grid on;

subplot(2,3,1)
plot(X_hist(1,:), X_hist(2,:), 'k.-'); hold on;
xlabel('Posizione x'); ylabel('Posizione y');
title('Traiettoria MPC con disturbo atmosferico'); grid on;

exportgraphics(gcf, 'results/mpc_control.png', 'Resolution', 300);

figure;
subplot(2,1,1);
plot(1:num_steps, Wind_hist(1,:), 'b.-');
xlabel('Tempo (k)'); ylabel('Wx');
title('Andamento del vento Wx'); grid on;

subplot(2,1,2);
plot(1:num_steps, Wind_hist(2,:), 'r.-');
xlabel('Tempo (k)'); ylabel('Wy');
title('Andamento del vento Wy'); grid on;


exportgraphics(gcf, 'results/wind_variation.png', 'Resolution', 300);
