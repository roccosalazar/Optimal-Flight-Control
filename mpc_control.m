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
Wind_hist = zeros(1, num_steps);  % Vettore per la storia del vento (ora corretto)

% Creazione dello stato iniziale del controllore MPC
mpc_state = mpcstate(mpc_obj);  % Inizializza lo stato MPC

% Selezione casuale degli istanti in cui si verificano le raffiche
num_raffiche = 10;  % Numero di raffiche totali
raffiche_index = randperm(num_steps, num_raffiche);  % Seleziona casualmente 10 passi

% Simulazione in loop chiuso con MPC
disp('Inizio della simulazione MPC con disturbo atmosferico...');
for k = 1:num_steps
    % Stato corrente
    xk = X_hist(:,k);
    
    % Definizione dell'output desiderato (riferimento per MPC)
    y_ref = x_ref(:,min(k, size(x_ref,2)));  % Usa la traiettoria di riferimento
    
    % Generazione del vento: solo nei passi selezionati
    if ismember(k, raffiche_index)
        wind = noise_generator(); % Applica vento solo nelle raffiche
    else
        wind = 0; % Nessun vento
    end
    Wind_hist(k) = wind;  % Salva il vento nel vettore corretto
    
    % Risoluzione del problema di controllo MPC con stato aggiornato
    u_mpc = mpcmove(mpc_obj, mpc_state, xk, y_ref);  
    
    % Aggiornamento dello stato con il vento nei momenti giusti
    xk(1) = A(1,:) * xk + B(1,:) * u_mpc - wind * delta_t; 
    xk(2) = A(2,:) * xk + B(2,:) * u_mpc;
    xk(3) = A(3,:) * xk + B(3,:) * u_mpc - wind;  % Aggiorna v_x con vento
    xk(4) = A(4,:) * xk + B(4,:) * u_mpc;  % Aggiorna v_y con vento
    xk(5) = A(5,:) * xk + B(5,:) * u_mpc;  % Aggiorna accelerazione
    
    % Salvataggio delle variabili per analisi
    X_hist(:,k+1) = xk;
    U_hist(:,k) = u_mpc;
end
disp('Simulazione MPC completata.');

%% Salvataggio dei risultati
save('data/mpc_results.mat', 'X_hist', 'U_hist', 'Wind_hist');
disp('Risultati MPC salvati in "data/mpc_results.mat".');

%% Visualizzazione dei risultati
figure('Position', [100, 100, 1400, 700]); % Imposta una figura più larga

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
title('Traiettoria MPC'); grid on;

exportgraphics(gcf, 'results/mpc_control.png', 'Resolution', 300);

figure;
plot(1:num_steps, Wind_hist, 'b.-');
xlabel('Tempo (k)'); ylabel('Intensità vento');
title('Andamento del vento nelle raffiche'); grid on;

exportgraphics(gcf, 'results/wind_variation.png', 'Resolution', 300);
