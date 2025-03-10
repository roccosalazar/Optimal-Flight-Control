%% Script: setup_mpc.m
% Configura l'oggetto MPC

clc; clear; close all;

% Carica i parametri del sistema
load('data/system_parameters.mat');

%% Creazione dell'oggetto MPC
sys_min = minreal(ss(A, B, C, D, delta_t)); % Modello minimo
mpc_obj = mpc(sys_min); % Usa il modello ridotto nell’MPC

% Impostazione dell'orizzonte di predizione e controllo
mpc_obj.PredictionHorizon = 40;  % Numero di passi di predizione
mpc_obj.ControlHorizon = 4;     % Numero di passi in cui il controllo viene aggiornato

% Impostazione delle matrici di costo (tuning dell'MPC)
mpc_obj.Weights.ManipulatedVariables = [0.5 0.5];  % Penalizzazione sugli input u
mpc_obj.Weights.ManipulatedVariablesRate = [1e2 1e2];  % Penalizzazione sulla variazione di u
mpc_obj.Weights.OutputVariables = [8e5 3e6 1.5e6 2e7 0];  % Penalizzazione sugli stati (solo x, y)

% Vincoli sugli input (forza massima applicabile)
mpc_obj.MV(1).Min = -2e4;  % Forza minima su x
mpc_obj.MV(1).Max = 2e4;   % Forza massima su x
mpc_obj.MV(2).Min = -2e4;  % Forza minima su y
mpc_obj.MV(2).Max = 2.5e4;   % Forza massima su y

% Disattivazione disturbo automatico
mpc_obj.Model.Noise = 1e-6 * eye(size(C,1)); % Aggiunge un rumore molto piccolo



%% Salvataggio dell'oggetto MPC per Simulink
save('data/mpc_object.mat', 'mpc_obj');  % Salva l'oggetto in un file .mat
disp('Oggetto MPC creato e salvato in "data/mpc_object.mat".');
