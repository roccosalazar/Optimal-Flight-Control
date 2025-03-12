function [x, u, J] = solve_lqr(A, B, Q, R, Qf, x0, N)
% Calcola il controllo ottimo a orizzonte finito
% per un sistema lineare stazionario a tempo discreto
% 
% INPUT:
% A, B   - Matrici del sistema
% Q, R   - Matrici del funzionale di costo
% Qf     - Matrice del costo finale
% x0     - Condizione iniziale
% N      - Orizzonte di tempo
%
% OUTPUT:
% x      - Traiettoria ottima degli stati
% u      - Controllo ottimo
% J      - Valore della funzione di costo ottimizzata

n = size(A, 1); % Dimensione dello stato
p = size(B, 2); % Dimensione dell’ingresso

% Inizializzazione delle variabili
x = zeros(n, N+1);
u = zeros(p, N);
x(:,1) = x0;
K = zeros(p, n, N);
P = Qf;

% Calcolo dei guadagni K(k) mediante equazione di Riccati all'indietro
for k = N:-1:1
    K(:,:,k) = (B' * P * B + R) \ (B' * P * A);
    P = A' * P * A + Q - K(:,:,k)' * B' * P * A;
end

% Inizializzazione del costo J
J = 0;

% Calcolo della traiettoria ottima e dell’ingresso ottimo
for k = 1:N
    u(:,k) = -K(:,:,k) * x(:,k);
    x(:,k+1) = A * x(:,k) + B * u(:,k);
    J = J + x(:,k)' * Q * x(:,k) + u(:,k)' * R * u(:,k);
end

% Aggiunta del termine di costo finale
J = J + x(:,N+1)' * Qf * x(:,N+1);

end
