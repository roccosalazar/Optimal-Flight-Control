function wind = noise_generator(Wx_max, Wy_max, p_wind)
    % Generazione del disturbo atmosferico con distribuzione di Bernoulli eUniforme
    wind = [0; 0]; % Inizializza il vettore vento [Wx; Wy]

    if rand < p_wind
        wind(1) = Wx_max * (2*rand - 1);  % Valore casuale tra -Wx_max e Wx_max
    end
    if rand < p_wind
        wind(2) = Wy_max * (2*rand - 1);  % Valore casuale tra -Wy_max e Wy_max
    end
end