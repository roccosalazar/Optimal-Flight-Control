function v_wind = noise_generator()
    % Parametri della distribuzione Gamma
    shape = 2;       % Parametro di forma k (shape)
    scale = 0.4;     % Parametro di scala theta (scale)

    % Generazione del valore del vento
    v_wind = gamrnd(shape, scale);
end
