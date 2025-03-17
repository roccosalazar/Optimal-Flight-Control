function v_wind = noise_generator()
    % Parametri della distribuzione Weibull (stimati)
    k = 1.8;      % Parametro di forma
    lambda_w = 20;  % Parametro di scala (rinominato per evitare conflitti)

    % Generazione del valore del vento secondo Weibull
    v_wind = wblrnd(lambda_w, k);
end