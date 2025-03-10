function x_next = dynamics_x(x, vx, wx, delta_t)
    % Evoluzione della posizione x con vento
    x_next = x + vx * delta_t + wx * delta_t;
end