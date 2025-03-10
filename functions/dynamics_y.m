function y_next = dynamics_y(y, vy, wy, delta_t, g)
    % Evoluzione della posizione y con gravità e vento
    y_next = y + vy * delta_t - 0.5 * g * delta_t^2 + wy * delta_t;
end