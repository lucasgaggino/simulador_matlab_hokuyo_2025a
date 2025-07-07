function weight = weight_gauss(real_data,simulated_lidar)

    valid = ~isnan(real_data) & ~isnan(simulated_lidar);
    d = real_data(valid) - simulated_lidar(valid);
    sigma = 0.2;
    K = nnz(valid);
    % Calculo en log-space para evitar underflow numerico
    log_coef = -K * log(sqrt(2*pi)*sigma);
    log_expo = -sum(d.^2)/(2*sigma^2);
    log_weight = log_coef + log_expo;
    weight = exp(log_weight);

end
