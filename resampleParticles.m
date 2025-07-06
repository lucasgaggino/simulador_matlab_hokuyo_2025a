function resampled = resampleParticles(particles, map)
    N = size(particles,2);
    w = particles(4,:);
    w = w + eps;        
    w = w / sum(w);

    % Calcular Neff (número efectivo de partículas)
    Neff = 1 / sum(w.^2);
    threshold = N * 0.5;  % Resamplear solo si Neff < 50% de N
    
    if Neff > threshold
        % No resamplear, solo añadir jitter ligero
        resampled = particles;
        % Jitter más agresivo para mantener diversidad
        sigma_pos = 0.05;  % Aumentado de 0.02 a 0.05
        sigma_ang = 0.03;  % Aumentado de 0.01 a 0.03
        resampled(1:2,:) = resampled(1:2,:) + sigma_pos*randn(2,N);
        resampled(3,:)   = resampled(3,:)   + sigma_ang*randn(1,N);
        return;
    end

    % 1) muestreo sistemático
    cdf = cumsum(w);
    u0  = rand/N;
    u   = u0 + (0:(N-1))/N;
    idx = arrayfun(@(ui) find(cdf>=ui,1), u);
    resampled = particles(:, idx);

    % 2) rejuvenecimiento reducido (solo 5-10% en lugar de 20%)
    alpha = 0.05;  % Reducido de 0.2 a 0.05
    M     = round(alpha * N);
    if M > 0
        for k = 1:M
            resampled(:, N-M+k) = generar_particula(map);
        end
    end

    % 3) jitter más agresivo para romper clones idénticos
    sigma_pos = 0.05;  % Aumentado de 0.02 a 0.05
    sigma_ang = 0.03;  % Aumentado de 0.01 a 0.03
    resampled(1:2,:) = resampled(1:2,:) + sigma_pos*randn(2,N);
    resampled(3,:)   = resampled(3,:)   + sigma_ang*randn(1,N);

    % 4) pesos uniformes para la siguiente iteración
    resampled(4,:) = 1/N;
end
