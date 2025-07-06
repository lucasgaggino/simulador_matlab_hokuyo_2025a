function resampled = resampleParticles(particles, map)
    N = size(particles,2);
    w = particles(4,:);
    w = w + eps;        
    w = w / sum(w);

    % 1) muestreo sistemico
    cdf = cumsum(w);
    u0  = rand/N;
    u   = u0 + (0:(N-1))/N;
    idx = arrayfun(@(ui) find(cdf>=ui,1), u);
    resampled = particles(:, idx);

    % 2) rejuvenecimiento aumentado (20%)
    alpha = 0.2;
    M     = round(alpha * N);
    for k = 1:M
        resampled(:, N-M+k) = generar_particula(map);
    end

    % 3) jitter ligero para romper clones identicos
    sigma_pos = 0.02;  sigma_ang = 0.01;
    resampled(1:2,:) = resampled(1:2,:) + sigma_pos*randn(2,N);
    resampled(3,:)   = resampled(3,:)   + sigma_ang*randn(1,N);

    % 4) pesos uniformes para la siguiente iteracion
    resampled(4,:) = 1/N;
end
