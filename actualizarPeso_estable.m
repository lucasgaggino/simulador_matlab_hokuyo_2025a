function particles = actualizarPeso_estable(particles, lidar, ranges, map)
    N     = size(particles,2);
    logw  = zeros(1, N);
    sigma = 0.2;                % desviación típica del sensor [m]

    for i = 1:N
        x   = particles(1,i);
        y   = particles(2,i);
        th  = particles(3,i);
        zhat = double(lidar([x; y; th]));

        % índices válidos
        valid = ~isnan(ranges) & ~isnan(zhat);
        d     = ranges(valid) - zhat(valid);
        K     = nnz(valid);

        if K == 0
            logw(i) = -Inf;   % sin datos válidos
        else
            % suma de cuadrados de errores
            S = sum(d.^2);
            % log peso no normalizado
            logw(i) = - S/(2*sigma^2) ...
                      - K*log(sqrt(2*pi)*sigma);
        end
    end

    % estabilizar y normalizar (log-sum-exp)
    M   = max(logw);
    u   = exp(logw - M);     % evita underflow
    w   = u / sum(u);        % pesos normalizados
    w(isnan(w)) = 1/N;       % en caso de todo cero

    % escribir nuevos pesos
    particles(4,:) = w;
end