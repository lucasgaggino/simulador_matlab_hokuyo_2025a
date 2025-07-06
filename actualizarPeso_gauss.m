function particles = actualizarPeso_gauss(particles, lidar, ranges, map)
%ACTUALIZARPESO_GAUSS  Actualiza y normaliza el peso de cada part�cula usando un modelo Gaussiano
%  particles: 4�N [x; y; ?; weight]
%  lidar:      handle a funci�n lidar([x; y; ?]) ? lecturas predichas (Mx1)
%  ranges:     lecturas reales del robot (Mx1), puede contener NaN
%  map:        robotics.OccupancyGrid object

    N = size(particles, 2);
    weights = zeros(1, N);
    sigma = 0.2;  % desviaci�n t�pica del sensor [m]
    
    % Obtengo l�mites verdaderos del mapa
    xL = map.XWorldLimits;  % [xmin xmax]
    yL = map.YWorldLimits;  % [ymin ymax]
    
    for i = 1:N
        x  = particles(1,i);
        y  = particles(2,i);
        th = particles(3,i);
        
        % 1) Fuera de l�mites ? regenero y peso muy bajo
        if x < xL(1) || x > xL(2) || y < yL(1) || y > yL(2)
            particles(:,i) = generar_particula(map);
            weights(i)     = 1e-3;
            continue;
        end
        
        % 2) Dentro de l�mites: compruebo ocupancia
        if map.getOccupancy([x, y]) > 0.1
            particles(:,i) = generar_particula(map);
            weights(i)     = 1e-3;
            continue;
        end
        
        % 3) Lecturas predichas y real ? verosimilitud Gaussiana
        z_hat = double(lidar([x; y; th]));  % M�1
        valid = ~isnan(ranges) & ~isnan(z_hat);
        K = nnz(valid);
        if K == 0
            weights(i) = eps;  % si no hay datos v�lidos
        else
            d = ranges(valid) - z_hat(valid);
            % C�lculo en log-space para evitar underflow num�rico
            log_coef = -K * log(sqrt(2*pi)*sigma);
            log_expo = -sum(d.^2)/(2*sigma^2);
            log_weight = log_coef + log_expo;
            weights(i) = exp(log_weight);
        end
    end
    
    % 4) Normalizo pesos para resampling
    weights = weights + eps;
    weights = weights / sum(weights);
    particles(4, :) = weights;
end
