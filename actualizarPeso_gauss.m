function particles = actualizarPeso_gauss(particles, lidar, ranges, map)
%ACTUALIZARPESO_GAUSS  Actualiza y normaliza el peso de cada partícula usando un modelo Gaussiano
%  particles: 4×N [x; y; θ; weight]
%  lidar:      handle a función lidar([x; y; θ]) → lecturas predichas (Mx1)
%  ranges:     lecturas reales del robot (Mx1), puede contener NaN
%  map:        robotics.OccupancyGrid object

    N = size(particles, 2);
    weights = zeros(1, N);
    
    % Parámetros más robustos
    sigma = 0.4;  % Aumentado de 0.2 a 0.4 para ser menos sensible
    min_weight = 1e-6;  % Peso mínimo para evitar underflow
    
    % Obtengo límites verdaderos del mapa
    xL = map.XWorldLimits;  % [xmin xmax]
    yL = map.YWorldLimits;  % [ymin ymax]
    
    for i = 1:N
        x  = particles(1,i);
        y  = particles(2,i);
        th = particles(3,i);
        
        % 1) Fuera de límites → regenero y peso muy bajo
        if x < xL(1) || x > xL(2) || y < yL(1) || y > yL(2)
            particles(:,i) = generar_particula(map);
            weights(i)     = min_weight;
            continue;
        end
        
        % 2) Dentro de límites: compruebo ocupancia
        if map.getOccupancy([x, y]) > 0.1
            particles(:,i) = generar_particula(map);
            weights(i)     = min_weight;
            continue;
        end
        
        % 3) Lecturas predichas y real → verosimilitud Gaussiana
        z_hat = double(lidar([x; y; th]));  % M×1
        valid = ~isnan(ranges) & ~isnan(z_hat);
        K = nnz(valid);
        
        if K == 0
            weights(i) = min_weight;  % si no hay datos válidos
        else
            d = ranges(valid) - z_hat(valid);
            
            % Modelo más robusto: usar solo un subconjunto de las mediciones
            % para evitar que demasiadas mediciones hagan el peso extremadamente pequeño
            max_measurements = min(K, 15);  % Limitar a máximo 15 mediciones
            if K > max_measurements
                % Seleccionar mediciones distribuidas uniformemente
                idx = round(linspace(1, K, max_measurements));
                d = d(idx);
                K = max_measurements;
            end
            
            % Cálculo en log-space para evitar underflow numérico
            log_coef = -K * log(sqrt(2*pi)*sigma);
            log_expo = -sum(d.^2)/(2*sigma^2);
            log_weight = log_coef + log_expo;
            
            % Evitar underflow y aplicar peso mínimo
            if log_weight < log(min_weight)
                weights(i) = min_weight;
            else
                weights(i) = exp(log_weight);
            end
        end
    end
    
    % 4) Normalizo pesos para resampling con mejor estabilidad
    weights = max(weights, min_weight);  % Asegurar peso mínimo
    weights = weights / sum(weights);
    particles(4, :) = weights;
end
