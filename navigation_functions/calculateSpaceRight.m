function space_right = calculateSpaceRight(ranges)
    % Calcula la cantidad de espacio libre disponible en el lado derecho
    % ranges: vector de distancias del lidar
    % Inf indica espacio libre maximo, NaN se descarta completamente
    
    % Procesar mediciones: mantener Inf como valor alto, descartar NaN completamente
    valid_ranges = ranges;
    
    total_readings = length(valid_ranges);
    
    % Sector derecho (aproximadamente de +15° a +90°)
    right_end = round(total_readings * 0.4); % 60% del rango total

    
    % Obtener distancias del sector derecho
    right_distances = valid_ranges(1:right_end);
    
    % Descartar mediciones NaN y procesar solo las validas
    valid_right = right_distances(~isnan(right_distances));
    
    if isempty(valid_right)
        % Si no hay mediciones validas, asumir espacio minimo
        space_right = 0.5;
    else
        % Calcular espacio promedio considerando Inf como espacio maximo (5.0m)
        % Esto permite que los rangos infinitos contribuyan como espacio libre
        processed_distances = valid_right;
        processed_distances(isinf(processed_distances)) = 5.0; % Tratar Inf como distancia maxima del lidar
        
        space_right = mean(processed_distances);
    end
end 