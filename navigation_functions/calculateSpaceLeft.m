function space_left = calculateSpaceLeft(ranges)
    % Calcula la cantidad de espacio libre disponible en el lado izquierdo
    % ranges: vector de distancias del lidar
    % Inf indica espacio libre maximo, NaN se descarta completamente
    
    % Procesar mediciones: mantener Inf como valor alto, descartar NaN completamente
    valid_ranges = ranges;
    
    total_readings = length(valid_ranges);
    
    

    left_start = round(total_readings * 0.6); 
    
    % Obtener distancias del sector izquierdo
    left_distances = valid_ranges(left_start:end);
    
    % Descartar mediciones NaN y procesar solo las validas
    valid_left = left_distances(~isnan(left_distances));
    
    if isempty(valid_left)
        % Si no hay mediciones validas, asumir espacio minimo
        space_left = 0.5;
    else
        % Calcular espacio promedio considerando Inf como espacio maximo (5.0m)
        % Esto permite que los rangos infinitos contribuyan como espacio libre
        processed_distances = valid_left;
        processed_distances(isinf(processed_distances)) = 5.0; % Tratar Inf como distancia maxima del lidar
        
        space_left = mean(processed_distances);
    end
end 