function [front_obstacle, left_obstacle, right_obstacle, critical_front] = analyzeObstacles(ranges, min_distance_threshold, critical_distance)
    % Analiza las mediciones del lidar para detectar obstaculos en diferentes sectores
    % ranges: vector de distancias del lidar (-90° a +90°)
    % min_distance_threshold: distancia minima para considerar un obstaculo
    % critical_distance: distancia critica que requiere accion inmediata
    
    % Procesar mediciones: descartar NaN, mantener Inf como espacio libre
    valid_ranges = ranges;
    % NaN se descarta completamente (no se cambia por ningun valor)
    
    % Definir sectores del lidar (indices aproximados para 171 mediciones)
    total_readings = length(valid_ranges);
    
    % Sector frontal (30Â° hacia cada lado del centro)
    front_angle_range = 30; % grados
    front_sector_width = round((front_angle_range * 2) * total_readings / 180);
    center_idx = round(total_readings / 2);
    front_start = max(1, center_idx - round(front_sector_width / 2));
    front_end = min(total_readings, center_idx + round(front_sector_width / 2));
    
    % Sector derecho (de -90 a -30)
    right_start = 1;
    right_end = round(total_readings / 3);
    
    % Sector izquierdo (de -90  a -30 )
    left_start = round(2 * total_readings / 3);
    left_end = total_readings;
    
    % Analizar obstaculos en cada sector
    front_distances = valid_ranges(front_start:front_end);
    left_distances = valid_ranges(left_start:left_end);
    right_distances = valid_ranges(right_start:right_end);

    % Detectar obstaculos (considerar obstaculo si al menos 30% de las mediciones VALIDAS estan cerca)
    % NaN se descarta, Inf se considera como espacio libre (distancia muy grande)
    %obstacle_percentage_threshold = 0.3* rand()*0.2+1; % opcion para introducir moviemientos aleartoreos
    obstacle_percentage_threshold = 0.3;
    
    
    % Solo contar mediciones validas (no NaN) para calcular porcentajes
    front_valid = ~isnan(front_distances);
    front_close_readings = sum(front_distances < min_distance_threshold);
    front_total_valid = sum(front_valid);
    front_obstacle = front_total_valid > 0 && (front_close_readings / front_total_valid) > obstacle_percentage_threshold;
    
    left_valid = ~isnan(left_distances);
    left_close_readings = sum(left_distances < min_distance_threshold );
    left_total_valid = sum(left_valid);
    left_obstacle = left_total_valid > 0 && (left_close_readings / left_total_valid) > obstacle_percentage_threshold;
    
    right_valid = ~isnan(right_distances);
    right_close_readings = sum(right_distances < min_distance_threshold);
    right_total_valid = sum(right_valid);
    right_obstacle = right_total_valid > 0 && (right_close_readings / right_total_valid) > obstacle_percentage_threshold;
    
    % Detectar obstaculo critico en el frente (muy cerca)
    critical_front_readings = sum(front_distances < critical_distance);
    critical_front = front_total_valid > 0 && (critical_front_readings / front_total_valid) > 0.2; % 20% de mediciones criticas
end 