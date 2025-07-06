function [v_cmd, w_cmd, current_state] = stateManager(ranges, current_state, main_loop_index)
    % Maquina de estados para navegacion libre de choques
    % Estados: 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'BACKUP'
    
    % Parametros de configuracion
    min_distance_threshold = 0.8;  % Distancia minima para detectar obstaculo
    critical_distance = 0.4;       % Distancia critica para retroceder
    forward_velocity = 0.12;       % Velocidad de avance
    turn_velocity = 0.4;          % Velocidad angular de giro
    backup_velocity = -0.08;      % Velocidad de retroceso
    
    v_max = 0.15;
    w_max= 0.5;
    
    % Inicializar estado si es necesario
    if isempty(current_state)
        current_state = 'FORWARD';
    end
    
    % Analizar obstaculos en diferentes sectores del lidar
    [front_obstacle, left_obstacle, right_obstacle, critical_front] = analyzeObstacles(ranges, min_distance_threshold, critical_distance);
    
    % Calcular espacio disponible en cada lado (considerando inf como espacio maximo)
    space_left = calculateSpaceLeft(ranges);
    space_right = calculateSpaceRight(ranges);
    
    % Logica de la maquina de estados
    switch current_state
        case 'FORWARD'
            if critical_front
                current_state = 'BACKUP';
                v_cmd = backup_velocity;
                w_cmd = 0;
            elseif front_obstacle
                % Decidir direccion de giro basado en el lado con mas espacio
                if space_left > space_right
                    current_state = 'TURN_LEFT';
                    v_cmd = 0;
                    w_cmd = turn_velocity;
                else
                    current_state = 'TURN_RIGHT';
                    v_cmd = 0;
                    w_cmd = -turn_velocity;
                end
            else
                % Camino libre, avanzar
                v_cmd = forward_velocity;
                w_cmd = 0;
            end
            
        case 'TURN_LEFT'
            if ~front_obstacle
                current_state = 'FORWARD';
                v_cmd = forward_velocity;
                w_cmd = 0;
            else
                v_cmd = 0;
                w_cmd = turn_velocity;
            end
            
        case 'TURN_RIGHT'
            if ~front_obstacle
                current_state = 'FORWARD';
                v_cmd = forward_velocity;
                w_cmd = 0;
            else
                v_cmd = 0;
                w_cmd = -turn_velocity;
            end
            
        case 'BACKUP'
            if ~critical_front
                % Despues de retroceder, girar hacia el lado con mas espacio
                if space_left > space_right
                    current_state = 'TURN_LEFT';
                    v_cmd = 0;
                    w_cmd = turn_velocity;
                else
                    current_state = 'TURN_RIGHT';
                    v_cmd = 0;
                    w_cmd = -turn_velocity;
                end
            else
                v_cmd = backup_velocity;
                w_cmd = 0;
            end
            
        otherwise
            % Estado por defecto
            current_state = 'FORWARD';
            v_cmd = forward_velocity;
            w_cmd = 0;
    end
    
    % Limitar velocidades dentro de los rangos permitidos
    v_cmd = max(-v_max, min(v_max, v_cmd));
    w_cmd = max(-w_max, min(w_max, w_cmd));
end 