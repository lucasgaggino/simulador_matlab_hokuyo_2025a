function [particula] = generar_particula(map)
%GENERAR_PARTICULA Genera una partícula aleatoriamente ubicada en el mapa
%   La función debe generar una partícula aleatoriamente ubicada en el mapa
%   en una posición válida (no ocupada)

    % Límites del mapa
    xLim = map.XWorldLimits;
    yLim = map.YWorldLimits;
    
    % Intentar generar partícula válida con límite de intentos
    max_attempts = 100;
    attempt = 0;
    
    while attempt < max_attempts
        % Generar posición aleatoria dentro de los límites
        x = xLim(1) + rand() * (xLim(2) - xLim(1));
        y = yLim(1) + rand() * (yLim(2) - yLim(1));
        theta = rand() * 2 * pi;
        
        % Verificar si la posición está libre (ocupación < 0.5)
        if map.getOccupancy([x, y]) < 0.5
            particula = [x; y; theta; 0];
            return;
        end
        
        attempt = attempt + 1;
    end
    
    % Si no se encontró posición válida después de max_attempts,
    % usar la última generada (fallback)
    warning('No se pudo generar partícula válida después de %d intentos', max_attempts);
    particula = [x; y; theta; 0];
end

