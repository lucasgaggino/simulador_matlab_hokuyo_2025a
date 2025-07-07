function weight = weight_corr(real_data, simulated_lidar)
    % Calculate weight using cross-correlation
    % Inputs:
    %   real_data - actual lidar measurements
    %   simulated_lidar - simulated lidar measurements for particle pose
    % Output:
    %   weight - correlation-based weight for the particle
    
    % Filter out NaN values
    valid = ~isnan(real_data) & ~isnan(simulated_lidar);
    real_valid = real_data(valid);
    sim_valid = simulated_lidar(valid);
    
    % Check if we have enough valid data points
    if length(real_valid) < 2
        weight = 0;
        return;
    end
    
    % Calculate normalized cross-correlation
    [correlation, ~] = xcorr(real_valid, sim_valid, 'normalized');
    
    % Use the maximum value of cross-correlation as weight
    weight = max(correlation);
    
 
end 