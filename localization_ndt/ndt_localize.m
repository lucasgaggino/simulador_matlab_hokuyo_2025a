function [mu, sigma, success] = ndt_localize(ranges, scanAngles, robotPose, map)
    % NDT-based localization using pcregisterndt
    % Inputs:
    %   ranges - lidar range measurements
    %   scanAngles - lidar scan angles (in radians)
    %   robotPose - current robot pose estimate [x, y, theta]
    %   map - robotics.OccupancyGrid object
    % Outputs:
    %   mu - estimated robot pose [x, y, theta]
    %   sigma - uncertainty covariance matrix (3x3)
    %   success - boolean indicating if localization was successful
    
    % Initialize outputs
    mu = robotPose;
    sigma = eye(3) * 0.1; % Default uncertainty
    success = false;
    
    try
        % Convert lidar data to point cloud (moving point cloud)
        lidarPC = lidarToPointCloud(ranges, scanAngles, robotPose);
        
        % Check if we have enough lidar points
        if lidarPC.Count < 10
            fprintf('NDT: Not enough lidar points (%d)\n', lidarPC.Count);
            return;
        end
        
        % Convert local map to point cloud (fixed point cloud)
        mapPC = mapToPointCloud(map, robotPose, 4.0); % 4m radius
        
        % Check if we have enough map points
        if mapPC.Count < 20
            fprintf('NDT: Not enough map points (%d)\n', mapPC.Count);
            return;
        end
        
        % NDT registration parameters
        gridStep = 0.5; % Grid step size for NDT (adjust as needed)
        
        % Perform NDT registration
        [tform, ~, rmse] = pcregisterndt(lidarPC, mapPC, gridStep, ...
            'MaxIterations', 50, ...
            'Tolerance', [0.01, 0.01]); % Position and angle tolerance
        
        % Extract translation and rotation from transformation
        translation = tform.Translation;
        rotation_matrix = tform.Rotation;
        
        % Calculate angle from rotation matrix (2D case)
        theta = atan2(rotation_matrix(2,1), rotation_matrix(1,1));
        
        % The transformation gives us how to transform lidar to map
        % But we want the robot pose in the map frame
        mu = [robotPose(1) + translation(1); 
              robotPose(2) + translation(2); 
              robotPose(3) + theta];
        
        % Calculate uncertainty from RMSE
        sigma = calculateUncertainty(rmse, lidarPC.Count);
        
        % Check if the result is reasonable
        if rmse < 0.5 && abs(translation(1)) < 2.0 && abs(translation(2)) < 2.0 && abs(theta) < pi/2
            success = true;
        else
            fprintf('NDT: Registration failed - RMSE: %.3f, Translation: [%.2f, %.2f], Angle: %.2f\n', ...
                rmse, translation(1), translation(2), rad2deg(theta));
        end
        
    catch ME
        fprintf('NDT: Error during registration: %s\n', ME.message);
        success = false;
    end
end

function sigma = calculateUncertainty(rmse, numPoints)
    % Calculate uncertainty covariance matrix from RMSE
    % Simple heuristic: higher RMSE = higher uncertainty
    
    % Base uncertainty that scales with RMSE
    base_uncertainty = max(0.05, min(0.5, rmse)); % Clamp between 5cm and 50cm
    
    % Reduce uncertainty with more points
    point_factor = max(0.5, min(1.0, 50 / numPoints));
    
    % Position uncertainty
    pos_var = (base_uncertainty * point_factor)^2;
    
    % Angle uncertainty (typically higher than position)
    angle_var = (base_uncertainty * point_factor * 0.5)^2; % radians^2
    
    % Create covariance matrix
    sigma = diag([pos_var, pos_var, angle_var]);
end 