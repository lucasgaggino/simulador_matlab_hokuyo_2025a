function visualize_ndt_result(mu, sigma, robotPose, map, ranges, scanAngles)
    % Visualize NDT localization result
    % Inputs:
    %   mu - estimated robot pose [x, y, theta]
    %   sigma - uncertainty covariance matrix (3x3)
    %   robotPose - original robot pose estimate [x, y, theta]
    %   map - robotics.OccupancyGrid object
    %   ranges - lidar range measurements
    %   scanAngles - lidar scan angles
    
    figure;
    
    % Plot the map
    subplot(1,2,1);
    show(map);
    hold on;
    
    % Plot original pose
    plot(robotPose(1), robotPose(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Plot NDT estimated pose
    plot(mu(1), mu(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    
    % Plot uncertainty ellipse (2D)
    if size(sigma, 1) >= 2
        % Extract position covariance
        pos_cov = sigma(1:2, 1:2);
        
        % Calculate ellipse parameters
        [V, D] = eig(pos_cov);
        angle = atan2(V(2,1), V(1,1));
        
        % 95% confidence ellipse (2.45 sigma)
        ellipse_scale = 2.45;
        a = ellipse_scale * sqrt(D(1,1));
        b = ellipse_scale * sqrt(D(2,2));
        
        % Generate ellipse points
        theta = linspace(0, 2*pi, 100);
        ellipse_x = a * cos(theta);
        ellipse_y = b * sin(theta);
        
        % Rotate ellipse
        rotated_ellipse = [cos(angle) -sin(angle); sin(angle) cos(angle)] * [ellipse_x; ellipse_y];
        
        % Translate to robot position
        plot(mu(1) + rotated_ellipse(1,:), mu(2) + rotated_ellipse(2,:), 'g--', 'LineWidth', 1.5);
    end
    
    % Plot orientation arrows
    arrow_length = 0.3;
    
    % Original orientation
    plot([robotPose(1), robotPose(1) + arrow_length*cos(robotPose(3))], ...
         [robotPose(2), robotPose(2) + arrow_length*sin(robotPose(3))], 'r-', 'LineWidth', 3);
    
    % Estimated orientation
    plot([mu(1), mu(1) + arrow_length*cos(mu(3))], ...
         [mu(2), mu(2) + arrow_length*sin(mu(3))], 'g-', 'LineWidth', 3);
    
    title('NDT Localization Result');
    legend('Original Pose', 'NDT Estimate', 'Uncertainty (95%)', 'Original Orient.', 'NDT Orient.', 'Location', 'best');
    axis equal;
    hold off;
    
    % Plot point clouds
    subplot(1,2,2);
    
    % Convert data to point clouds for visualization
    lidarPC = lidarToPointCloud(ranges, scanAngles, robotPose);
    mapPC = mapToPointCloud(map, robotPose, 4.0);
    
    if lidarPC.Count > 0 && mapPC.Count > 0
        % Plot map points
        pcshow(mapPC.Location, 'b.', 'MarkerSize', 20);
        hold on;
        
        % Plot lidar points
        pcshow(lidarPC.Location, 'r.', 'MarkerSize', 30);
        
        title('Point Clouds for NDT Registration');
        legend('Map Points', 'Lidar Points', 'Location', 'best');
        view(2); % 2D view
        axis equal;
        hold off;
    else
        text(0.5, 0.5, 'No point cloud data available', 'HorizontalAlignment', 'center');
        title('Point Clouds (No Data)');
    end
    
    % Print results
    fprintf('NDT Localization Results:\n');
    fprintf('Original Pose: [%.3f, %.3f, %.1f°]\n', robotPose(1), robotPose(2), rad2deg(robotPose(3)));
    fprintf('NDT Estimate:  [%.3f, %.3f, %.1f°]\n', mu(1), mu(2), rad2deg(mu(3)));
    fprintf('Position Error: [%.3f, %.3f] m\n', mu(1)-robotPose(1), mu(2)-robotPose(2));
    fprintf('Angle Error: %.1f°\n', rad2deg(mu(3)-robotPose(3)));
    fprintf('Position Uncertainty: %.3f m\n', sqrt(sigma(1,1)));
    fprintf('Angle Uncertainty: %.1f°\n', rad2deg(sqrt(sigma(3,3))));
end 