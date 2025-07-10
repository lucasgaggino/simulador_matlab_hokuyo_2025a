function ptCloud = lidarToPointCloud(ranges, scanAngles, robotPose)
    % Convert lidar ranges to point cloud
    % Inputs:
    %   ranges - vector of lidar distances
    %   scanAngles - vector of scan angles (in radians)
    %   robotPose - current robot pose [x, y, theta]
    % Output:
    %   ptCloud - pointCloud object
    
    % Filter out invalid readings
    valid_idx = ~isnan(ranges) & ~isinf(ranges) & (ranges > 0.05) & (ranges < 5.0);
    valid_ranges = ranges(valid_idx);
    valid_angles = scanAngles(valid_idx);
    
    % Check if we have enough valid points
    if length(valid_ranges) < 10
        % Return empty point cloud if not enough data
        ptCloud = pointCloud(zeros(0,3));
        return;
    end
    
    % Convert to Cartesian coordinates in robot frame
    x_robot = valid_ranges .* cos(valid_angles);
    y_robot = valid_ranges .* sin(valid_angles);
    z_robot = zeros(size(x_robot)); % 2D case, z=0
    
    % Transform to world frame
    cos_theta = cos(robotPose(3));
    sin_theta = sin(robotPose(3));
    
    x_world = robotPose(1) + cos_theta * x_robot - sin_theta * y_robot;
    y_world = robotPose(2) + sin_theta * x_robot + cos_theta * y_robot;
    z_world = z_robot; % Still zero for 2D
    
    % Create point cloud
    points = [x_world(:), y_world(:), z_world(:)];
    ptCloud = pointCloud(points);
end 