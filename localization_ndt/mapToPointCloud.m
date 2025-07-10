function ptCloud = mapToPointCloud(map, robotPose, radius)
    % Convert occupancy map to point cloud for NDT registration
    % Inputs:
    %   map - robotics.OccupancyGrid object
    %   robotPose - current robot pose [x, y, theta] (for local extraction)
    %   radius - radius around robot to extract map points (optional, default 3.0m)
    % Output:
    %   ptCloud - pointCloud object of occupied cells
    
    if nargin < 3
        radius = 3.0; % Default radius
    end
    
    % Get map properties
    resolution = map.Resolution;
    grid_size = map.GridSize;
    world_limits_x = map.XWorldLimits;
    world_limits_y = map.YWorldLimits;
    
    % Define local region around robot
    x_min = max(world_limits_x(1), robotPose(1) - radius);
    x_max = min(world_limits_x(2), robotPose(1) + radius);
    y_min = max(world_limits_y(1), robotPose(2) - radius);
    y_max = min(world_limits_y(2), robotPose(2) + radius);
    
    % Create grid of points to check
    x_step = resolution / 2; % Higher resolution for better matching
    y_step = resolution / 2;
    
    [X, Y] = meshgrid(x_min:x_step:x_max, y_min:y_step:y_max);
    grid_points = [X(:), Y(:)];
    
    % Get occupancy values for these points
    occupancy = map.occupancyMatrix;
    
    % Keep only occupied cells (threshold > 0.5)
    occupied_idx = occupancy > 0.5;
    occupied_points = grid_points(occupied_idx, :);
    
    % Check if we have enough occupied points
    if size(occupied_points, 1) < 20
        % Return empty point cloud if not enough data
        ptCloud = pointCloud(zeros(0,3));
        return;
    end
    
    % Add z-coordinate (zero for 2D)
    points_3d = [occupied_points, zeros(size(occupied_points, 1), 1)];
    
    % Create point cloud
    ptCloud = pointCloud(points_3d);
end 