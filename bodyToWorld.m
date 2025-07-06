function velW = bodyToWorld(velB,pose)
% BODYTOWORLD Converts a 2D velocity input velB = [vx;vy;w] from body 
% (vehicle) coordinates to world (global) coordinates.
%
% velB = 3xN matrix where each column is [vx;vy;w] for N vehicles
%        OR 3x1 vector to be applied to all poses
% pose = 3xN matrix where each column is [x;y;theta] for N vehicles
% vx = Linear velocity in x-direction (longitudinal)
% vy = Linear velocity in y-direction (lateral)
% w = Angular velocity (about z-axis)
%
% Copyright 2018 The MathWorks, Inc.

    % Extract theta values (third row of pose matrix)
    theta = pose(3,:);
    N = size(pose,2);
    
    % Handle case where velB is 3x1 but pose is 3xN
    if size(velB,2) == 1 && N > 1
        velB = repmat(velB, 1, N);
    end
    
    % Vectorized rotation transformation
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    
    % Initialize output matrix with correct size
    velW = zeros(3, N);
    
    % Apply rotation matrix to each column using element-wise operations
    velW(1,:) = cos_theta .* velB(1,:) - sin_theta .* velB(2,:);
    velW(2,:) = sin_theta .* velB(1,:) + cos_theta .* velB(2,:);
    velW(3,:) = velB(3,:);  % Angular velocity remains unchanged

end
