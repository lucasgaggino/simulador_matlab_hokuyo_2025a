% Test script for SimpleController
clear all
close all

% Define waypoints
waypoints = [1, 1; 3, 2; 5, 1; 4, 4; 2, 3];

% Controller parameters
max_linear_velocity = 0.5;    % m/s
max_angular_velocity = 1.0;   % rad/s
waypoint_distance_threshold = 0.2; % m

% Create controller
controller = SimpleController(waypoints, max_linear_velocity, max_angular_velocity, waypoint_distance_threshold);

% Simulate robot following waypoints
current_pose = [0, 0, 0]; % [x, y, theta]
dt = 0.1; % time step
trajectory = [];

figure;
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro-', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
title('Robot Navigation with SimpleController');
grid on;

% Simulation loop
for i = 1:500 % Maximum iterations
    % Get velocities from controller
    [v, w] = controller.getVelocities(current_pose);
    
    % Check if finished
    if controller.isFinished()
        fprintf('All waypoints reached!\n');
        break;
    end
    
    % Update pose (simple integration)
    current_pose(1) = current_pose(1) + v * cos(current_pose(3)) * dt;
    current_pose(2) = current_pose(2) + v * sin(current_pose(3)) * dt;
    current_pose(3) = current_pose(3) + w * dt;
    
    % Store trajectory
    trajectory = [trajectory; current_pose];
    
    % Display current status
    if mod(i, 50) == 0
        current_target = controller.getCurrentTarget();
        if ~isempty(current_target)
            fprintf('Step %d: Pose = [%.2f, %.2f, %.2f], Target = [%.2f, %.2f], v = %.3f, w = %.3f\n', ...
                i, current_pose(1), current_pose(2), current_pose(3), ...
                current_target(1), current_target(2), v, w);
        end
    end
end

% Plot trajectory
if ~isempty(trajectory)
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
    plot(trajectory(1,1), trajectory(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(trajectory(end,1), trajectory(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end

legend('Waypoints', 'Trajectory', 'Start', 'End', 'Location', 'best');
axis equal; 