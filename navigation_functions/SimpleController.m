classdef SimpleController < handle
    properties (Access = public)
        waypoints
        max_linear_velocity
        max_angular_velocity
        waypoint_distance_threshold
        current_waypoint_index
    end
    
    methods
        function obj = SimpleController(waypoints, max_linear_velocity, max_angular_velocity, waypoint_distance_threshold)
            % Constructor
            % waypoints: Nx2 matrix of [x, y] coordinates
            % max_linear_velocity: maximum linear velocity (m/s)
            % max_angular_velocity: maximum angular velocity (rad/s)
            % waypoint_distance_threshold: distance threshold to consider waypoint reached (m)
            
            if nargin < 4
                error('SimpleController requires 4 arguments: waypoints, max_linear_velocity, max_angular_velocity, waypoint_distance_threshold');
            end
            
            obj.waypoints = waypoints;
            obj.max_linear_velocity = max_linear_velocity;
            obj.max_angular_velocity = max_angular_velocity;
            obj.waypoint_distance_threshold = waypoint_distance_threshold;
            obj.current_waypoint_index = 1;
        end
        
        function [v, w] = getVelocities(obj, pose)
            % Get velocities based on current pose
            % pose: [x, y, theta] - current robot pose
            % Returns: v (linear velocity), w (angular velocity)
            
            % Check if we have reached all waypoints
            if obj.current_waypoint_index > size(obj.waypoints, 1)
                v = 0;
                w = 0;
                return;
            end
            
            % Get current target waypoint
            target = obj.waypoints(obj.current_waypoint_index, :);
            
            % Calculate distance to target
            dx = target(1) - pose(1);
            dy = target(2) - pose(2);
            distance = sqrt(dx^2 + dy^2);
            
            % Check if we've reached the current waypoint
            if distance <= obj.waypoint_distance_threshold
                obj.current_waypoint_index = obj.current_waypoint_index + 1;
                
                % Check if we've reached all waypoints
                if obj.current_waypoint_index > size(obj.waypoints, 1)
                    v = 0;
                    w = 0;
                    return;
                end
                
                % Update target to next waypoint
                target = obj.waypoints(obj.current_waypoint_index, :);
                dx = target(1) - pose(1);
                dy = target(2) - pose(2);
                distance = sqrt(dx^2 + dy^2);
            end
            
            % Calculate desired heading
            desired_theta = atan2(dy, dx);
            
            % Calculate angular error
            theta_error = desired_theta - pose(3);
            
            % Normalize angle to [-pi, pi]
            theta_error = atan2(sin(theta_error), cos(theta_error));
            
            % Simple proportional controller for angular velocity
            kp_angular = 2.0; % Proportional gain for angular control
            w = kp_angular * theta_error;
            
            % Limit angular velocity
            w = max(-obj.max_angular_velocity, min(obj.max_angular_velocity, w));
            
            % Simple proportional controller for linear velocity
            % Reduce linear velocity when angular error is large
            kp_linear = 0.5; % Proportional gain for linear control
            angular_factor = max(0, 1 - abs(theta_error) / (pi/2)); % Reduce speed when turning
            v = kp_linear * distance * angular_factor;
            
            % Limit linear velocity
            v = max(0, min(obj.max_linear_velocity, v));
        end
        
        function waypoints = getRemainingWaypoints(obj)
            % Get remaining waypoints
            if obj.current_waypoint_index > size(obj.waypoints, 1)
                waypoints = [];
            else
                waypoints = obj.waypoints(obj.current_waypoint_index:end, :);
            end
        end
        
        function current_target = getCurrentTarget(obj)
            % Get current target waypoint
            if obj.current_waypoint_index > size(obj.waypoints, 1)
                current_target = [];
            else
                current_target = obj.waypoints(obj.current_waypoint_index, :);
            end
        end
        
        function finished = isFinished(obj)
            % Check if all waypoints have been reached
            finished = obj.current_waypoint_index > size(obj.waypoints, 1);
        end
        
        function setWaypoints(obj, waypoints)
            % Set new waypoints and reset controller
            obj.waypoints = waypoints;
            obj.current_waypoint_index = 1;
        end
    end
end 