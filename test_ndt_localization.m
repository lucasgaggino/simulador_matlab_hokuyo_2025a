%% Test NDT Localization
% Simple test script to demonstrate NDT-based localization
close all;
clear;
clc;

% Add NDT functions to path
addpath('localization_ndt');

fprintf('Testing NDT Localization...\n');

%% Setup map and lidar
% Load map
verMatlab = ver('MATLAB');
if verMatlab.Release=='(R2016b)'
    imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'
    imagen_mapa = 1-double(imread('imagen_mapa_viejo.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
else
    % Try loading from mat file first
    if exist('mapa_fiuba_1p.mat', 'file')
        load('mapa_fiuba_1p.mat'); % This should load 'map' variable
    else
        % Fall back to image
        imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
        map = robotics.OccupancyGrid(imagen_mapa, 25);
    end
end

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
scaleFactor = 10;
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);
lidar.scanAngles = linspace(hokuyo_step_a, hokuyo_step_c, num_scans);
lidar.maxRange = 5;

fprintf('Map loaded: %dx%d cells, Resolution: %.2f m/cell\n', ...
    map.GridSize(1), map.GridSize(2), map.Resolution);
%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);



%% Test NDT localization at different poses
test_poses = [
    4.0, 4.0, pi/2;     % Center of map
    3.0, 3.0, pi/4;     % Different position
    5.0, 5.0, 0;        % Another position
    4.5, 3.5, -pi/4;    % Different orientation
];

for i = 1:size(test_poses, 1)
    fprintf('\n--- Test %d ---\n', i);
    
    % True robot pose
    truePose = test_poses(i, :)';
    
    % Add some noise to create "estimated" pose
    noise_std = [0.2, 0.2, deg2rad(10)]; % 20cm, 20cm, 10 degrees
    noisyPose = truePose + noise_std' .* randn(3, 1);
    
    % Get lidar measurement at true pose
    ranges = double(lidar(truePose));
    
    % Check if we have valid lidar data
    valid_ranges = sum(~isnan(ranges) & ~isinf(ranges));
    if valid_ranges < 10
        fprintf('Not enough valid lidar readings (%d), skipping...\n', valid_ranges);
        continue;
    end
    
    % Perform NDT localization
    fprintf('True Pose: [%.3f, %.3f, %.1f°]\n', truePose(1), truePose(2), rad2deg(truePose(3)));
    fprintf('Noisy Pose: [%.3f, %.3f, %.1f°]\n', noisyPose(1), noisyPose(2), rad2deg(noisyPose(3)));
    
    tic;
    [mu, sigma, success] = ndt_localize(ranges, lidar.scanAngles, noisyPose, map);
    elapsed_time = toc;
    
    if success
        fprintf('NDT Success! Time: %.3f seconds\n', elapsed_time);
        
        % Calculate errors
        position_error = norm(mu(1:2) - truePose(1:2));
        angle_error = abs(angdiff(mu(3), truePose(3)));
        
        fprintf('Position Error: %.3f m\n', position_error);
        fprintf('Angle Error: %.1f°\n', rad2deg(angle_error));
        
        % Visualize result
        visualize_ndt_result(mu, sigma, noisyPose, map, ranges, lidar.scanAngles);
        
        % Add title to distinguish different tests
        sgtitle(sprintf('NDT Test %d - Position Error: %.3f m, Angle Error: %.1f°', ...
            i, position_error, rad2deg(angle_error)));
        
    else
        fprintf('NDT Failed!\n');
    end
    
    % Pause between tests
    if i < size(test_poses, 1)
        fprintf('Press any key to continue to next test...\n');
        pause;
    end
end

fprintf('\nNDT Localization test completed.\n'); 