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
scaleFactor = 1;
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

%% Crear Nube de puntos del mapa

bin_map  = map.occupancyMatrix > map.OccupiedThreshold;
[Y,X] = find(bin_map);
X = X/map.Resolution;
Y = (map.GridSize(2)- Y)  / map.Resolution;
mapCartPC = [X,Y];
mapCartPC = downsample(mapCartPC,10);


mapPC = lidarScan(mapCartPC);
%plot(mapPC.Cartesian(:,1),mapPC.Cartesian(:,2),'.')
plot(mapPC)
pause(1)
figure
show(map)
pause(1)



%% Test NDT localization at different poses
test_poses = [
    4.0, 4.0, pi/2;     % Center of map
    3.0, 3.0, pi/4;     % Different position
    5.0, 5.0, 0;        % Another position
    4.5, 3.5, -pi/4;    % Different orientation
];
angles = lidar.scanAngles;

for i = 1:size(test_poses, 1)
    fprintf('\n--- Test %d ---\n', i);
    
    % True robot pose
    truePose = test_poses(i, :)';
    
    % Add some noise to create "estimated" pose
    noise_std = [0.2, 0.2, deg2rad(10)]; % 20cm, 20cm, 10 degrees
    noisyPose = truePose + noise_std' .* randn(3, 1);
    
    % Get lidar measurement at true pose
    ranges = double(lidar(truePose));
    scan_real  = lidarScan(ranges,angles);
    
    ranges_noisy = double(lidar(noisyPose));
    scan_noisy = lidarScan(ranges_noisy,angles);
    
    [relPose,stats] = matchScansGrid(scan_noisy,mapPC,'InitialPose',truePose);
    scan2Tformed = transformScan(scan_noisy,relPose);

    R =rotz(relPose(3));
    T = [R(1:2,1:2),relPose(1:2)'; 0 0 1];
    
    
    correctedPose = T*noisyPose;
    error_pose = truePose - noisyPose;
    error_pose_corrected = truePose - correctedPose;
    
    figure(1)
    plot(mapPC)
    hold on
    plot(scan_noisy)
    plot(scan2Tformed)
    plot(truePose(1), truePose(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2)
    plot(error_pose_corrected(1), error_pose_corrected(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2)
    


    leg_names= ["Real","Noise","Transformed"];
    legend(leg_names,'Location','Best');
    hold off
    fprintf('True pose: [%.2f, %.2f, %.2f°]\n', truePose(1), truePose(2), rad2deg(truePose(3)));
    fprintf('Noisy pose: [%.2f, %.2f, %.2f°]\n', noisyPose(1), noisyPose(2), rad2deg(noisyPose(3)));
    fprintf('Corrected pose: [%.2f, %.2f, %.2f°]\n', correctedPose(1), correctedPose(2), rad2deg(correctedPose(3)));
    fprintf('Position error before: %.3f m\n', norm(error_pose(1:2)));
    fprintf('Position error after: %.3f m\n', norm(error_pose_corrected(1:2)));
    fprintf('Angle error before: %.2f°\n', rad2deg(abs(error_pose(3))));
    fprintf('Angle error after: %.2f°\n', rad2deg(abs(error_pose_corrected(3))));
    

    figure(2)
    viz(truePose,ranges)
    pause(1)
    
end

fprintf('\nNDT Localization test completed.\n'); 