%% Simple test to see how weight changes with small pose variations
close all
clear 
clc

% Add necessary paths
addpath('navigation_functions');
addpath('localization_pf');

%% Load map and setup lidar
% Load map


imagen_mapa = 1-double(imread('imagen_mapa_viejo.tiff'))/255;
map = robotics.OccupancyGrid(imagen_mapa, 25);


% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 10;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Base pose and get reference lidar reading
base_pose = [4.0; 4.0; pi/2];
ranges = double(lidar(base_pose));



fprintf('Base pose: [%.2f, %.2f, %.1f�]\n', base_pose(1), base_pose(2), rad2deg(base_pose(3)));

%% Test variations
% Define small changes
delta_x = -0.2:0.01:0.2;  % ±20cm in X
delta_y = -0.2:0.01:0.2;  % ±20cm in Y
delta_th = deg2rad(-10:0.5:10);  % ±10° in theta

% Initialize weight arrays
weights_x = zeros(size(delta_x));
weights_y = zeros(size(delta_y));
weights_th = zeros(size(delta_th));

%% Test X variations
fprintf('Testing X variations...\n');
for i = 1:length(delta_x)
    test_pose = base_pose + [delta_x(i); 0; 0];
    particle = [test_pose; 1];  % [x; y; th; weight]
    particle = actualizarPeso_gauss(particle, lidar, ranges, map);
    weights_x(i) = particle(4);
end

%% Test Y variations
fprintf('Testing Y variations...\n');
for i = 1:length(delta_y)
    test_pose = base_pose + [0; delta_y(i); 0];
    particle = [test_pose; 1];  % [x; y; th; weight]
    particle = actualizarPeso_gauss(particle, lidar, ranges, map);
    weights_y(i) = particle(4);
end

%% Test Theta variations
fprintf('Testing Theta variations...\n');
for i = 1:length(delta_th)
    test_pose = base_pose + [0; 0; delta_th(i)];
    particle = [test_pose; 1];  % [x; y; th; weight]
    particle = actualizarPeso_gauss(particle, lidar, ranges, map);
    weights_th(i) = particle(4);
end

%% Plot results
figure('Position', [100, 100, 1200, 400]);

% X variation plot
subplot(1, 3, 1);
plot(delta_x*100, weights_x, 'b-', 'LineWidth', 2);
xlabel('Delta X (cm)');
ylabel('Weight');
title('Weight vs X Position Change');
grid on;

% Y variation plot
subplot(1, 3, 2);
plot(delta_y*100, weights_y, 'r-', 'LineWidth', 2);
xlabel('Delta Y (cm)');
ylabel('Weight');
title('Weight vs Y Position Change');
grid on;

% Theta variation plot
subplot(1, 3, 3);
plot(rad2deg(delta_th), weights_th, 'g-', 'LineWidth', 2);
xlabel('Delta Theta (degrees)');
ylabel('Weight');
title('Weight vs Orientation Change');
grid on;

sgtitle('Particle Weight Sensitivity to Pose Changes', 'FontSize', 14, 'FontWeight', 'bold');

%% Summary
fprintf('\n=== SUMMARY ===\n');
fprintf('Max weight (X variation): %.6f at delta_x = %.2f cm\n', max(weights_x), delta_x(weights_x==max(weights_x))*100);
fprintf('Max weight (Y variation): %.6f at delta_y = %.2f cm\n', max(weights_y), delta_y(weights_y==max(weights_y))*100);
fprintf('Max weight (Theta variation): %.6f at delta_th = %.1f°\n', max(weights_th), rad2deg(delta_th(weights_th==max(weights_th))));

fprintf('\nTest completed!\n'); 