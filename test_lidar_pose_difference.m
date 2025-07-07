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
delta_x = -0.5:0.01:0.5;  % ±20cm in X
delta_y = -0.5:0.01:0.5;  % ±20cm in Y
delta_th = deg2rad(-10:0.5:10);  % ±10° in theta

% Initialize weight arrays for both methods
weights_x_gauss = zeros(size(delta_x));
weights_y_gauss = zeros(size(delta_y));
weights_th_gauss = zeros(size(delta_th));

weights_x_corr = zeros(size(delta_x));
weights_y_corr = zeros(size(delta_y));
weights_th_corr = zeros(size(delta_th));

%% Test X variations
fprintf('Testing X variations...\n');
for i = 1:length(delta_x)
    test_pose = base_pose + [delta_x(i); 0; 0];
    simulated_ranges = double(lidar(test_pose));
    weights_x_gauss(i) = weight_gauss(ranges, simulated_ranges);
    weights_x_corr(i) = weight_corr(ranges, simulated_ranges);
end

%% Test Y variations
fprintf('Testing Y variations...\n');
for i = 1:length(delta_y)
    test_pose = base_pose + [0; delta_y(i); 0];
    simulated_ranges = double(lidar(test_pose));
    weights_y_gauss(i) = weight_gauss(ranges, simulated_ranges);
    weights_y_corr(i) = weight_corr(ranges, simulated_ranges);
end

%% Test Theta variations
fprintf('Testing Theta variations...\n');
for i = 1:length(delta_th)
    test_pose = base_pose + [0; 0; delta_th(i)];
    simulated_ranges = double(lidar(test_pose));
    weights_th_gauss(i) = weight_gauss(ranges, simulated_ranges);
    weights_th_corr(i) = weight_corr(ranges, simulated_ranges);
end

%% Plot results
figure('Position', [100, 100, 1400, 800]);

% X variation plots
subplot(2, 3, 1);
plot(delta_x*100, weights_x_gauss, 'b-', 'LineWidth', 2);
xlabel('Delta X (cm)');
ylabel('Weight');
title('Gaussian Weight vs X Position Change');
grid on;

subplot(2, 3, 4);
plot(delta_x*100, weights_x_corr, 'b-', 'LineWidth', 2);
xlabel('Delta X (cm)');
ylabel('Weight');
title('Correlation Weight vs X Position Change');
grid on;

% Y variation plots
subplot(2, 3, 2);
plot(delta_y*100, weights_y_gauss, 'r-', 'LineWidth', 2);
xlabel('Delta Y (cm)');
ylabel('Weight');
title('Gaussian Weight vs Y Position Change');
grid on;

subplot(2, 3, 5);
plot(delta_y*100, weights_y_corr, 'r-', 'LineWidth', 2);
xlabel('Delta Y (cm)');
ylabel('Weight');
title('Correlation Weight vs Y Position Change');
grid on;

% Theta variation plots
subplot(2, 3, 3);
plot(rad2deg(delta_th), weights_th_gauss, 'g-', 'LineWidth', 2);
xlabel('Delta Theta (degrees)');
ylabel('Weight');
title('Gaussian Weight vs Orientation Change');
grid on;

subplot(2, 3, 6);
plot(rad2deg(delta_th), weights_th_corr, 'g-', 'LineWidth', 2);
xlabel('Delta Theta (degrees)');
ylabel('Weight');
title('Correlation Weight vs Orientation Change');
grid on;

sgtitle('Comparison of Gaussian vs Correlation Weight Methods', 'FontSize', 14, 'FontWeight', 'bold');

%% Summary
fprintf('\n=== SUMMARY ===\n');
fprintf('GAUSSIAN METHOD:\n');
fprintf('Max weight (X variation): %.6f at delta_x = %.2f cm\n', max(weights_x_gauss), delta_x(weights_x_gauss==max(weights_x_gauss))*100);
fprintf('Max weight (Y variation): %.6f at delta_y = %.2f cm\n', max(weights_y_gauss), delta_y(weights_y_gauss==max(weights_y_gauss))*100);
fprintf('Max weight (Theta variation): %.6f at delta_th = %.1f°\n', max(weights_th_gauss), rad2deg(delta_th(weights_th_gauss==max(weights_th_gauss))));

fprintf('\nCORRELATION METHOD:\n');
fprintf('Max weight (X variation): %.6f at delta_x = %.2f cm\n', max(weights_x_corr), delta_x(weights_x_corr==max(weights_x_corr))*100);
fprintf('Max weight (Y variation): %.6f at delta_y = %.2f cm\n', max(weights_y_corr), delta_y(weights_y_corr==max(weights_y_corr))*100);
fprintf('Max weight (Theta variation): %.6f at delta_th = %.1f°\n', max(weights_th_corr), rad2deg(delta_th(weights_th_corr==max(weights_th_corr))));

fprintf('\nTest completed!\n'); 