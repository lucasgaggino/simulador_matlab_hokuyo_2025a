function [] = plotearParticulas(particles,pose, map)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
figure(2)
% Display the map as underlay in grayscale
show(map)
colormap(flipud(gray))  % Set inverted grayscale for map background
hold on
% Extract pose components
pose_x = pose(1);
pose_y = pose(2);
pose_w = pose(3); % orientation angle

% Plot circle for robot position
plot(pose_x, pose_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);

% Plot line for orientation
arrow_length = 0.3; % Length of the orientation line
arrow_x = pose_x + arrow_length * cos(pose_w);
arrow_y = pose_y + arrow_length * sin(pose_w);
plot([pose_x, arrow_x], [pose_y, arrow_y], 'r-', 'LineWidth', 2);

x=particles(1,:);
y=particles(2,:);
theta=particles(3,:);%orientation of particles
c=particles(4,:);%color en base al peso

% Find indices of top 3 particles by weight
[~, sorted_indices] = sort(c, 'descend');
top3_indices = sorted_indices(1:min(10, length(c)));

% Create color array: dark green for top 3, blue for rest
particle_colors = repmat([0, 0, 1], length(x), 1); % All blue initially
particle_colors(top3_indices, :) = repmat([0, 0.5, 0], length(top3_indices), 1); % Top 3 dark green

scatter(x,y,[],particle_colors,'filled')


% Plot orientation lines for particles

particle_arrow_length = 0.15; % Length of particle orientation lines
for i = 1:length(x)
    arrow_x_end = x(i) + particle_arrow_length * cos(theta(i));
    arrow_y_end = y(i) + particle_arrow_length * sin(theta(i));
    plot([x(i), arrow_x_end], [y(i), arrow_y_end], 'k-', 'LineWidth', 1);
end

% Plot the pose as a circle with an arrow


hold off

%xlim([0 10])
%ylim([0 10])
end

