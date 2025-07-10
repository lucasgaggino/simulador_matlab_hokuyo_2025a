%% NDT Integration Example
% This shows how to integrate NDT localization into the main robot script
% with minimal changes to the existing code

function ndt_integration_example()
    % This function demonstrates how to integrate NDT localization
    % into the main robot script with minimal changes
    
    fprintf('NDT Integration Example:\n');
    fprintf('=======================\n\n');
    
    fprintf('1. Add NDT path to main script:\n');
    fprintf('   addpath(''localization_ndt'');\n\n');
    
    fprintf('2. Initialize NDT variables (add after line ~50):\n');
    fprintf('   %% NDT Localization variables\n');
    fprintf('   use_ndt = true;           %% Enable NDT localization\n');
    fprintf('   ndt_pose = initPose;      %% NDT pose estimate\n');
    fprintf('   ndt_sigma = eye(3)*0.1;   %% NDT uncertainty\n');
    fprintf('   ndt_success = false;      %% NDT success flag\n\n');
    
    fprintf('3. Replace particle filter code (around line 175) with:\n');
    fprintf('   if use_ndt\n');
    fprintf('       %% NDT Localization\n');
    fprintf('       [ndt_pose, ndt_sigma, ndt_success] = ndt_localize(ranges, lidar.scanAngles, pose(:,idx), map);\n');
    fprintf('       if ndt_success\n');
    fprintf('           pose(:,idx) = ndt_pose;  %% Update pose with NDT result\n');
    fprintf('           fprintf(''NDT: Pose updated to [%%.2f, %%.2f, %%.1f°]\\n'', pose(1,idx), pose(2,idx), rad2deg(pose(3,idx)));\n');
    fprintf('       else\n');
    fprintf('           fprintf(''NDT: Using odometry pose\\n'');\n');
    fprintf('       end\n');
    fprintf('   else\n');
    fprintf('       %% Original particle filter code\n');
    fprintf('       velB = [v;0;w];\n');
    fprintf('       vel = bodyToWorld(velB,particles(1:3,:));\n');
    fprintf('       particles(1:3,:)=particles(1:3,:)+vel*sampleTime;\n');
    fprintf('       particles=actualizarPeso_gauss(particles,lidar,ranges,map);\n');
    fprintf('       particles = resampleParticles(particles, map);\n');
    fprintf('   end\n\n');
    
    fprintf('4. Optional: Add NDT visualization (replace plotting code):\n');
    fprintf('   if use_ndt && ndt_success\n');
    fprintf('       %% Simple NDT visualization\n');
    fprintf('       figure(1); clf;\n');
    fprintf('       show(map); hold on;\n');
    fprintf('       plot(ndt_pose(1), ndt_pose(2), ''go'', ''MarkerSize'', 10, ''LineWidth'', 2);\n');
    fprintf('       %% Plot uncertainty ellipse\n');
    fprintf('       pos_cov = ndt_sigma(1:2, 1:2);\n');
    fprintf('       [V, D] = eig(pos_cov);\n');
    fprintf('       angle = atan2(V(2,1), V(1,1));\n');
    fprintf('       a = 2.45 * sqrt(D(1,1)); b = 2.45 * sqrt(D(2,2));\n');
    fprintf('       theta = linspace(0, 2*pi, 100);\n');
    fprintf('       ellipse = [a*cos(theta); b*sin(theta)];\n');
    fprintf('       R = [cos(angle) -sin(angle); sin(angle) cos(angle)];\n');
    fprintf('       ellipse_rot = R * ellipse;\n');
    fprintf('       plot(ndt_pose(1) + ellipse_rot(1,:), ndt_pose(2) + ellipse_rot(2,:), ''g--'');\n');
    fprintf('       title(''NDT Localization''); drawnow;\n');
    fprintf('   end\n\n');
    
    fprintf('5. Complete minimal integration code:\n');
    fprintf('   See the code block below for a complete minimal example.\n\n');
    
    % Show the complete code block
    show_complete_integration_code();
end

function show_complete_integration_code()
    fprintf('Complete Integration Code Block:\n');
    fprintf('================================\n\n');
    
    code_block = sprintf([
        '%% Add this after line ~8 in main script:\n'...
        'addpath(''localization_ndt'');\n\n'...
        '%% Add these variables after line ~50:\n'...
        'use_ndt = true;           %% Enable NDT localization\n'...
        'ndt_pose = initPose;      %% NDT pose estimate\n'...
        'ndt_sigma = eye(3)*0.1;   %% NDT uncertainty\n'...
        'ndt_success = false;      %% NDT success flag\n\n'...
        '%% Replace the "COMPLETAR ACA" section (around line 175) with:\n'...
        'if use_ndt\n'...
        '    %% NDT Localization\n'...
        '    [ndt_pose, ndt_sigma, ndt_success] = ndt_localize(ranges, lidar.scanAngles, pose(:,idx), map);\n'...
        '    if ndt_success\n'...
        '        pose(:,idx) = ndt_pose;  %% Update pose with NDT result\n'...
        '        if mod(idx, 50) == 0  %% Print every 5 seconds\n'...
        '            fprintf(''NDT: [%%.2f, %%.2f, %%.1f°], σ=%%.3f m\\n'', ...\n'...
        '                pose(1,idx), pose(2,idx), rad2deg(pose(3,idx)), sqrt(ndt_sigma(1,1)));\n'...
        '        end\n'...
        '    end\n'...
        'else\n'...
        '    %% Original particle filter code\n'...
        '    velB = [v;0;w];\n'...
        '    vel = bodyToWorld(velB,particles(1:3,:));\n'...
        '    particles(1:3,:)=particles(1:3,:)+vel*sampleTime;\n'...
        '    particles=actualizarPeso_gauss(particles,lidar,ranges,map);\n'...
        '    particles = resampleParticles(particles, map);\n'...
        '    plotearParticulas(particles,pose(:,idx),map,idx,false);\n'...
        'end\n\n'...
        '%% Optional: Replace visualization with NDT plot\n'...
        'if use_ndt && ndt_success && mod(idx, 10) == 0\n'...
        '    figure(1); clf; show(map); hold on;\n'...
        '    plot(ndt_pose(1), ndt_pose(2), ''go'', ''MarkerSize'', 8, ''LineWidth'', 2);\n'...
        '    title(sprintf(''NDT Localization - Time: %%.1fs'', tVec(idx)));\n'...
        '    drawnow;\n'...
        'end\n'
    ]);
    
    fprintf('%s', code_block);
    fprintf('\n');
end 