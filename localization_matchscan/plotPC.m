function plotPC(mapPC,tfrom,pose,scan,stats)

    scan2Tformed = transformScan(scan,tfrom);
    scanreal = transformScan(scan,pose);
    figure(1)
    
    plot(mapPC)
    hold on
    plot(scan2Tformed)
    plot(scanreal)
    %plot(truePose(1), truePose(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2)
    %plot(error_pose_corrected(1), error_pose_corrected(2), 'ko', 'MarkerSize', 10, 'LineWidth', 2)
    
    leg_names= ["Map","Transformed Scan", "Real Scan"];
    legend(leg_names,'Location','Best');
    title(["Score:",stats.Score])
    hold off
    


end