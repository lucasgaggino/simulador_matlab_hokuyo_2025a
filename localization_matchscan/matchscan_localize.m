function [tform, stats] = matchscan_localize(ranges, angles, initialGuess, map, maxRange)
if nargin < 5
    maxRange = 5;
end
scan = lidarScan(ranges, angles);
[tform, stats] = matchScans(scan, map, 'InitialPose', initialGuess,'SolverAlgorithm', 'fminunc');
% scan2Tformed = transformScan(scan,tform);
% estimatedPose = [tform.Translation(1); tform.Translation(2); tform.Yaw];
end 