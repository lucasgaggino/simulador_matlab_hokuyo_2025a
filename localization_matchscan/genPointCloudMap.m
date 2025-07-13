function [mapPC] = genPointCloudMap(map,downsampleFactor)


    bin_map  = map.occupancyMatrix > map.OccupiedThreshold;
    [Y,X] = find(bin_map);
    X = X/map.Resolution;
    Y = (map.GridSize(2)- Y)  / map.Resolution;
    mapCartPC = [X,Y];
    mapCartPC = downsample(mapCartPC,downsampleFactor);
    mapPC = lidarScan(mapCartPC);
    
    
    ptCloud = pcread('mapa.ply');
    loc = ptCloud.Location(:,1:2);
    mapPC = lidarScan(loc);
    
end












