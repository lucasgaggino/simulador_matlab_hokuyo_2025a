clear
close all
imagen_mapa = 1-double(imread('imagen_mapa_viejo.tiff'))/255;
map = robotics.OccupancyGrid(imagen_mapa, 25);
bmap = robotics.BinaryOccupancyGrid(imagen_mapa,25);
bmap.inflate(0.2)





start= [2 2];
goal = [5 4];
planner = robotics.PRM(bmap) ;
controller  = robotics.PurePursuit;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity =  0.5;


xy = findpath(planner,start,goal);
controller .Waypoints = xy;

[v,w] = controller ([start 0])


figure
subplot(1,2,1)
show(bmap)
subplot(1,2,2)
show(planner)
