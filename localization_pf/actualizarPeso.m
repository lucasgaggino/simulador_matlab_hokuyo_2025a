function [particles] = actualizarPeso(particles,lidar,ranges,map)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
ranges_particles=ones(length(ranges),size(particles,2));
%las particulas que esten afuera del mapa son reemplazadas por particulas
%nuevas aleatoriamente generada
xmap_max=map.GridSize(1)*map.Resolution;
ymap_max=map.GridSize(2)*map.Resolution;
for i=1:size(particles,2)
    
    try
        if particles(1,i)>xmap_max||particles(1,i)<0||particles(2,i)>ymap_max||particles(2,i)<0
            particles(:,i)=generar_particula(map);
        elseif map.getOccupancy([particles(1,i) particles(2,i)])>0.1
            particles(:,i)=generar_particula(map);
        end
    catch ME
        fprintf("Particle: %d location: [%f.3 , %f.3] \n",i,particles(1,i),particles(2,i))
        ME = addCause(ME,causeException);
    end
    
    ranges_particles(:,i)=double(lidar(particles(1:3,i)));
end
inv_weight = mean(abs(ranges-ranges_particles),'omitnan');
if isnan(inv_weight)
    inv_weight = 100;
end
particles(4,:)=1./max(inv_weight,0.01);
end

