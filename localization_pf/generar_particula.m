function [particula] = generar_particula(map)
%UNTITLED Summary of this function goes here
%   La funcion debe generar una particula aleatoriamente ubicada en el mapa
    particula=[rand(3,1).*[map.XWorldLimits(2),map.YWorldLimits(2),2*pi]';0];
    while map.getOccupancy([particula(1) particula(2)])>0.5
        particula=[rand(3,1).*[map.XWorldLimits(2),map.YWorldLimits(2),2*pi]';0];
    end
end

