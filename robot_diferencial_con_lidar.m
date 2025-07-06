%% Robot diferencial con lidar
% Robotica Movil - 2025 2c
close all
clear 
%defino mapa para ver particulas

% Agregar carpeta de funciones de navegacion al path
addpath('navigation_functions');

verMatlab= ver('MATLAB');       % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

ubicado = false;                %poner en true cuando el robot haya terminado la parte de ubicarse
num_particles = 50;            %cantidad de particulas en el filtro (aumentado de 20 a 50)
simular_ruido_lidar = false;    %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;               % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.102';
    ipaddress_local = '192.168.0.108';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.108');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(.5)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(.5) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
%load mapa_TP_2025a.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_fiuba_1p.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_lae.mat         %mapa viejo para probar cosas

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
    imagen_mapa = 1-double(imread('imagen_mapa_viejo.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

particles = []; %genero particulas del filtro de particulas
for i=1:num_particles
    particles=[particles,generar_particula(map)];
end
fprintf("Generando %d particulas \n",num_particles)

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 10;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 60*100; %3*60;     % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [4; 4; pi/2];           % Pose inicial (x y theta) del robot simulado (el robot puede arrancar en cualquier lugar valido del mapa)
                                    %  probar iniciar el robot en distintos lugares                                  
                                  
% Inicializar vectores de tiempo:1010
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
vxRef = 0.1*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.1;
wRef(tVec >=7.5) = 0.1;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

% Variables para la maquina de estados de navegacion
current_state = [];  % Estado actual de la maquina de estados
prev_state_time = 0; % Tiempo del estado anterior

%% Simulacion

if verMatlab.Release=='(R2019a)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
    
    %% COMPLETAR ACA:
        % Generar velocidades usando la maquina de estados para navegacion libre de choques
        % Solo se ejecuta si ya tenemos mediciones del lidar (no en la primera iteracion)
        if idx > 2 && exist('ranges', 'var')
            if ubicado==false
                [v_cmd, w_cmd, current_state] = stateManager(ranges, current_state, idx);
            else
            end
        end
        % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad en el formato que pide el robot:
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = double(ranges_full(1:scaleFactor:end));
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        ranges(ranges<0.05)=NaN; % compensación por errores de medicion no identificados a Dic/24
        
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime;
        % Tomar nueva medicion del lidar
        ranges = double(lidar(pose(:,idx)));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% COMPLETAR ACA:
        
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,particles(1:3,:)); 
        %actualizo pose de particulas
        
        particles(1:3,:)=particles(1:3,:)+vel*sampleTime;
        %actualizo el peso de las particulas
        particles=actualizarPeso_gauss(particles,lidar,ranges,map);
        
        %remuestreo
        particles = resampleParticles(particles, map);

        % Opcional: Mostrar informacion de debug del estado actual
        if mod(idx, 50) == 0  % Mostrar cada 5 segundos
            % Calcular estadísticas de pesos
            weights = particles(4,:);
            max_weight = max(weights);
            min_weight = min(weights);
            mean_weight = mean(weights);
            Neff = 1 / sum(weights.^2);
            
            fprintf('Tiempo: %.1fs, Neff: %.1f, Max_w: %.2e, Min_w: %.2e, Mean_w: %.2e\n', ...
                    tVec(idx), Neff, max_weight, min_weight, mean_weight);
        end
        
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    %viz(pose(:,idx),ranges)
    %solo para debugear voy a plotear las particulas
    %plotearMapa(map)
    plotearParticulas(particles,pose(:,idx),map)
    %waitfor(r);
end
%imse para error cuadratico medio
