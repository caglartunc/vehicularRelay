% VEHICLE PROPERTIES
vehicleProperties.speed = [100, 100]; % in km/h
vehicleProperties.speed_ms = vehicleProperties.speed./3600; % in m/ms
vehicleProperties.length = [5, 13];
vehicleProperties.height = [1.6, 3];
vehicleProperties.width = [3, 3];
vehicleProperties.probability = [0.5 0.5];
vehicleProperties.commProbability = [0.1 0]; % probability of communication for each vehicle class

% DEPLOYMENT PARAMETERS
config.numLane = 4;
config.heightRSU = 6;
config.heightAntenna = 1.6;
config.numRSU = 1;
config.laneWidth = 4;
config.LOSrange = 200;
config.CVlane = 4;
config.CVspeed = 120; % in km/h
config.CVspeed_ms = config.CVspeed/3600; % in m/ms
config.s_average = 1/30; % 40 meters inter-vehicle distance

% SIMULATION PARAMETERS
config.TIME_LIMIT = 1e5; % in ms
config.MAX_ITER = 1000; % total number of simulations to run
config.DELTA = 1; % time granularity in ms. Can be chosen >1 to speed up simulations (but might miss some blockage events)
config.MAX_NUM_BLOCKAGE = 20; % for each simulation, stop if MAX_NUM_BLOCKAGE blockages have been recorded
