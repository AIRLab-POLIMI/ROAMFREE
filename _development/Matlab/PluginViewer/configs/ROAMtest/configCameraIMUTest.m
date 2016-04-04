
%% create config

config = struct();
 
ss = get(0,'MonitorPositions'); % gets screen coordinates
    
config.global.layout = 'horizontal';
% config.global.figureOuterPosition = [ss(1,1) ss(1,2) ss(1,3) ss(1,4)]; % monitor 1
config.global.figureOuterPosition = [ss(2,1) ss(2,2) ss(2,3) ss(2,4)]; % monitor 2
config.global.logPath = '/tmp/roamfree/';

config.global.plugins = {};
config.pluginConfig = {};

config.global.plugins{end+1} = 'Trajectory';
config.pluginConfig{end+1}.axesLenght = 0.5;
config.pluginConfig{end}.absolutePositionSensor = 'GPS'; %display also measures from an AbsolutePosition sensor
config.pluginConfig{end}.euclideanFeatureSensors = 'Camera'; %display also euclidean 3D features.
% config.pluginConfig{end}.FHPFeatureSensors = 'Camera'; %display also FHP 3D features.

config.global.plugins{end+1} = 'EuclideanFeatures';
config.pluginConfig{end+1}.sensorName = 'Camera';

config.global.plugins{end+1} = 'GenericEdge';
config.pluginConfig{end+1}.sensorName = 'GPS';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 3;
% 
% config.global.plugins{end+1} = 'GenericEdge';   
% config.pluginConfig{end+1}.sensorName = 'IMUintegralDeltaP';
% config.pluginConfig{end}.errorSize = 3;
% config.pluginConfig{end}.measureSize = 27;
% 
% config.global.plugins{end+1} = 'GenericEdge';
% config.pluginConfig{end+1}.sensorName = 'IMUintegralDeltaQ';
% config.pluginConfig{end}.errorSize = 4;
% config.pluginConfig{end}.measureSize = 16;
% 
% config.global.plugins{end+1} = 'StochasticProcess';
% config.pluginConfig{end+1}.parameterName = 'IMUintegralDeltaP_Ba';
% config.pluginConfig{end}.parameterSize = 3;
% 
% config.global.plugins{end+1} = 'StochasticProcess';
% config.pluginConfig{end+1}.parameterName = 'IMUintegralDeltaP_Bw';
% config.pluginConfig{end}.parameterSize = 3;
