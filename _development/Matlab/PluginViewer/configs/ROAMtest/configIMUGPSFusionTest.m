
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
config.pluginConfig{1}.axesLenght = 0.5;
config.pluginConfig{end}.absolutePositionSensor = 'GPS'; %display also measures from an AbsolutePosition sensor

config.global.plugins{end+1} = 'GenericEdge';
config.pluginConfig{end+1}.sensorName = 'GPS';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 3;

config.global.plugins{end+1} = 'GenericEdge';
config.pluginConfig{end+1}.sensorName = 'Accelerometer';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 3;

config.global.plugins{end+1} = 'GenericEdge';
config.pluginConfig{end+1}.sensorName = 'Gyroscope';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 3;

% config.global.plugins{end+1} = 'LinearlyInterpolatedEuclideanParameter';
% config.pluginConfig{end+1}.parameterName = 'Accelerometer_B';
% config.pluginConfig{end}.parameterSize = 3;
% 
% config.global.plugins{end+1} = 'LinearlyInterpolatedEuclideanParameter';
% config.pluginConfig{end+1}.parameterName = 'Gyroscope_B';
% config.pluginConfig{end}.parameterSize = 3;