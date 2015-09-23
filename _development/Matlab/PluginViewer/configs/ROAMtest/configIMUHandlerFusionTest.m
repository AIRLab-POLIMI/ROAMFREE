
%% create config

config = struct();

ss = get(0,'MonitorPositions'); % gets screen coordinates
    
config.global.layout = 'horizontal';
%config.global.figureOuterPosition = [ss(1,1) ss(1,2) ss(1,3) ss(1,4)]; % monitor 1
config.global.figureOuterPosition = [ss(2,1) ss(2,2) ss(2,3) ss(2,4)]; % monitor 2
config.global.logPath = '/tmp/roamfree/';

config.global.plugins = {'Trajectory', 'GenericEdge', 'GenericEdge', 'GenericEdge', 'LinearlyInterpolatedEuclideanParameter', 'LinearlyInterpolatedEuclideanParameter'};

config.pluginConfig{1}.axesLenght = 0.5;
config.pluginConfig{end}.sensorName = 'GPS'; %display also measures from an AbsolutePosition sensor
 
config.pluginConfig{end+1}.sensorName = 'GPS';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 3;

config.pluginConfig{end+1}.sensorName = 'IMUintegralDeltaP';
config.pluginConfig{end}.errorSize = 3;
config.pluginConfig{end}.measureSize = 27;

config.pluginConfig{end+1}.sensorName = 'IMUintegralDeltaQ';
config.pluginConfig{end}.errorSize = 4;
config.pluginConfig{end}.measureSize = 16;

config.pluginConfig{end+1}.parameterName = 'IMUintegralDeltaP_Ba_GM';
config.pluginConfig{end}.parameterSize = 3;

config.pluginConfig{end+1}.parameterName = 'IMUintegralDeltaP_Ba_RC';
config.pluginConfig{end}.parameterSize = 3;
