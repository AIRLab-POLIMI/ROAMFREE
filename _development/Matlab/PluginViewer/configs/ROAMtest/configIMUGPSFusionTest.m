
%% create config

config = struct();

ss = get(0,'MonitorPositions'); % gets screen coordinates
    
config.global.layout = 'horizontal';
%config.global.figureOuterPosition = [ss(1,1) ss(1,2) ss(1,3) ss(1,4)]; % monitor 1
config.global.figureOuterPosition = [ss(2,1) ss(2,2) ss(2,3) ss(2,4)]; % monitor 2
config.global.logPath = '/tmp/roamfree/';

config.global.plugins = {'Trajectory', 'GenericEdge', 'GenericEdge', 'GenericEdge', 'LinearlyInterpolatedEuclideanParameter'};

config.pluginConfig{1}.axesLenght = 0.5;
config.pluginConfig{1}.sensorName = 'GPS'; %display also measures from an AbsolutePosition sensor

config.pluginConfig{2}.sensorName = 'GPS';
config.pluginConfig{2}.errorSize = 3;
config.pluginConfig{2}.measureSize = 3;

config.pluginConfig{3}.sensorName = 'Accelerometer';
config.pluginConfig{3}.errorSize = 3;
config.pluginConfig{3}.measureSize = 3;

config.pluginConfig{4}.sensorName = 'Gyroscope';
config.pluginConfig{4}.errorSize = 4;
config.pluginConfig{4}.measureSize = 3;

config.pluginConfig{5}.parameterName = 'Accelerometer_B';
config.pluginConfig{5}.parameterSize = 3;