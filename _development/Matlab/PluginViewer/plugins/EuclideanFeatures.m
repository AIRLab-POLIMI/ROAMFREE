function EuclideanFeatures( area, globalConfig, pluginConfig )

subplot('Position', squeezeArea(area,0.02))
hold on

F = dir(globalConfig.logPath);

measureSize = 3;
errorSize = 3;

for i = 1:length(F)            
    % assume it is a Euclidean feature edge log, get feature id
    F(i).name(length(pluginConfig.sensorName)+6:end);            
    n = sscanf(F(i).name(length(pluginConfig.sensorName)+6:end), '%d');

    parf = sprintf('%s_feat%d.log', pluginConfig.sensorName, n);          

    if (strcmp(F(i).name, parf)== true) 
        
        
        
        [edge, flag] = stubbornLoad([globalConfig.logPath parf]);

        if flag == 1 && size(edge,1) > 0 % sometimes I got an empty p
            
            err = edge(:,(23+measureSize):(23+measureSize+errorSize-1));
                
            plot(edge(:,1) - edge(1,1),err, '.');
        end

    end            
end

axis tight

end

