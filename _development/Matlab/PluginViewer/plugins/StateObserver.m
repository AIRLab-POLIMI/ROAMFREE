function StateObserver(area, globalConfig, pluginConfig)

edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);

if exist(edgef, 'file')
    
    [edge, outcome] = stubbornLoad(edgef);
    
    if outcome == 1
        
        subplot('Position', squeezeArea(area,0.02))

        if strcmp(pluginConfig.stateComponent, 'x')
            range = 4:6;
        elseif strcmp(pluginConfig.stateComponent, 'q')
            range = 7:10;
        elseif strcmp(pluginConfig.stateComponent, 'v')
            range = 11:13;
        elseif strcmp(pluginConfig.stateComponent, 'w')
            range = 14:16;
        elseif strcmp(pluginConfig.stateComponent, 'a')
            range = 17:19;
        elseif strcmp(pluginConfig.stateComponent, 'alpha')
            range = 20:22;
        end

        plot(edge(:,1) - edge(1,1),[edge(:,range)])

        axis tight

        title(sprintf('%s %s',pluginConfig.sensorName, pluginConfig.stateComponent));
    end    
end
       

end

