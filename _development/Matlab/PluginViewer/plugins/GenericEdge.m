function genericEdge( area, globalConfig, pluginConfig )

edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);

if exist(edgef, 'file')
    
    [edge, outcome] = stubbornLoad(edgef);
    
    if outcome == 1
        
        edge = sortByT(edge);        

        %% plot error
        if (pluginConfig.measureSize > 0)
            subplot('Position', squeezeArea([area(1:3) area(4)*0.5],0.02))
        else
            subplot('Position', squeezeArea(area,0.02))
        end

        plot(edge(:,1) - edge(1,1),[edge(:,23:(23+pluginConfig.errorSize-1))])

        axis tight

        %% plot measure
        if (pluginConfig.measureSize > 0)         
            subplot('Position', squeezeArea([area(1) area(2)+area(4)*0.5, area(3) area(4)*0.5],0.02))

            plot(edge(:,1) - edge(1,1),edge(:,(23+pluginConfig.measureSize):(23+pluginConfig.measureSize+pluginConfig.errorSize-1)))        
        end

        axis tight

        title(pluginConfig.sensorName);
        
    end
    
end

end

