function genericEdge( area, globalConfig, pluginConfig )

edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);

if exist(edgef, 'file')
    
    [edge, outcome] = stubbornLoad(edgef);
    
    if outcome == 1
        
        %% 
        if ~isfield(pluginConfig, 'errorOnly')
            pluginConfig.errorOnly = false
        end
        
        edge = sortByT(edge);        

        %% plot error
        if (pluginConfig.measureSize > 0 && ~pluginConfig.errorOnly)
            subplot('Position', squeezeArea([area(1:3) area(4)*0.5],0.02))
        else
            subplot('Position', squeezeArea(area,0.02))
        end

        err = edge(:,(23+pluginConfig.measureSize):(23+pluginConfig.measureSize+pluginConfig.errorSize-1));
                
        plot(edge(:,1) - edge(1,1),err);
        
        yl = max(std(err));
        
        title([pluginConfig.sensorName ' residual']);        
        
        xlim([0, edge(end,1)-edge(1,1)])
        ylim([-3*yl 3*yl]);        
        
        %axis tight

        %% plot measure
        if (pluginConfig.measureSize > 0 && ~pluginConfig.errorOnly)         
            subplot('Position', squeezeArea([area(1) area(2)+area(4)*0.5, area(3) area(4)*0.5],0.02))
            
            plot(edge(:,1) - edge(1,1),[edge(:,23:(23+pluginConfig.errorSize-1))])
            
            title([pluginConfig.sensorName ' measurement']);
        
            axis tight
        end
    end
    
end

end

