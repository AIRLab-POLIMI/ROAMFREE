function genericEdge( area, globalConfig, pluginConfig )

edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.priorName);

if exist(edgef, 'file')
    
    [edge, outcome] = stubbornLoad(edgef);
    
    if outcome == 1
        
        edge = sortByT(edge);        

        %% plot error
        subplot('Position', squeezeArea(area,0.02))

        plot(edge(:,1) - edge(1,1),[edge(:,4:(4+pluginConfig.errorSize-1))])

        axis tight

        title(pluginConfig.priorName);        
    end
    
end

end

