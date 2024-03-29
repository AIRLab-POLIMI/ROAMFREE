function GenericEdge( area, globalConfig, pluginConfig )

edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);

if exist(edgef, 'file')
    
    [edge, outcome] = stubbornLoad(edgef);
    
    if outcome == 1
        
        %% 
        if ~isfield(pluginConfig, 'errorOnly')
            pluginConfig.errorOnly = false;
        end
        
        edge = sortByT(edge);        

        %% plot error
        if (pluginConfig.measureSize > 0 && ~pluginConfig.errorOnly)
            subplot('Position', squeezeArea([area(1:3) area(4)*0.5],0.02))
        else
            subplot('Position', squeezeArea(area,0.02))
        end
        hold on
        
        err = edge(:,(23+pluginConfig.measureSize):(23+pluginConfig.measureSize+pluginConfig.errorSize-1));
                
        plot(edge(:,1)-(isfield(pluginConfig, 'relativeTime') && pluginConfig.relativeTime == true)*edge(1,1),err);
        
        yl = max(std(err));
        
        title([pluginConfig.sensorName ' res'], 'interpreter', 'none');        
        
        xlim([0, edge(end,1)-edge(1,1)])
        ylim([-3*yl-eps 3*yl+eps]);
        grid on        
        axis tight
        
        % possibly plot events
        if isfield(pluginConfig, 'events')
            for j = 1:length(pluginConfig.events)
                if (pluginConfig.events(j) >= edge(1,1) && pluginConfig.events(j) <= edge(end,1))                
                    plot( (pluginConfig.events(j) - -(isfield(pluginConfig, 'relativeTime') && pluginConfig.relativeTime == true)*edge(1,1))* [1 1], ylim(), 'Color', [0.4, 0.4, 0.4] );
                end
            end
        end     

        %% plot measure
        if (pluginConfig.measureSize > 0 && ~pluginConfig.errorOnly)         
            subplot('Position', squeezeArea([area(1) area(2)+area(4)*0.5, area(3) area(4)*0.5],0.02))
            
            meas = edge(:,23:(23+pluginConfig.measureSize-1));
            plot(edge(:,1)-(isfield(pluginConfig, 'relativeTime') && pluginConfig.relativeTime == true)*edge(1,1), meas);
            
%             title([pluginConfig.sensorName ' z']);
        
            axis tight           
            grid on
        end        
        
    end
    
end

end

