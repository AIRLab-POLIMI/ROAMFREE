function EuclideanFeatures( area, globalConfig, pluginConfig )

subplot('Position', squeezeArea(area,0.02))
hold on

title('Image meas residuals (px)')

F = dir(globalConfig.logPath);

measureSize = 2;
errorSize = 2;

legs = {};

for i = 1:length(F)            
    % assume it is a Euclidean feature edge log, get feature id
    F(i).name(length(pluginConfig.sensorName)+6:end);
    n = sscanf(F(i).name(length(pluginConfig.sensorName)+6:end), '%d');

    parf = sprintf('%s_feat%d.log', pluginConfig.sensorName, n);    

    if (strcmp(F(i).name, parf)== true)
        legs{end+1} = sprintf('%d',n);
        
        [edge, flag] = stubbornLoad([globalConfig.logPath parf]);

        if flag == 1 && size(edge,1) > 0 % sometimes I got an empty p
            
            err = edge(:,(23+measureSize):(23+measureSize+errorSize-1));            
            
            subplot('Position', squeezeArea([area(1:3) area(4)*0.5],0.02))
            title('x')
            hold on
            plot(edge(:,1), err(:,1), '.');
            
            subplot('Position', squeezeArea([area(1) area(2)+area(4)*0.5, area(3) area(4)*0.5],0.02))
            title('y')
            hold on
            plot(edge(:,1), err(:,2), '.');
            
            %d = sqrt(sum(err.^2,2));                
            %plot(edge(:,1), d, '.');
        end

    end            
end

axis tight
% legend(legs)

end

