function EuclideanFeaturesStats( area, globalConfig, pluginConfig )

subplot('Position', squeezeArea(area,0.02))
hold on

F = dir(globalConfig.logPath);

measureSize = 2;
errorSize = 2;

err = [];

for i = 1:length(F)            
    % assume it is a Euclidean feature edge log, get feature id
    F(i).name(length(pluginConfig.sensorName)+6:end);
    n = sscanf(F(i).name(length(pluginConfig.sensorName)+6:end), '%d');
    
    parf = sprintf('%s_feat%d.log', pluginConfig.sensorName, n);    

    if (strcmp(F(i).name, parf) == true)
        [edge, flag] = stubbornLoad([globalConfig.logPath parf]);

        if flag == 1 && size(edge,1) > 0 % sometimes I got an empty p
            err = [err; edge(:,(23+measureSize):(23+measureSize+errorSize-1))];
        end
    end            
end

title('img obs residuals (px)')

errNorm = sqrt(sum(err.^2,2));
hist(errNorm, max(errNorm)/0.1 );

legend(sprintf('N = %d, mean = %.2f', length(errNorm), mean(errNorm)));

end

