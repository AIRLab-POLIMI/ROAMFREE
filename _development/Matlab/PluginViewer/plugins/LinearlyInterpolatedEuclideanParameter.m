function LinearlyInterpolatedEuclideanParameter(area, globalConfig, pluginConfig)

pf = sprintf('%s%s.log', globalConfig.logPath, pluginConfig.parameterName);

if exist(pf, 'file')
   par = sortByT(stubbornLoad(pf));
   
   subplot('Position', squeezeArea(area,0.02))   
   
   plot(par(:,1)-par(1,1), par(:, 3:2+pluginConfig.parameterSize), '-o');
   
   title(pluginConfig.parameterName)   
end

end

