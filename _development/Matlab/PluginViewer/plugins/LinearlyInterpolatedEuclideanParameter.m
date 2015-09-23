function LinearlyInterpolatedEuclideanParameter(area, globalConfig, pluginConfig)

pf = sprintf('%s%s.log', globalConfig.logPath, pluginConfig.parameterName);

if exist(pf, 'file')
   par = sortByT(stubbornLoad(pf));
   
   subplot('Position', squeezeArea(area,0.02))   
   hold on
   
   plot(par(:,1)-par(1,1), par(:, 3:2+pluginConfig.parameterSize), '-o');
   
   if strcmp(pluginConfig.parameterName(end-5:end),'_Ba_GM') == 1              
       ihndlf = [globalConfig.logPath pluginConfig.parameterName(1:end-6) '.log'];
       
       ihndl = sortByT(stubbornLoad(ihndlf));
       
       plot(ihndl(:,1)-par(1,1), ihndl(:, [13:15]+22), '--');   
   end
   
   if strcmp(pluginConfig.parameterName(end-2:end),'_Bw') == 1
       
       ihndlf = [globalConfig.logPath pluginConfig.parameterName(1:end-3) '.log'];
       
       ihndl = sortByT(stubbornLoad(ihndlf));
       
       plot(ihndl(:, [25:27]+22), '--');   
   end 
   
   title(pluginConfig.parameterName)   
   
   hold off
end

end

