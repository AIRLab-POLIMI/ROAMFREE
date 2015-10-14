function LinearlyInterpolatedEuclideanParameter(area, globalConfig, pluginConfig)

pf = sprintf('%s%s.log', globalConfig.logPath, pluginConfig.parameterName);

if exist(pf, 'file')
    par = sortByT(stubbornLoad(pf));

    subplot('Position', squeezeArea(area,0.02))   
    hold on

    CO = get(gcf,'DefaultAxesColorOrder');
    
    % for single vertices duplicate the first row
    if size(par,1) == 1 
        par(2,1) = par(1,1)+eps;
        par(2,2:end)=par(1,2:end);
    end

    for i = 0:(pluginConfig.parameterSize-1)
        plot(par(:,1)-par(1,1), par(:, 3+i), '-o', 'MarkerSize',3, 'Color', CO(i+1,:));

        if par(1,3+pluginConfig.parameterSize) ~= Inf
            pos = i*(pluginConfig.parameterSize+1)-i*(i+1)/2;

            plot(par(:,1)-par(1,1), par(:, 3+i)+sqrt(par(:,3+pluginConfig.parameterSize+pos)), 'LineStyle','--', 'Color', CO(i+1,:));
            plot(par(:,1)-par(1,1), par(:, 3+i)-sqrt(par(:,3+pluginConfig.parameterSize+pos)), 'LineStyle','--', 'Color', CO(i+1,:));        
        end   
    end
    
    axis tight

    grid on

    title(strrep(pluginConfig.parameterName,'_','\_'))

    hold off
end

end

