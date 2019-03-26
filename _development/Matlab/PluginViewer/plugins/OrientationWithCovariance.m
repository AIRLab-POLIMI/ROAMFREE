function Trajectory(area, globalConfig, pluginConfig)

%% load data
posef = [globalConfig.logPath 'PoseSE3(W).log'];
[x, flag] = stubbornLoad(posef);

if flag == 1
    
    x = sortByT(x);
    
    i = find(x(:,2) == max(x(:,2)));
    
    t0 = x(i(1),1);
    
    %% decide about drawing covarinace    
    plotStd = false;
    if isfield(pluginConfig, 'plotStd')
        plotStd = pluginConfig.plotStd;
        if isfield(pluginConfig, 'covarianceScaling')
            covScaling = pluginConfig.covarianceScaling;
        else
            covScaling = 1.0;
        end
    end
    
    %% decide about reference
    reference = false;
    if isfield(pluginConfig, 'referenceFile')
        reference = true;
        xref = load(pluginConfig.referenceFile);
        rpyref = rad2deg(quat2euler(xref(:, 5:8)));
    end
    
    rpy = rad2deg(quat2euler(x(:, 6:9)));

    if plotStd
        vars = x(:, 9+[16 19 21]);
        iv = find(vars(:,1) ~= inf);    
        stds = sqrt(vars(iv,:)*covScaling)*0.01; 
        
        qstds = [sqrt(1-sum(stds.^2,2)) stds];
        rpystd = rad2deg(quat2euler(qstds));
    end
    
    ttls = {'roll','pitch','yaw'};
    for j = 1:3        
        subplot('Position', squeezeArea([area(1) (j-1)*0.33 area(3) area(4)*0.33],0.02))
        hold on

        yyaxis left
        plot(x(i,1)-t0, rpy(i,j));
        plot(x(1:i(1),1)-t0, rpy(1:i(1),j),'k');
        
        if reference
            plot(xref(:,1)-t0, rpyref(:,j), '-', 'Color', [0.9290, 0.6940, 0.1250])
        end
        
        if plotStd
            yyaxis right
            plot(x(iv,1)-t0,rpystd(:,j),'--');
        end
        
        title(ttls{j})
        
        xlim(x([1 end],1)-t0);
        grid on
    end
end

end

