function Trajectory(area, globalConfig, pluginConfig)

%% load data
posef = [globalConfig.logPath 'PoseSE3(W).log'];
[x, flag] = stubbornLoad(posef);

if flag == 1
    
    x = sortByT(x);
    
    i = find(x(:,2) == max(x(:,2)));
    
    %% decide about drawing covarinace    
    plotStd = false;
    if isfield(pluginConfig, 'plotStd')
        plotStd = pluginConfig.plotStd;
    end
    
    rpy = rad2deg(quat2euler(x(:, 6:9)));

    if plotStd
        vars = x(:, 9+[16 19 21]);
        iv = find(vars(:,1) ~= inf);    
        stds = sqrt(vars(iv,:))*0.01;    
    end
    
    ttls = {'roll','pitch','yaw'};
    for j = 1:3        
        subplot('Position', squeezeArea([area(1) (j-1)*0.33 area(3) area(4)*0.33],0.02))
        hold on

        yyaxis left
        plot(x(i,1), rpy(i,j),'m');
        plot(x(1:i(1),1), rpy(1:i(1),j),'k');
        
        if plotStd
            yyaxis right
            plot(x(iv,1),stds(:,j),'--');
        end
        
        title(ttls{j})
        
        xlim(x([1 end],1));
        grid on
    end
end

end

