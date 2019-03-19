function Trajectory(area, globalConfig, pluginConfig)

%% load data
posef = [globalConfig.logPath 'PoseSE3(W).log'];
[x, flag] = stubbornLoad(posef);

if flag == 1
    
    x = sortByT(x);
    
    i = find(x(:,2) == max(x(:,2)));

    t0 = x(i(1),1);
    
    %% decide about position shift    
    x0 = [0 0 0];    
    if isfield(pluginConfig, 'plotPositionRelativeToFirst')
        if pluginConfig.plotPositionRelativeToFirst == true            
           x0 = x(i(1),3:5);
        end
    end
    
    %% decide about drawing covarinace    
    plotStd = false;
    if isfield(pluginConfig, 'plotStd')
        plotStd = pluginConfig.plotStd;
    end
    
    %% decide about reference
    reference = false;
    if isfield(pluginConfig, 'referenceFile')
        reference = true;
        xref = load(pluginConfig.referenceFile);
    end
    
    if plotStd
        vars = x(:, 9+[1 7 12]);
        iv = find(vars(:,1) ~= inf);    
        stds = sqrt(vars(iv,:))*0.001;    
    end
    
    ttls = {'X','Y','Z'};
    for j = 1:3        
        subplot('Position', squeezeArea([area(1) (j-1)*0.33 area(3) area(4)*0.33],0.02))
        hold on

        yyaxis left
        plot(x(i,1)-t0, x(i,j+2)-x0(j));
        plot(x(1:i(1),1)-t0, x(1:i(1),j+2)-x0(j),'k');
        
        if reference
            plot(xref(:,1)-t0, xref(:,j+1)-x0(j), '-', 'Color', [0.9290, 0.6940, 0.1250])
        end
        
        if plotStd
            yyaxis right
            plot(x(iv,1)-t0,stds(:,j),'--');
        end
        
        title(ttls{j})
        
        xlim(x([1 end],1)-t0);
        grid on
    end
end

end

