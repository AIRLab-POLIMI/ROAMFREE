function Trajectory(area, globalConfig, pluginConfig)

%% initialize local gT variable

persistent gT;

if size(gT,1) == 0
    if isfield(pluginConfig, 'gtData')               
        gT = pluginConfig.gtData;        
    end
    
    if isfield(pluginConfig, 'gtFile')
        if exist(pluginConfig.gtFile, 'file')        
            gT = load(pluginConfig.gtFile);
        end
    end
end

%% load data
posef = [globalConfig.logPath 'PoseSE3(W).log'];
[x, flag] = stubbornLoad(posef);

if flag == 1
    
    
    x = sortByT(x);
    
    i = find(x(:,2) == max(x(:,2)));

    %% decide about position shift
    
    x0 = 0;
    y0 = 0;
    z0 = 0;
    
    if isfield(pluginConfig, 'plotPositionRelativeToFirst')
        if pluginConfig.plotPositionRelativeToFirst == true            
           x0 = x(i(1),3);
           y0 = x(i(1),4);
           z0 = x(i(1),5);        
        end
    end
    
    %% decide about orientation plot
    
    orstep = 20;
    if isfield(pluginConfig, 'drawOrientationEvery')
        orstep = pluginConfig.drawOrientationEvery;
    end
    
    %% plot

    subplot('Position', squeezeArea(area,0.02))
    hold on       

    plot3(x(i,3)-x0,x(i,4)-y0,x(i,5)-z0,'m');
    plot3(x(1:i(1),3)-x0,x(1:i(1),4)-y0,x(1:i(1),5)-z0,'k');

    for j = 1:orstep:size(x,1)
      plotAxis(x(j,3:5)'-[x0 y0 z0]',x(j,6:9),pluginConfig.axesLenght)       
    end
    
    if size(gT,1) > 0
        igt = find(gT(:,1) >= x(1,1) & gT(:,1) <= x(end,1));
        if (size(igt,1) > 0)
            plot3(gT(igt,2)-x0, gT(igt,3)-y0, gT(igt,4)-z0,'-')   
        end
    end
    
    if isfield(pluginConfig, 'absolutePositionSensor')        
        edgef = [globalConfig.logPath pluginConfig.absolutePositionSensor '.log'];
        
        if exist(edgef, 'file')        
            [e, flag] = stubbornLoad(edgef);

            if flag == 1
                plot3(e(:,23)-x0, e(:,24)-y0, e(:,25)-z0,'rx');

                %plot predicted
                plot3(e(:,23)+e(:,26)-x0, e(:,24)+e(:,27)-y0, e(:,25)+e(:,28)-z0,'bo');
            end
        end
    end
    
    if isfield(pluginConfig, 'euclideanFeatureSensors')
        F = dir(globalConfig.logPath);
        
        for i = 1:length(F)            
            % assume it is a _Lw log, get feature id
            F(i).name(length(pluginConfig.euclideanFeatureSensors)+6:end);            
            n = sscanf(F(i).name(length(pluginConfig.euclideanFeatureSensors)+6:end), '%d');
            
            parf = sprintf('%s_feat%d_Lw.log', pluginConfig.euclideanFeatureSensors, n);          
            
            if (strcmp(F(i).name, parf)== true)                 
                [p, flag] = stubbornLoad([globalConfig.logPath parf]);
                
                if flag == 1 && size(p,1) > 0 % sometimes I got an empty p
                    plot3(p(1,3),p(1,4),p(1,5),'b.');
                    %text(p(1,3),p(1,4),p(1,5),sprintf('%d',n));
                end
                
            end            
        end
    end

     
%     mx = min(x(i,3));
%     Mx = max(x(i,3));    
%     Lx = (Mx-mx)*[-0.25 1.25]+mx
%     xlim(Lx);
%     
%     my = min(x(i,4));
%     My = max(x(i,4));    
%     Ly = (My-my)*[-0.25 1.25]+my
%     ylim(Ly);    

    grid on
    axis equal
    
    hold off

end

end

