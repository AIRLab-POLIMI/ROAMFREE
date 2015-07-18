function Trajectory(area, globalConfig, pluginConfig)

%% initialize local gT variable

persistent gT;

if size(gT,1) == 0
    if isfield(pluginConfig, 'gtData')               
        gT = pluginConfig.gtData;        
    end
    
    if isfield(pluginConfig, 'gtFile')
        gT = load(pluginConfig.gtFile);
    end
end

%% load data
posef = sprintf('%s%s',globalConfig.logPath, 'PoseSE3(W).log');
[x, flag] = stubbornLoad(posef);

if flag == 1
    
    x = sortByT(x);

    %% plot

    subplot('Position', area)

    hold on

    i = find(x(:,2) == max(x(:,2)));

    plot3(x(i,3),x(i,4),x(i,5),'m');
    plot3(x(1:i(1),3),x(1:i(1),4),x(1:i(1),5),'k');

    for j = 1:20:size(x,1)    
      plotAxis(x(j,3:5)',x(j,6:9),pluginConfig.axesLenght)       
    end
    
    if size(gT,1) > 0
        igt = find(gT(:,1) >= x(1,1) & gT(:,1) <= x(end,1));
        if (size(igt,1) > 0)
            plot3(gT(igt,2), gT(igt,3), gT(igt,4),'-')   
        end
    end
    
    if isfield(pluginConfig, 'sensorName')
        edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);
        [e, flag] = stubbornLoad(edgef);

        if flag == 1
            plot3(e(:,23), e(:,24), e(:,25),'rx');
        end
    end

%     
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

