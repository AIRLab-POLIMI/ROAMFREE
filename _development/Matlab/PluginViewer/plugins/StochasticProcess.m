function StochasticProcess(area, globalConfig, pluginConfig)

% search for log files

F = dir(globalConfig.logPath);

toPlot = {};

for i=1:length(F)
    if strncmp(F(i).name, pluginConfig.parameterName, length(pluginConfig.parameterName))
        if ~strncmp(F(i).name(end-7:end),'proc.log',8) && ~strcmp(F(i).name(end-3:end),'.txt')
            toPlot{end+1} = F(i).name;
        end
    end
end

if length(toPlot) == 0
    return
end

% load data
for i=1:length(toPlot)
     pf = [globalConfig.logPath toPlot{i}];
    
     par = sortByT(stubbornLoad(pf));

     toPlotRaw{i} = par(:,[1 3:2+pluginConfig.parameterSize]);
end

% select time vector
it = 1;

for i=1:length(toPlotRaw)
    if size(toPlotRaw{i},1) > size(toPlotRaw{it},1)
        it = i
    end
end

if size(toPlotRaw{it},1) == 1
    toPlotRaw{it} = repmat(toPlotRaw{it},2,1);
    toPlotRaw{it}(2,1) = toPlotRaw{it}(2,1)+eps;    
end

% generate plot data
t = toPlotRaw{it}(:,1);
y = zeros(length(t), pluginConfig.parameterSize+1);

for i=1:length(toPlotRaw)
    if i == it
        y(:,2:end) = y(:,2:end) + toPlotRaw{i}(:,2:end);
    else
        if (size(toPlotRaw(:,1)) == 1)
            y(:,2:end) = y(:,2:end) + repmat(toPlotRaw{i}(1,2:end),length(t),1);
        else
            y(:,2:end) = y(:,2:end) + interp1(toPlotRaw{i}(:,1),toPlotRaw{i}(:,2:end), t);
        end
    end
end

% plot
subplot('Position', squeezeArea(area,0.02))   
hold on

if isfield(pluginConfig, 'scale')
    y = y * pluginConfig.scale;
end

CO = get(gcf,'DefaultAxesColorOrder');

for i = 0:(pluginConfig.parameterSize-1)
    plot(t-t(1), y(:, 2+i), '-o', 'MarkerSize',3, 'Color', CO(i+1,:));

%     if par(1,3+pluginConfig.parameterSize) ~= Inf
%         pos = i*(pluginConfig.parameterSize+1)-i*(i+1)/2;
% 
%         plot(par(:,1)-par(1,1), par(:, 3+i)+sqrt(par(:,3+pluginConfig.parameterSize+pos)), 'LineStyle','--', 'Color', CO(i+1,:));
%         plot(par(:,1)-par(1,1), par(:, 3+i)-sqrt(par(:,3+pluginConfig.parameterSize+pos)), 'LineStyle','--', 'Color', CO(i+1,:));        
%     end   
end

axis tight

grid on

%generate title
l = [];
for i=1:length(toPlot)
    l = [l toPlot{i}(length(pluginConfig.parameterName)+1:end-4) '+'];
end
l=l(1:end-1);

title(strrep([pluginConfig.parameterName '(' l ')'], '_','\_'))

hold off


end

