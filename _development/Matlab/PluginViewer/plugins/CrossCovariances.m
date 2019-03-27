function CrossCovariances( area, globalConfig, pluginConfig )

subplot('Position', squeezeArea(area,0.02))
hold on

title('Cross covariances')

blocks = cell(length(pluginConfig.parameters),length(pluginConfig.parameters));

for i = 1:length(pluginConfig.parameters)    
    thsz = pluginConfig.parameters(i).size;
    
    % load
    tmp = load(sprintf('%s%s.log',globalConfig.logPath, pluginConfig.parameters(i).name));
    tmp = tmp(3+thsz:end);
    
    % reconstruct full cov
    blocks{i,i} = zeros(thsz, thsz);    
    h = 1;
    for r = 1:thsz
        for c = r:thsz
            blocks{i,i}(r,c) = tmp(h);
            blocks{i,i}(c,r) = tmp(h);
            h = h + 1;
        end
    end           

    % cross diagonal blocks
    for j = i+1:length(pluginConfig.parameters)
        blocks{i,j} = load(sprintf('%s%s_%s.txt', globalConfig.logPath, pluginConfig.parameters(i).name, pluginConfig.parameters(j).name));
        blocks{j,i} = blocks{i,j}';
    end
end


% compute correlation matrix

cov = cell2mat(blocks);

nm = diag(diag(cov).^-0.5,0);
corr = nm * cov * nm;

%% plot 

subplot('Position', squeezeArea(area,0.05))

imshow(corr, 'initialMagnification', 'fit');

% generate ticks

tks = zeros(size(blocks,1),1);
mtks = zeros(size(blocks,1),1)+1;
mtks(1)=0.5;

cum = 1;
for i = 1:length(tks)
    tks(i) = (2*cum+size(blocks{1,i},2)-1)/2;
    cum = cum + size(blocks{1,i},2);
    mtks(i+1)=cum-1;
end

a = gca();
a.YTickLabelRotation = 90;
a.TickLabelInterpreter = 'none';
a.XTick = tks;
a.YTick = tks;
a.XTickLabel = {pluginConfig.parameters.name};
a.YTickLabel = {pluginConfig.parameters.name};

x = 0.5;
for i = 1:size(blocks,2)
   line(x*[1 1], ylim(), 'LineWidth', 2, 'Color', 'red');
   x = x+size(blocks{1,i},2);
end
line(x*[1 1], ylim(), 'LineWidth', 2, 'Color', 'red');

y = 0.5;
for i = 1:size(blocks,1)
   line(xlim(), y*[1 1], 'LineWidth', 2, 'Color', 'red');
   y = y+size(blocks{i,1},1);
end
line(xlim(), y*[1 1], 'LineWidth', 2, 'Color', 'red');

axis on

axis tight

end

