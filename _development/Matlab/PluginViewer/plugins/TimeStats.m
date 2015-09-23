function TimeStats(area, globalConfig, pluginConfig)

%load data
timef = sprintf('%s%s',globalConfig.logPath, 'timeStats.txt');
[x, flag] = stubbornLoad(timef);

subplot('Position', area)

plot(cumsum(x,2))

legend('afterInit', 'afterOptimize', 'afterCovariance', 'afterLog');

title('est time stats');

end

