function t = watchFile(config)

period = 0.5; %seconds between directory checks

cb = @(x,y)(hasChanged(x, @runPlugins, config));

t = timer('TimerFcn', cb, 'Period', period, 'executionmode', 'fixedrate');
start(t);

end

function s = hasChanged(t, callback, config)

persistent lasttime

[s,r] = unix(sprintf('stat -c %%y "%s%s"',config.global.logPath, 'PoseSE3(W).log'));
if s == 0 && length(r) > 0   
    curtime = str2double(r(18:30));

    if curtime ~= lasttime
        display('Files have changed')
        callback(config);
        pause(0.01);
    end

    lasttime = curtime;
end

global stopAll

if stopAll == 1
    display('Exiting')
    stop(t)
end
    
end