addpath(genpath('../quatlib'), genpath('plugins'))
   
%% clear each plugin status

L = dir('plugins');
for i = 1:size(L,1)
    if L(i).isdir == 0        
        clear(L(i).name(1:end-2))
    end   
end

%% spawn figures 

close all
figure(1)
set(1,'OuterPosition', config.global.figureOuterPosition)

%% if a compressed log file is given 

if isfield(config.global, 'logFile')
    if ~exist(config.global.logFile, 'file')
        error('* Error: %s does not exist', config.global.logFile);        
    end
    
    mustUncompress = true;
    logFile = config.global.logFile;
    logDate = dir(config.global.logFile);
    logDate = logDate.datenum;
    
    if exist('/tmp/log', 'file') == 7
      if exist('/tmp/log/meta.mat', 'file') == 2
        oldLog = load('/tmp/log/meta.mat');
        
        if strcmp(oldLog.logFile, logFile) && logDate == oldLog.logDate
          fprintf('* uncompressed folder up to date\n');
          mustUncompress = false;
        end        
      end
    end

    if mustUncompress == true
      system('rm -rf /tmp/log');
      
      mkdir('/tmp/log');
      fprintf('* uncompressing ... ');
      system(['tar xzf ''' config.global.logFile ''' -C /tmp/log']);
      fprintf('done\n');
      
      save('/tmp/log/meta.mat', 'logFile', 'logDate');
    end
    
    clear('logFile', 'logDate', 'mustUncompress')

    config.global.logPath = '/tmp/log/';
    
    runPlugins(config)
else
    global stopAll

    stopAll = 1;
    pause(1);   
    stopAll = 0;

    watchFile(config);
end
    