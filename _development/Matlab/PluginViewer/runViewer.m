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

%% run

global stopAll

stopAll = 1;
pause(1);   
stopAll = 0;

watchFile(config);

