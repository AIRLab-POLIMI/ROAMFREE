function runPlugins(config)

%% create plugin areas

N = size(config.global.plugins,2);
base = linspace(0,1,N+1)';

if strcmp(config.global.layout,'vertical')
    A = [repmat(0,N,1) base(1:end-1) repmat([1 1/N],N,1)];
else
    A = [base(1:end-1) repmat(0,N,1) repmat([1/N 1],N,1)];
end

%%

clf

for i = 1:N
    f = str2func(config.global.plugins{i});    
    f(A(i, :),config.global, config.pluginConfig{i});    
end

end

