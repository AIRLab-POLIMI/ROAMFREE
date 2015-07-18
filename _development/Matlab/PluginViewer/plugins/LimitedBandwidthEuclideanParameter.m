function LimitedBandwidthEuclideanParameter(area, globalConfig, pluginConfig)

pf = sprintf('%s%s.log', globalConfig.logPath, pluginConfig.parameterName);

if exist(pf, 'file')
   par = sortByT(stubbornLoad(pf));
   
   subplot('Position', squeezeArea(area,0.02))   
   
   dt = 0.01;
   a = pluginConfig.a;
   bw = 1/2/(par(2,1)-par(1,1));
      
   %create the lanczos window
   t = -a:(dt*2*bw):a;    
   L = a*sin(pi*t).*sin(pi*t/a)/pi^2./t.^2;
   it0 = find(t >= 0,1);  
   L(isnan(L)) = 1;

   %do the convolution
   T = par(1,1)-a/2/bw:dt:par(end,1)+a/2/bw;
   if (T(end) ~= par(end,1)+a/2/bw)
       T(end+1) = par(end,1)+a/2/bw;
   end
   
   P = zeros(length(T),pluginConfig.parameterSize);
   
   for i = 1:size(par,1)
       itp = find(T >=par(i,1),1);
       
       for l = 1:pluginConfig.parameterSize
        P(itp-it0+1:itp+length(t)-it0,l) = P(itp-it0+1:itp+length(t)-it0,l) + L'*par(i,2+l);
       end
   end
   
   hold on

%    i = find (T>=par(1+a,1) & T<=par(end-a,1));
%    plot(T(i), P(i,: ));   
%    plot(par(1+a:end-a,1), par(1+a:end-a,3:2+pluginConfig.parameterSize), 'o');

   plot(T, P);
   plot(par(:,1), par(:, 3:2+pluginConfig.parameterSize), 'o');
   
   title(pluginConfig.parameterName)   
   

end

end

