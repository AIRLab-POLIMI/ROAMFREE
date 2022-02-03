function CrossCovariances( area, globalConfig, pluginConfig )

subplot('Position', squeezeArea(area,0.02))
hold on

title('Cross covariances')

blocks = cell(length(pluginConfig.parameters),length(pluginConfig.parameters));
vert_num = zeros(length(pluginConfig.parameters),1);

for i = 1:length(pluginConfig.parameters)
    thsz = pluginConfig.parameters(i).size;
    
    % load
    tmp = load(sprintf('%s%s.log',globalConfig.logPath, pluginConfig.parameters(i).name));
    
    % get the estimate size
    es = size(tmp,2) - 2 - thsz - thsz*(thsz-1)/2;
    
    % cut away everything but covariance part
    tmp = tmp(:,(2+es+1):end);
    
    n_v_1 = size(tmp,1);
    vert_num(i) = n_v_1;
    % reconstruct full cov
    blocks{i,i} = zeros(thsz*n_v_1, thsz*n_v_1);
    
    % For Diagonal blocks
    
    for a=1:1:n_v_1
        
        for b=a:1:n_v_1
            
            if a==b
                tmp_block = zeros(thsz,thsz);
                h = 1;
                for r = 1:thsz
                    for c = r:thsz
                        tmp_block(r,c) = tmp(a,h);
                        tmp_block(c,r) = tmp(a,h);
                        h = h + 1;
                    end
                end
                blocks{i,i}(thsz*(a-1)+1:thsz*a,thsz*(b-1)+1:thsz*b) = tmp_block;
            else
                tmp_block =  load(strcat(globalConfig.logPath, pluginConfig.parameters(i).name,'_', pluginConfig.parameters(i).name,'(',num2str(a-1),',',num2str(b-1),').txt'));
                
                blocks{i,i}(thsz*(a-1)+1:thsz*a,thsz*(b-1)+1:thsz*b) = tmp_block;
                blocks{i,i}(thsz*(b-1)+1:thsz*b,thsz*(a-1)+1:thsz*a) = tmp_block';
                
            end
            
        end
        
    end
    
    
  % For Off-Diagonal Matrix  
    for j = i+1:length(pluginConfig.parameters)
        
        tmp_2 = load(sprintf('%s%s.log',globalConfig.logPath, pluginConfig.parameters(j).name));
        tvsz = pluginConfig.parameters(j).size;
        n_v_2 = size(tmp_2,1);
        
        blocks{i,j} = zeros(n_v_1*thsz,n_v_2*tvsz);
        
        for a=1:1:n_v_1
           
            for b=1:1:n_v_2
               
               cross_corr_file_name =  strcat(globalConfig.logPath, pluginConfig.parameters(i).name,'_', pluginConfig.parameters(j).name,'(',num2str(a-1),',',num2str(b-1),').txt');
                
               if ~exist(cross_corr_file_name, 'file')
                   cross_corr_file_name =  strcat(globalConfig.logPath, pluginConfig.parameters(j).name,'_', pluginConfig.parameters(i).name,'(',num2str(b-1),',',num2str(a-1),').txt');
                   tmp_block =  load(cross_corr_file_name); 
                   tmp_block =  tmp_block';
                   
               else
                   tmp_block =  load(cross_corr_file_name); 
               end
               
               blocks{i,j}(thsz*(a-1)+1:thsz*a,tvsz*(b-1)+1:tvsz*b) = tmp_block;
               blocks{j,i}(tvsz*(b-1)+1:tvsz*b,thsz*(a-1)+1:thsz*a) = tmp_block'; 
               
            end
            
        end
        
        
    end
    

end


% compute correlation matrix

cov = cell2mat(blocks);

nm = diag(diag(cov).^-0.5,0);
corr = abs( nm * cov * nm );

%% plot

subplot('Position', squeezeArea(area,0.05))

imshow(corr, 'initialMagnification', 'fit');

% generate labels

lbls = {};
for i = 1:length(pluginConfig.parameters)    
    id = sscanf(pluginConfig.parameters(i).name, 'Camera_feat%d_Lw');
    if ~isempty(id)
        lbls{end+1} = sprintf('%d',id);
    else
        idx = find(pluginConfig.parameters(i).name=='_',1,'last');
        lbls{end+1} = pluginConfig.parameters(i).name(idx+1:end);
    end
end

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

set(a, 'YTickLabelRotation', 90);
set(a, 'TickLabelInterpreter', 'none');
set(a, 'XTick', tks);
set(a, 'YTick', tks);
set(a, 'XTickLabel', lbls);
set(a, 'YTickLabel', lbls);

x = 0.5;
for i = 1:size(blocks,2)
    line(x*[1 1], ylim(), 'LineWidth', 1, 'Color', 'red');
    thsz = pluginConfig.parameters(i).size;
    x_mini = x;
    
    for k = 2:1:vert_num(i)
       x_mini = x_mini+ thsz;       
       line(x_mini*[1 1], ylim(), 'LineWidth', 1, 'Color', 'blue');
    end
        
    x = x+size(blocks{1,i},2);
    
end
line(x*[1 1], ylim(), 'LineWidth', 1, 'Color', 'red');

y = 0.5;
for i = 1:size(blocks,1)
    line(xlim(), y*[1 1], 'LineWidth', 1, 'Color', 'red');
    thsz = pluginConfig.parameters(i).size;
    y_mini = y;
    
    for k = 2:1:vert_num(i)
       y_mini = y_mini+ thsz;       
       line(xlim(), y_mini*[1 1], 'LineWidth', 1, 'Color', 'blue');
    end
    
    y = y+size(blocks{i,1},1);
end
line(xlim(), y*[1 1], 'LineWidth', 1, 'Color', 'red');

axis on

axis tight

end

