function [l, flag] = stubbornLoad( file )

flag = 0;
cnt = 0;

l = [];

while (flag == 0 && cnt < 50)

    try
        l = load(file);
        %l(:,1) = l(:,1);
        flag = 1;
    catch err
        display(sprintf('retrying to load %s', file))    
        cnt = cnt + 1;
    end    
end

if flag == 0
    display(sprintf('something wrong with %s', file))
end

