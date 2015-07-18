function y = sortByT(x)

[v,i] = sort(x(:,1));

y = x(i,:);

end

