function ib = matchingTimes(a, b)


m = size(a,2); n = size(b,2);
[c,p] = sort([a,b]);
q = 1:m+n; q(p) = q;
t = cumsum(p>m);
r = 1:n; r(t(q(m+1:m+n))) = r;
s = t(q(1:m));
id = r(max(s,1));
iu = r(min(s+1,n));
[d,it] = min([abs(a-b(id));abs(b(iu)-a)]);
ib = id+(it-1).*(iu-id);

end

