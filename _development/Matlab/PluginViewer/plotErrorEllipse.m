function h=plotErrorEllipse(x,y, logcov, col)

cov = [logcov([1 2]); logcov([3 4])];

e = eig(cov).^(0.5);

if (e(2)>e(1))
    e = [e(2); e(1)];
end

theta = 0.5 * atan2( 2*(mean([cov(1,2); cov(2,1)])), (cov(1,1)-cov(2,2)) );

ellipse(3*e(1),3*e(2),theta,x,y,col);

h=[e(1), e(2), theta];
