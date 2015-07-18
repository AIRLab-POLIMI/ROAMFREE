function plotAxis(x, q, l)

R = l*quatrot(q);

axis = repmat(x,1,3) + R;
line([x(1) axis(1,1)],[x(2) axis(2,1)],[x(3) axis(3,1)],'Color', 'r');
line([x(1) axis(1,2)],[x(2) axis(2,2)],[x(3) axis(3,2)],'Color', 'g');
line([x(1) axis(1,3)],[x(2) axis(2,3)],[x(3) axis(3,3)],'Color', 'b');

end

