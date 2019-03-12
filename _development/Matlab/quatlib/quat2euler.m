function attE = quat2euler( attQ )

N = size(attQ,1);
Qres = reshape(attQ', 4*N, 1);
Eres = quat2euler_raw(Qres);
attE = reshape(Eres, 3, N)';

end

function attE = quat2euler_raw( attQ )

n = size(attQ,1)/4;
id0 = 1:4:4*n;
idx = 2:4:4*n+1;
idy = 3:4:4*n+2;
idz = 4:4:4*n+3;

% Convert angles
q0q0 = attQ(id0,:).*attQ(id0,:);
q0q1 = attQ(id0,:).*attQ(idx,:);
q0q2 = attQ(id0,:).*attQ(idy,:);
q0q3 = attQ(id0,:).*attQ(idz,:);

q1q1 = attQ(idx,:).*attQ(idx,:);
q1q2 = attQ(idx,:).*attQ(idy,:);
q1q3 = attQ(idx,:).*attQ(idz,:);

q2q2 = attQ(idy,:).*attQ(idy,:);
q2q3 = attQ(idy,:).*attQ(idz,:);

q3q3 = attQ(idz,:).*attQ(idz,:);

attE = zeros(3*n,1);
attE(1:3:3*n,:) = atan2( 2*(q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3 );
attE(2:3:3*n+1,:) = -asin(2*(q1q3 - q0q2));
attE(3:3:3*n+2,:) = atan2(2*(q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);

end