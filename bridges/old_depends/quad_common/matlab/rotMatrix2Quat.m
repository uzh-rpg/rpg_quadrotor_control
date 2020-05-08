% Return quaternion from rotation matrix
% q = [qw qx qy qz]' where qw is the real part.

function q = rotMatrix2Quat(M)

i = 1; j = 2; k = 3;
if M(2,2) > M(1,1)
    i = 2; j = 3; k = 1;
end

if M(3,3) > M(i,i)
    i = 3; j = 1; k = 2;
end

t = M(i,i) -(M(j,j) + M(k,k)) + 1;
q = [1 0 0 0]';
if (t > 1e-10)
    q(1) = M(k,j) - M(j,k);
    q(i+1) = t;
    q(j+1) = M(i,j) + M(j,i);
    q(k+1) = M(k,i) + M(i,k);
    q = q * 0.5 / sqrt(t);
end

q = q/norm(q);
