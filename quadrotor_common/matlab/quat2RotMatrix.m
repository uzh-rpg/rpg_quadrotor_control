function [R] = quat2RotMatrix(q)
% Transforms a quaternion q into a rotation matrix.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

[ Q, Q_bar ]= quatQ(q);

%
% R_tmp = [ 1 0; 0 R(q) ] = Q_bar(q)'*Q(q)
%

R_tmp = Q_bar'*Q;
R = R_tmp(2:4,2:4);
end

% R = [ q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2, 2*(q(2)*q(3) + q(1)*q(4)),         2*(q(2)*q(4) - q(1)*q(3));
%       2*(q(2)*q(3) - q(1)*q(4)),         q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2, 2*(q(3)*q(4) + q(1)*q(2));
%       2*(q(2)*q(4) + q(1)*q(3)),         2*(q(3)*q(4) - q(1)*q(2)),         q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]';
