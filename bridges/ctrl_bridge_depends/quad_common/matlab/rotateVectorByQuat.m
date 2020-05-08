function [v_rot] = rotateVectorByQuat(v,q)
% Rotates a vector v by a quaternion q.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.
% The vector v is defined as v = [vx vy vz]'

[ Q, Q_bar ]= quatQ(q);

v_rot_hom = Q_bar'*Q*[0; v];

v_rot = v_rot_hom(2:4);

end

