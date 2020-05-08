function [q_inv] = quatInverse(q)
% Takes the inverse of a quaternion.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

q_inv = [q(1); -q(2:4)];

end

