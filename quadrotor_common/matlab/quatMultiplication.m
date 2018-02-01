function [mult] = quatMultiplication(q, p)
% Multiplies the two quaternions q*p in the indicated order.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.
[ Q, ~ ]= quatQ(q); 
  
mult = Q*p;

end

