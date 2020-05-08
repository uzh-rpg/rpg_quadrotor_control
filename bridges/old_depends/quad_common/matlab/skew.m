function [ M ] = skew( v )
%SKEW Computes the skew symetric matrix M from a vector v
M = [ 0  -v(3)  v(2);
 v(3)   0 -v(1);
-v(2)  v(1)   0];

end

