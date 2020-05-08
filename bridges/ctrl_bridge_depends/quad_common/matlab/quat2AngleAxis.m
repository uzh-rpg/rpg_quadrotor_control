function [ alpha, vector ] = quat2AngleAxis( q )
%QUAT2ANGLEAXIS q = [qw qx qy qz]'
    alpha = 2*acos( q(1) );
    vector = q(2:4)/sin(alpha/2);

end
