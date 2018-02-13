function [ q ] = slerp( q0, q1, alpha )
%SLERP computes the interpolation between to quaternions
%   http://en.wikipedia.org/wiki/Slerp
omega = acos( q0'*q1 );

q = sin( (1-alpha)*omega )/sin( omega )*q0 + sin( (alpha)*omega )/sin( omega )*q1; 
end

