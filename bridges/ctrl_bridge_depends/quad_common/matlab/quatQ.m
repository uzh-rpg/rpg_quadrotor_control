function [ Q, Q_bar ] = quatQ( q )
%QUATQ Summary of this function goes here
%   Detailed explanation goes here

Q = [ q(1) -q(2) -q(3) -q(4);
      q(2)  q(1) -q(4)  q(3);
      q(3)  q(4)  q(1) -q(2);
      q(4) -q(3)  q(2)  q(1)];

Q_bar = [ q(1) -q(2) -q(3) -q(4);
          q(2)  q(1)  q(4) -q(3);
          q(3) -q(4)  q(1)  q(2);
          q(4)  q(3) -q(2)  q(1) ];

end

