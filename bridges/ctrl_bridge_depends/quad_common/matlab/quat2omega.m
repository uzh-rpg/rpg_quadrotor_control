function [ omega ] = quat2omega( q2, q1, dt )
%QUAT2OMEGA computes the angular rates omega (in world frame) for a transition from q2 to q1
% within dt. 

    q_dot = (q2-q1)/dt;
    q2_bar = [q2(1);-q2(2:4)];

    tmp = 2*quatMultiplication( q_dot, q2_bar ); % in world system
%     B = 2*quatMultiplication( q2_bar, q_dot ); % in Body system 
    
    omega = tmp(2:4);
end

