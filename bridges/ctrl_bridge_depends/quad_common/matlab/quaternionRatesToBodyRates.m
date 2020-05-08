function [ bodyrates ] = quaternionRatesToBodyRates( q2, q1, dt )
%QUATERNIONRATERTOBODYRATES Computes the angular velocity in body 
% coordinate system for a rotation from q1 to q2 within the time dt


	% Note, that the space of unit quaternion S(3) double covers the space
	% of physical attitudes SO(3) therefore q = -q.
	% I want the minimal q_dot, therefore I need to check which is
	% better q or -q (they both represent the same attitude in space)

	if( norm(-q2 -q1) < norm(q2 -q1) )
		q2 = -q2;
    end
	q_dot = ( q2 - q1 ) /dt;

	% [ o W_omega ]' = 2q_dot * q_bar in world frame
	% [ o B_omega ]' = 2q_bar * q_dot in body frame

    bodyrates = 2*(  quatMultiplication( quatInverse(q2), q_dot ));
    bodyrates = bodyrates(2:4);       
end

