function [ bodyrates ] = quaternionDeltaToBodyRates( q2, q1, dt )
%QUATERNIONRATERTOBODYRATES Computes the angular velocity in body 
% coordinate system for a rotation from q1 to q2 within the time dt

    q_e = quatMultiplication(quatInverse(q1), q2);
    if (q_e(1) < 0)
        q_e = -q_e;
    end

    a = sinc(acos(q_e(1)) / pi);

    alpha = [q_e(2) / a, q_e(3) / a, q_e(4) / a]';

    bodyrates = 2.0 * alpha / dt;
end

