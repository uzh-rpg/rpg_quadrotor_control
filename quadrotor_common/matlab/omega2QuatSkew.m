function [ skew ] = omega2QuatSkew( omega )
% Converts rates to a quaternion skew.
% q = [q_w q_x q_y q_z]'    omega = [omega_x omega_y omega_z]'

skew = [0       , -omega(1), -omega(2), -omega(3);...
        omega(1),  0       ,  omega(3), -omega(2);...
        omega(2), -omega(3),  0       ,  omega(1);...
        omega(3),  omega(2), -omega(1),  0        ];
end

