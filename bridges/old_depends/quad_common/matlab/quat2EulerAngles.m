function [euler_angles] = quat2EulerAngles(q)
% Computes the euler angles from a unit quaternion using the y-y-x convention.
% euler_angles = [roll pitch yaw]'
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

euler_angles = zeros(3,1);

euler_angles(1) = atan2(2*q(1)*q(2) + 2*q(3)*q(4), q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4));
euler_angles(2) = -asin(2*q(2)*q(4) - 2*q(1)*q(3));
euler_angles(3) = atan2(2*q(1)*q(4) + 2*q(2)*q(3), q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4));

end

