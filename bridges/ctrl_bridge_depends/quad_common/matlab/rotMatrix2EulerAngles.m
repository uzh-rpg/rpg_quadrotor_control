function [euler_angles] = rotMatrix2EulerAngles(R)
% Computes the euler angles from a rotation matrix using the y-y-x convention.
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
% euler_angles = [roll pitch yaw]'

euler_angles = zeros(3,1);

euler_angles(1) = atan2(R(3, 2), R(3, 3));
euler_angles(2) = -asin(R(3, 1));
euler_angles(3) = atan2(R(2, 1), R(1, 1));

end

