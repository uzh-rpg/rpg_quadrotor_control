function [q] = eulerAngles2Quat(euler_angles)
% Computes a unit quaternion from euler angles using the y-y-x convention.
% euler_angles = [roll pitch yaw]'
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

q = zeros(4,1);

r = euler_angles(1)/2.0;
p = euler_angles(2)/2.0;
y = euler_angles(3)/2.0;
q(1) = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);
q(2) = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);
q(3) = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);
q(4) = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);

end

