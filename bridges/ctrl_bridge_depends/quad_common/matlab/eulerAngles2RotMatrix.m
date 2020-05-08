function [R] = eulerAngles2RotMatrix(euler_angles)
% Computes a rotation matrix from euler angles using the y-y-x convention.
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
% euler_angles = [roll pitch yaw]'

roll = euler_angles(1);
pitch = euler_angles(2);
yaw = euler_angles(3);

R = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)];

end

