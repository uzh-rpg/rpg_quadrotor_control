%% Test the code
clear all
clc

deg2rad = @(x) x*(pi/180)

phi = deg2rad( 43 );
theta = deg2rad( 46 );
psi = deg2rad( 21 );
q_x =  [ cos( phi/2 ) sin( phi/2 )*[1 0 0] ]';
q_y =  [ cos( theta/2 ) sin( theta/2 )*[0 1 0] ]';
q_z =  [ cos( psi/2 ) sin( psi/2 )*[0 0 1] ]';

% test the quat2RotMatrix
R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]
quat2RotMatrix( q_x ) 

R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]
quat2RotMatrix( q_y )


R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1 ]
quat2RotMatrix( q_z )

% test the quatMultiplication
R_zyx = R_z* R_y * R_x
q_zyx = quatMultiplication( quatMultiplication( q_z, q_y ), q_x );
quat2RotMatrix( q_zyx ) 


% test rotMatrix2Quat
q_x_rec = rotMatrix2Quat(R_x);
norm(q_x - q_x_rec)

q_y_rec = rotMatrix2Quat(R_y);
norm(q_y - q_y_rec)

q_z_rec = rotMatrix2Quat(R_z);
norm(q_z - q_z_rec)

q_zyx_rec = rotMatrix2Quat(R_zyx);
norm(q_zyx - q_zyx_rec)


%%%%%
% compare against 
% 
% #include <iostream>
% #include <Eigen/Dense>
% #include "stdio.h"
% 
% using namespace std;
% 
% float deg2rad(float deg)
% {
% 	return deg* 0.0175;
% }
% 
% void printQ(Eigen::Quaternionf q)
% {
% 	printf( "w: %.3f\nx: %.3f\ny: %.3f\nz: %.3f\n\n", q.w(), q.x(), q.y(), q.z() );
% 	return;
% }
% 
% int main() {
% 
% 	float phi = deg2rad( 43 );
% 	float theta = deg2rad( 46 );
% 	float psi = deg2rad( 21 );
% 
% 
% 
% 	Eigen::Quaternionf q_x( Eigen::AngleAxisf(phi, Eigen::Vector3f(1, 0, 0) ) );
% 	Eigen::Quaternionf q_y( Eigen::AngleAxisf(theta, Eigen::Vector3f(0, 1, 0) ) );
% 	Eigen::Quaternionf q_z( Eigen::AngleAxisf(psi, Eigen::Vector3f(0, 0, 1) ) );
% 
% 	std::cout << q_x.toRotationMatrix() << std::endl << std::endl;
% 	std::cout << q_y.toRotationMatrix() << std::endl << std::endl;
% 	std::cout << q_z.toRotationMatrix() << std::endl << std::endl;
% 	std::cout << ( q_z * q_y *q_x ).toRotationMatrix() << std::endl << std::endl;
% 	return 0;
% }


% 
%         1         0         0
%         0  0.729982 -0.683466
%         0  0.683466  0.729982
% 
% 0.693111        0 0.720831
%        0        1        0
% -0.720831        0 0.693111
% 
%  0.933228 -0.359284         0
%  0.359284  0.933228         0
%         0         0         1
% 
%  0.646831  0.197497  0.736617
%  0.249023  0.858246 -0.448777
% -0.720831  0.473718  0.505959










