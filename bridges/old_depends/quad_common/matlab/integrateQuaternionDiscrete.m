function [q_next] = integrateQuaternionDiscrete(q_current, omega, dt)
% Integrates the quaternion q_current over a time step of dt assuming
% constant body rates omega, i.e. it performs a zeroth order integration.
% A quaternion is defined as q = [qw qx qy qz]' where qw is the real part.

omega_norm = norm(omega);

if (omega_norm > 1e-4)
    [ ~, Q_bar ] = quatQ([ 0; omega]);
    Lambda = 1/2*Q_bar;
    q_next = (eye(4)*cos(omega_norm*dt/2) + 2/omega_norm*Lambda*sin(omega_norm*dt/2))*q_current;
else
    q_next = q_current;
end

end

