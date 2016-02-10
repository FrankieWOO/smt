% Calculate Coriolis/centripetal force matrix for planar 2-link arm.
%
% in:
%    q     - joint angles
%    qdot  - joint anglular velocities
%    model - struct containing model parameters
%
% out:  
%    C     - Coriolis/centripetal matrix
% 
function C = get_coriolis_matrix_planar_2_link_arm ( q, qdot, model )

L   = model.L;  % link lengths
M   = model.M;  % mass
Lg  = model.Lg; % center of gravity

% Coriolis matrix
C=M(2)*L(1)*Lg(2)*sin(q(2))*[-2*qdot(2),-qdot(2);...
	                            qdot(1),      0];
