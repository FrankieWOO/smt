% Calculate gravity force vector for planar 2-link ar given current joint angles.
%
% in:   
%    q     - joint angles
%    model - struct containing model parameters
%
% out:  
%     G    - gravity vector
% 
function G = get_gravity_vector_planar_2_link_arm ( q, model )

L   = model.L;  % link lengths
M   = model.M;  % mass
Lg  = model.Lg; % center of gravity
g   = model.g;

% gravity vector
G = -g*[(M(1)*Lg(1)+M(2)*L(1))*cos(q(1))+M(2)*Lg(2)*cos(q(1)+q(2));...
                                        +M(2)*Lg(2)*cos(q(1)+q(2))];

