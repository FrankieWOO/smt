% Calculate mass matrix for planar 2-link arm.
%
% in:
%      q     - joint angles
%      model - struct containing model parameters
%
% out:  
%      M     - mass matrix
% 
function M = get_mass_matrix_planar_2_link_arm ( q, model )

I   = model.I;  % inertia
L   = model.L;  % link lengths
M   = model.M;  % mass
Lg  = model.Lg; % center of gravity

% mass matrix
M =[I(1)+I(2)+M(2)*(L(1)^2)+2*M(2)*L(1)*Lg(2)*cos(q(2)), I(2)+M(2)*L(1)*Lg(2)*cos(q(2));...
    I(2)+M(2)*L(1)*Lg(2)*cos(q(2))                     , I(2)                         ];

