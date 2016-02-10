% Calculate acceleration for planar 2-link arms, given a function for calculating joint torques
%
% in:
%      q     - joint angles
%      qdot  - joint velocities
%      tau   - joint torques
%      model - struct containing model parameters
%
% out:  
%     qddot  - acceleration
% 
function qddot = qddot_planar_2_link_arm ( q, qdot, tau, model )

% mass matrix
Mq = Mq_planar_2_link_arm ( q, model );

% coriolis/centripetal term
C = C_planar_2_link_arm   ( q, qdot, model );

% gravity
g = g_planar_2_link_arm   ( q, model );

% acceleration
qddot=Mq\(tau-C*qdot-g);

% add friction
fc = model.coulomb_friction;
fv = model.viscous_friction;
qddot = qddot - Mq\(fv*qdot + fc*sign(qdot));

