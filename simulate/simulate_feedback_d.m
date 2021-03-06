% Simulate trajectory under closed-loop feedback control, in discrete time
%
%    [x,u] = simulate_feedback_d ( x0, f, pi, p )
%
% in:
%    x0 - initial state
%    f  - (discrete time) dynamics (function handle: x_{n+1}=f(x_n,u_n))
%    pi - policy (function handle representing the policy u=pi(x))
%    p  - parameter struct containing:
%         p.N      - number of steps to simulate
% 
% out: 
%    x  - state trajectory 
%    u  - actions taken along the trajectory 
% 
function [x,u] = simulate_feedback_d ( x0, f, pi, p )

N = p.N;
x = nan(size(x0,1),N); x(:,1)=x0; % initialise x
u = nan(size(pi(x0),1),N-1);      % initialise u
for n=1:N-1
	u(:,n  ) = pi(x(:,n)); % get next action 
	x(:,n+1) = f(x(:,n),u(:,n)); % state update
end


