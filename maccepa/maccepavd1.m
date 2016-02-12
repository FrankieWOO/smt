clear all

% add path
curPath = pwd;
curPaths = strsplit(curPath,{'\','/'});
fatherPath = strjoin(curPaths(1:end-1),'/');
addpath(genpath(curPath));
addpath([fatherPath,'/simulate']);

% set up simulation
dt = .02;              % control frequency
N  = 400;              % no. samples
t  = (0:N-1)*dt;       % sample times
x0 = zeros(2,1);       % start state

% set up dynamics function
m  = model_maccepa('maccepa_model'); % model parameters
f = @(x, u) f_maccepa  ( x, u, m ); % dynamics function

% simulation parameter struct
p = [];
p.dt = dt;
p.solver = 'rk4';

% define feed forward commands
umax=m.umax;

u1=0.5*umax(1)*ones(1,N);

w=200*2*pi/N; % frequency
t=(0:N-1)*dt; % time
u2= .125*umax(2)*ones(1,N);

u3=0.05*ones(1,N);

U = [u1;u2;u3]; % concatenate the three phases

% simulate
X = simulate_feedforward ( x0, f, U, p );

% plot joint angles, velocities, commands
subplot(3,1,1),plot(X(1,:)'),ylabel('q (rad)')
subplot(3,1,2),plot(X(2,:)'),ylabel('dq/dt (rad/s)')
subplot(3,1,3),plot(U'),ylabel('u'),xlabel('t (s)')

% rm path
rmpath(genpath(curPath))
rmpath([fatherPath,'/simulate'])
