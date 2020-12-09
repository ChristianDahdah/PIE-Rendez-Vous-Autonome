% Clohessey-Wiltshire-Hill Model
clear all, close all, clc

addpath('C:\Users\Usuario\Desktop\Materias supaero\COS\PIE\MPC\CASADI')
import casadi.*

T = 2; % sampling time [s]
N = 50; % prediction horizon 

u_max = 2; u_min= -2; % !!!!

x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z');
xp = SX.sym('xp'); yp = SX.sym('yp'); zp = SX.sym('zp');
states = [x;y;z;xp;yp;zp]; n_states = length(states);

ux = SX.sym('ux'); uy = SX.sym('uy'); uz = SX.sym('uz');
controls = [ux;uy;uz]; n_controls = length(controls);
mu= 398600.4418; % en km^3/s^2
R0= 400; % en km 
n0= sqrt(mu/R0^3);
A= [0 0 0 1 0 0; 
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    3*n0^2 0 0 0 2*n0 0;
    0 0 0 -2*n0 0 0;
    0 0 0 0 0 -n0^2];
B= [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
rhs= A*states + B*controls;

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial and the reference state of the chaser)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

% compute solution symbolically
X(:,1) = P(1:n_states); % initial state
for k = 1:N
    X(:,k+1)  = X(:,k)+ T*f(X(:,k),U(:,k));
end
% this function has the purpose of getting
% the optimal trajectory knowing the optimal solution
ff=Function('ff',{U,P},{X});

Q = zeros(n_states,n_states);
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;
Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrices (states)
R = zeros(n_controls,n_controls);
R(1,1) = 0.05; R(2,2) = 0.05; R(3,3) = 0.05; % weighing matrices (controls)
P_lyap= dlyap(A,Q);

obj = 0; % Objective function
% compute objective
for k=1:N
    obj = obj+(X(:,k)-P(n_states+1:n_states*2))'*Q*(X(:,k)-P(n_states+1:n_states*2)) + U(:,k)'*R*U(:,k); % calculate obj
end
% Add lyapunov thing !!!!!
% obj = obj+ (X(:,n_states)-P(n_states+1:n_states*2))'*P_lyap*(X(:,n_states)-P(n_states+1:n_states*2))

g = [];  % constraints vector
% compute constraints
%for k = 1:N+1
%    g = [g ; X(1,k)];   %state x
%    g = [g ; X(2,k)];   %state y
%end

% make the decision variables one column vector
OPT_variables = reshape(U,n_controls*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;
% inequality constraints (state constraints)
args.lbg = 0;  % lower bound of the states x and y
args.ubg = 0;   % upper bound of the states x and y 

% input constraints
args.lbx(1:n_controls*N,1) = u_min;
args.ubx(1:n_controls*N,1) = u_max;

%% SIMULATION

t0 = 0;
x0 = [-150 ; -15 ; -300; 0; 0; 0];    % initial condition.
xs = [0 ; 0 ; 0 ; 0; 0; 0]; % Reference posture.

% Try to preallocate xx and t with their final size, xx=zeros(...,...)
t(1) = t0;
xx(:,1) = x0; % xx contains the history of states
u0 = zeros(N,n_controls);  % two control inputs 

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0; xx1 = []; u_cl = [];

%% 
%%%%%% ON S'EST ARRETÉ LÀ


% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0-xs),2) > 0.05 && mpciter < 100)
    args.p   = [x0;xs]; % set the values of the parameters vector - STATES
    args.x0 = reshape(u0',n_controls*N,1); % initial value of the optimization VARIABLES
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',n_controls,N)';
    ff_value = ff(u',args.p); % compute OPTIMAL solution TRAJECTORY
    xx1(:,1:n_states,mpciter+1)= full(ff_value)'; %Stores in every mpciter, the entire estimated trajectory for that iteration
    
    u_cl= [u_cl ; u(1,:)]; %I only take the first fcking control step
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift_init(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop)

subplot(211)
plot(1:mpciter+1,xx(1,:),1:mpciter+1,xx(2,:),1:mpciter+1,xx(3,:)), grid on
title('MPC simulation over axes x, y and z','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('x,y,z','FontSize',15)
legend('x','y','z')

subplot(212)
plot(1:mpciter,u_cl(:,1),1:mpciter,u_cl(:,2),1:mpciter,u_cl(:,3)), grid on
title('Control singals','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('Control signals','FontSize',15)
legend('u_x','u_y','u_z')

    figure
    plot3(xx(1,1),xx(2,1),xx(3,1),'*r'); hold on; grid on
    plot3(xs(1),xs(2),xs(3),'ob')
    xlim([min(xx(1,:)) max(xx(1,:))])
    ylim([min(xx(2,:)) max(xx(2,:))])
    zlim([min(xx(3,:)) max(xx(3,:))])
    xlabel('x'), ylabel('y'), zlabel('z')
    for i=2:length(xx(1,:))
      plot3(xx(1,i),xx(2,i),xx(3,i),'*r');
      pause(0.1);
    end