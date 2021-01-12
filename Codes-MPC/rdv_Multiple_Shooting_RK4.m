% Clohessey-Wiltshire-Hill Model
clear all, clc

addpath('C:\Users\Usuario\Desktop\Materias supaero\COS\PIE\MPC\CASADI')
import casadi.*

T = 1; % sampling time [s]
N = 50; % prediction horizon 

u_max = 2; u_min= -0.1; % !!!!

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

Q = zeros(n_states,n_states);
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;
Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrices (states)
R = zeros(n_controls,n_controls);
R(1,1) = 0.05; R(2,2) = 0.05; R(3,3) = 0.05; % weighing matrices (controls)
P_lyap= dlyap(A,Q);

obj = 0; % Objective function
%g = zeros(N+1,1);  % constraints vector
st = X(:,1); % initial state
g = [];
g = [g;st-P(1:n_states)]; % initial condition constraints
for k = 1:N-1
    st = X(:,k); con= U(:,k);
    obj = obj + (st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con; %calculate obj
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + T/2*k1, con); % new
    k3 = f(st + T/2*k2, con); % new
    k4 = f(st + T*k3, con); % new
    st_next_RK4=st +T/6*(k1 +2*k2 +2*k3 +k4); % new   
    g = [g; st_next-st_next_RK4]; %compute constraints, new
end
    st = X(:,N); con= U(:,N);
    obj = obj + (st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con; %calculate obj
    st_next = X(:,N+1);
    k1 = f(st, con);   % new 
    k2 = f(st + T/2*k1, con); % new
    k3 = f(st + T/2*k2, con); % new
    k4 = f(st + T*k3, con); % new
    st_next_RK4=st +T/6*(k1 +2*k2 +2*k3 +k4); % new   
    g = [g; st_next-st_next_RK4]; %compute constraints, new
    

% make the decision variables one column vector
OPT_variables = [reshape(X,n_states*(N+1),1);  reshape(U,n_controls*N,1)];
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
args.lbg(1:n_states*(N+1)) = 0;  % equality constraints
args.ubg(1:n_states*(N+1)) = 0;   % "" 

% State constraints (so that the spacecraft doesn't fuck off to space)
args.lbx(1:n_states*(N+1),1) = -10000;
args.ubx(1:n_states*(N+1),1) = 10000;

% input constraints
args.lbx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = u_min;
args.ubx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = u_max;

%% SIMULATION

t0 = 0;
x0 = [-50 ;-20 ; 0; 0; 0; 0];    % initial condition.
xs = [0 ; 0 ; 0 ; 0; 0; 0]; % Reference posture.

% Try to preallocate xx and t with their final size, xx=zeros(...,...)
t(1) = t0;
xx(:,1) = x0; % xx contains the history of states
u0 = zeros(N,n_controls);  % two control inputs 
X0 = repmat(x0,1,N+1); % Initialization of the states

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0; xx1 = []; u_cl = [];

%% 
% the main simulaton loop... it works as long as the error is greater
% than 10^-2 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
while(norm((x0-xs),2) > 0.05 && mpciter < 100)
    args.p   = [x0;xs]; % set the values of the parameters vector - STATES
    args.x0 = [reshape(X0',n_states*(N+1),1); reshape(u0',n_controls*N,1)]; % initial value of the optimization VARIABLES
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
    % Get controls from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';
    % Get trajectory from solution
    xx1(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 

    u_cl= [u_cl ; u(1,:)]; %I only take the first fcking control step
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift_init(T, t0, x0, u,f); % get the initialization of the next optimization step
    xx(:,mpciter+2) = x0;
    % Get solution trajectoy
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
    %Shift trajectory to initialize next step
    X0 = [X0(2:end,:); X0(end,:)];
    
    mpciter
    mpciter = mpciter + 1;
end;
main_loop_time = toc(main_loop)

figure
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

%% 2D plot x,y
figure
    plot(xs(1),xs(2),'ob'); hold on, grid on %objective
    th=0:pi/50:2*pi; 
    plot(xx(1,1),xx(2,1),'*r')
    xlim([min(0,min(xx(1,:))) max(0,max(xx(1,:)))])
    ylim([min(0,min(xx(2,:))) max(0,max(xx(2,:)))])
    xlabel('x'), ylabel('y')
    plot(xx(1,:),xx(2,:),'*r');

%% 3D plot     
    figure
    plot3(xs(1),xs(2),xs(3),'ob'); hold on, grid on
    plot3(xx(1,1),xx(2,1),xx(3,1),'*r')
    xlim([min(0,min(xx(1,:))) max(0,max(xx(1,:)))])
    ylim([min(0,min(xx(2,:))) max(0,max(xx(2,:)))])
    zlim([min(0,min(xx(3,:))) max(0,max(xx(3,:)))])
    xlabel('x'), ylabel('y'), zlabel('z')
    for i=2:length(xx(1,:))
      plot3(xx(1,i),xx(2,i),xx(3,i),'*r');
      %pause(0.1);
    end