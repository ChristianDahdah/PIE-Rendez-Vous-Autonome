% Port to port dynamics model derived in Pirat 2018
clear all, clc
close all;

%addpath('D:/Documents/3A SUPAERO/PIE/Github/CasadiMatlab')
%addpath('C:/CaSaDi')
addpath('./models')

import casadi.*

%% Dynamic model import
%addpath('D:/Documents/3A SUPAERO/PIE/Github/models')
addpath('./models')
load ./initialization/linear_model.mat % State-space representation of the coupled 6 dof system
load ./initialization/parameters.mat % Other parameters (inertia, constants) 

% To change the input data, modify and run the code
% initialization/CreateModel.m

%% Initial conditions (_i subscript)
% alphaDCDT_i = 0.2; betaDCDT_i = 0.1; gammaDCDT_i = 0; % DC -> DT initial Euler angles
% eulerDCDT_i = [alphaDCDT_i; betaDCDT_i; gammaDCDT_i];
% 
 sxDT_i = 10; syDT_i = 5; szDT_i = 0; % Initial chaser position wrt target (in target docking frame)
 sDT_i = [sxDT_i; syDT_i; szDT_i];
% 
% omegaDCDT_i = [0;0;0]; % Immobile chaser at t = 0
% dsDT_i = [0;0;0]; % Immobile target at t = 0

%% Reference variables (_ref subscript)
eulerDCDT_ref = [0;0;0]; % Relative euler angles between docking ports
omegaDCDT_ref = [0;0;0]; % No relative motion
sDT_ref = [0;0;0]; %  
dsDT_ref = [0;0;0];

%% Working point selection 
% mu= 398600.4418e9; % en m^3/s^2
% hT = 400e3; % en m
% rT= 6378e3+hT; % en m 
% 
% w0= sqrt(mu/rT^3);
% 
% mC = 12 ; % chaser mass in kg
% 
% % State initial values (ce have to give them the same names as in Pirat's
% % symbolic implementation)
% sxDT = sxDT_i; syDT = syDT_i; szDT =  szDT_i; % Initial chaser position wrt target (in target docking frame)
% alphaDCDT = alphaDCDT_i; betaDCDT = betaDCDT_i; gammaDCDT = gammaDCDT_i; % DC -> DT initial Euler angles
% 
% rxDCDC = .1; ryDCDC = .1; rzDCDC = .1;
% rxDTDT = .1; ryDTDT = .1; rzDTDT = .1;
% aDT0 = 50*pi/180; bDT0 = 50*pi/180; cDT0 = 50*pi/180; % Docking port orientation
% 
% ICDC11 = 2; ICDC22 = 4; ICDC33 = 6; % en kg.m^2 (refine these values)
% ITDT11 = 2; ITDT22 = 4; ITDT33 = 6;
% 
% 
% A  = eval(Ar);
% B = eval(Br);
% 
% 
% clear sxDT syDT szDT alphaDCDT betaDCDT gammaDCDT
%% MPC Initialization

T = 0.2; % sampling time [s]
N = 50; % prediction horizon 

u_max = 2; u_min= -0.1; % Obsolete
 
alphaDCDT = SX.sym('alphaDCDT'); betaDCDT  = SX.sym('betaDCDT'); gammaDCDT = SX.sym('gammaDCDT'); 
eulerDCDT = [alphaDCDT;betaDCDT;gammaDCDT];

wxDCDT = SX.sym('wxDCDT'); wyDCDT = SX.sym('wyDCDT'); wzDCDT = SX.sym('wzDCDT');
omegaDCDT = [wxDCDT;wyDCDT;wzDCDT];

sxDT = SX.sym('sxDT'); syDT = SX.sym('syDT'); szDT = SX.sym('szDT');
sDT = [sxDT;syDT;szDT];

dsxDT = SX.sym('dsxDT'); dsyDT = SX.sym('dsyDT'); dszDT = SX.sym('dszDT');
dsDT = [dsxDT;dsyDT;dszDT];

states = [eulerDCDT;omegaDCDT;sDT;dsDT]; n_states = length(states);

TxDC = SX.sym('TxDC'); TyDC = SX.sym('TyDC'); TzDC = SX.sym('TzDC');
TDC = [TxDC; TyDC; TzDC];

FxDC = SX.sym('FxDC'); FyDC = SX.sym('FyDC'); FzDC = SX.sym('FzDC');
FDC = [FxDC; FyDC; FzDC];

controls = [TDC;FDC]; n_controls = length(controls);



rhs= A*states + B*controls;

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial and the reference state of the chaser)

X = SX.sym('X',n_states,(N+1));
% A Matrix that represents the states over the optimization problem.

%Q = zeros(n_states,n_states);
Q = 10*eye(n_states); % TO REFINE
%Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10; % 
%Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrices (states)
%R = zeros(n_controls,n_controls);
R(1,1) = 0.5; R(2,2) = 0.5; R(3,3) = 0.5; % weighing matrices (controls)
R(4,4) = 0.05; R(5,5) = 0.05; R(6,6) = 0.05; % weighing matrices (controls)
%P_lyap= dlyap(A,Q);

obj = 0; % Objective function
%g = zeros(N+1,1);  % constraints vector
st = X(:,1); % initial state
g = [];
g = [g;st-P(1:n_states)]; % initial condition constraints

vdir = [0;0;1]; % Cone main axis => TODO redefine this

vdir = vdir/norm(vdir);

phi = 30;

for k = 1:N
    st = X(:,k); con= U(:,k);
    
    nn = st(7:9)'*st(7:9);
    q = st(7:9)'*vdir;
    d =  norm(st(7:9)-q*vdir);
    
    obj = obj + (st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con + (d/q > tand(30))*1e9; %calculate obj
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + T/2*k1, con); % new
    k3 = f(st + T/2*k2, con); % new
    k4 = f(st + T*k3, con); % new
    st_next_RK4=st +T/6*(k1 +2*k2 +2*k3 +k4); % new   
    g = [g; st_next-st_next_RK4]; %compute constraints, new
end





% for k=1:N
%     nn = X(7:9,k)'*X(7:9,k);
%     q  = X(7:9,k)'*vdir;
%     d =  norm(X(7:9,k)-q*vdir);
%     
%     
%     g = [g; d/q];
% end


    
    

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


% inequality constraints (state constraints)
% args.lbg(n_states*(N+1)+1:(n_states+1)*(N+1)-1) = 0;  % equality constraints
% args.ubg(n_states*(N+1)+1:(n_states+1)*(N+1)-1) = 0.5;   % "" 


% State constraints (so that the spacecraft doesn't fuck off to space)
args.lbx(1:n_states*(N+1),1) = -10000;
args.ubx(1:n_states*(N+1),1) = 10000;

% input constraints
args.lbx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = -10000;
args.ubx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = 10000;

%% SIMULATION

t0 = 0;
x0 = [eulerDCDT_i;omegaDCDT_i; sDT_i; dsDT_i];    % initial condition.


xs = [eulerDCDT_ref;omegaDCDT_ref; sDT_ref; dsDT_ref]; % Reference posture.

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
while(norm((x0-xs),2) > 0.005 && mpciter < 100)
    args.p   = [x0;xs]; % set the values of the parameters vector - STATES
    args.x0 = [reshape(X0',n_states*(N+1),1); reshape(u0',n_controls*N,1)]; % initial value of the optimization VARIABLES
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
    % Get controls from the solution
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';
    % Get trajectory from solution
    xx1(:,1:n_states,mpciter+1)= reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)'; 

    u_cl= [u_cl ; u(1,:)]; %I only take the first control step
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

%% STATES  (error = state value as the references are = 0)

figure
subplot(211)
plot(1:mpciter+1,xx(1,:), 'r');  hold on;
plot(1:mpciter+1,xx(2,:), 'b');  hold on;
plot(1:mpciter+1,xx(3,:), 'k');
grid on;
title('Relative Euler Angles $\mathbf\alpha^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('angle in rad','FontSize',15)
legend('\alpha', '\beta', '\gamma');

subplot(212)
plot(1:mpciter+1,xx(7,:),1:mpciter+1,xx(8,:),1:mpciter+1,xx(9,:)), grid on; hold on;
title('Relative position $s^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('distance in m','FontSize',15)
legend('x','y','z')


figure
subplot(211)
plot(1:mpciter+1,xx(4,:), 'r');  hold on;
plot(1:mpciter+1,xx(5,:), 'b');  hold on;
plot(1:mpciter+1,xx(6,:), 'k');  
grid on;
title('Relative angular velocity $\mathbf\omega^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('angular rate in rad/s','FontSize',15)
legend('\omega_x', '\omega_y', '\omega_z');

subplot(212)
plot(1:mpciter+1,xx(10,:), 'r');  hold on;
plot(1:mpciter+1,xx(11,:), 'b');  hold on;
plot(1:mpciter+1,xx(12,:), 'k');  
grid on;
title('Relative velocity $\mathbf v^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('velocity in m/s','FontSize',15)
legend('v_x', 'v_y', 'v_z');

%% Command plots

u_clp = [0 0 0 0 0 0; u_cl];

figure
subplot(211)
plot(1:mpciter+1,u_clp(:,1), 'r');  hold on;
plot(1:mpciter+1,u_clp(:,2), 'b');  hold on;
plot(1:mpciter+1,u_clp(:,3), 'k');
grid on;
title('Control torques $T^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('Torque in Nm','FontSize',15)
legend('T_x', 'T_y', 'T_z');

subplot(212)
plot(1:mpciter+1,xx(7,:),1:mpciter+1,xx(8,:),1:mpciter+1,xx(9,:)), grid on; hold on;
plot(1:mpciter+1,u_clp(:,4), 'r');  hold on;
plot(1:mpciter+1,u_clp(:,5), 'b');  hold on;
plot(1:mpciter+1,u_clp(:,6), 'k');
grid on;
title('Control forces $F^{d_cd_t}$', 'interpreter', 'latex', 'FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('Force in N','FontSize',15)
legend('F_x', 'F_y', 'F_z');

xrange = 0:0.01:max(xx(7,:));

%% 2D plot
figure;
plot(xx(7,:),xx(8,:), '*r'), grid on; hold on;
plot(xrange, tand(phi)*xrange, '-b'); hold on;
plot(xrange, -tand(phi)*xrange, '-b'); hold on;

% %% 2D plot x,y
% figure
%     plot(xs(1),xs(2),'ob'); hold on, grid on %objective
%     th=0:pi/50:2*pi; 
%     plot(xx(1,1),xx(2,1),'*r')
%     xlim([min(0,min(xx(1,:))) max(0,max(xx(1,:)))])
%     ylim([min(0,min(xx(2,:))) max(0,max(xx(2,:)))])
%     xlabel('x'), ylabel('y')
%     plot(xx(1,:),xx(2,:),'*r');

%% 3D plot     
figure
    plot3(xs(7),xs(8),xs(9),'ob'); hold on, grid on
    plot3(xx(7,1),xx(8,1),xx(9,1),'*r')    
    
    xlim([min(0,min(xx(7,:))) max(0,max(xx(7,:)))])
    ylim([min(0,min(xx(7,:))) max(0,max(xx(7,:)))])
    zlim([min(0,min(xx(7,:))) max(0,max(xx(7,:)))])
    
    
    %ylim([min(0,min(xx(8,:))) max(0,max(xx(8,:)))])
    %zlim([min(0,min(xx(9,:))) max(0,max(xx(9,:)))])
    xlabel('x'), ylabel('y'), zlabel('z')
    for i=2:length(xx(7,:))
      plot3(xx(7,i),xx(8,i),xx(9,i),'*r');
      %pause(0.1);
    end