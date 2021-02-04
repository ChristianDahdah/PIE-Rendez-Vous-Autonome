function [X,dX,Theta,dTheta,u_cl] = final_approach_cylinder(sDT_i,T,N,radius)
% This function simulates the final approach to the  coordenates (0,0,0) with velocity,
% rotation, euler angles, etc equal to 0. 
% sDT_i -> initial position (a,b,b), (b,a,b) or (b,b,a) with a negative and b~0
% T -> time step
% N -> Prediction horizon
% radius -> approach cylinder radius

%This next line is to add the entire PIE-Rendez-Vous-Autonome folder and
%its subfolders to the matlab search path
addpath(genpath('/home/gaston/Desktop/Materias supaero/COS/PIE/PIE-Rendez-Vous-Autonome'))

%addpath('./CASADI') % for Linux users
import casadi.*

%% SIMULATION INPUT

%addpath('./models')
load linear_model.mat A B % State-space representation of the coupled 6 dof system
load parameters.mat mu rT w0 ICDC mC ITDT eulerDCDT_i omegaDCDT_i dsDT_i % Other parameters (inertia, constants) 


%% MPC Initialization - This has changed when the new model was introduced

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




%% From now on I do not care what model I use

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
st = X(:,1); % initial state
g = [];
g = [g;st-P(1:n_states)]; % initial condition constraints
for k = 1:N
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
    
a = sDT_i(1); b = sDT_i(2); c = sDT_i(3);

if max(abs(sDT_i)) == a
    for k=1:N+1
        g = [g; X(8,k)^2 + X(9,k)^2 - radius^2];
    end
elseif max(abs(sDT_i)) == b
    for k=1:N+1
        g = [g; X(7,k)^2 + X(9,k)^2 - radius^2];
    end
elseif max(abs(sDT_i)) == c
    for k=1:N+1
        g = [g; X(7,k)^2 + X(8,k)^2 - radius^2];
    end
end

    
% make the decision variables one column vector
OPT_variables = [reshape(X,n_states*(N+1),1);  reshape(U,n_controls*N,1)];
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0; %0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;
% equality constraints (state constraints)
args.lbg(1:n_states*(N+1)) = 0;  % equality constraints
args.ubg(1:n_states*(N+1)) = 0;   % "" 

%Cylinder constraints
args.lbg(n_states*(N+1)+1 : n_states*(N+1)+(N+1)) = -inf;
args.ubg(n_states*(N+1)+1 : n_states*(N+1)+(N+1)) = 0;

% State constraints (so that the spacecraft doesn't fuck off to space)
args.lbx(1:n_states*(N+1),1) = -10000;
args.ubx(1:n_states*(N+1),1) = 10000;
args.ubx(7:7:n_states*(N+1),1) = 1;
% input constraints
args.lbx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = -10000;
args.ubx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = 10000;

args.lbg = args.lbg';
args.ubg = args.ubg';

%% SIMULATION

t0 = 0;
x0 = [eulerDCDT_i;omegaDCDT_i; sDT_i; dsDT_i];    % initial condition.
xs = [eulerDCDT_ref;omegaDCDT_ref; sDT_ref; dsDT_ref]; % Reference posture.

% Try to preallocate xx and t with their final size, xx=zeros(...,...)
t(1) = t0;
xx(:,1) = x0; % xx contains the history of states
u0 = zeros(N,n_controls);  % two control inputs 
X0 = repmat(x0,1,N+1); % Initialization of the states

% Start MPC
mpciter = 0; xx1 = []; u_cl = [];

%% 
% the main simulaton loop... it works as long as the error is greater
% than xxx and the number of mpc steps is less than its maximum
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

X = xx(7:9,:);
dX = xx(10:12,:);
Theta = xx(1:3,:);
dTheta = xx(4:6,:);

end
   