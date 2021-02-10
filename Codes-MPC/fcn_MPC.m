function u_cl = fcn_MPC(x0)
% Function that takes the current state of the chaser as an input and gives you the
% next command step to reach the target as an output.
%
% The input vector must have the following column structure:
% 
% x0 = [eulerDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i]

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

if max(abs(sDT_i)) == abs(a)
    for k=1:N+1
        g = [g; X(8,k)^2 + X(9,k)^2 - radius^2];
    end
elseif max(abs(sDT_i)) == abs(b)
    for k=1:N+1
        g = [g; X(7,k)^2 + X(9,k)^2 - radius^2];
    end
elseif max(abs(sDT_i)) == abs(c)
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
%args.ubx(7:7:n_states*(N+1),1) = 1;

if max(abs(sDT_i)) == abs(a)
    args.lbx(10:10:n_states*(N+1),1)= -0.1;
    args.ubx(10:10:n_states*(N+1),1)= 0.1;
elseif max(abs(sDT_i)) == abs(b)
    args.lbx(11:11:n_states*(N+1),1)= -0.1;
    args.ubx(11:11:n_states*(N+1),1)= 0.1;
elseif max(abs(sDT_i)) == abs(c)
    args.lbx(12:12:n_states*(N+1),1)= -0.1;
    args.ubx(12:12:n_states*(N+1),1)= 0.1;
end

% input constraints
args.lbx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = -10000;
args.ubx(n_states*(N+1)+1:n_states*(N+1)+n_controls*N,1) = 10000;

args.lbg = args.lbg';
args.ubg = args.ubg';

%% Optimization
%% SIMULATION
xs = zeros(12,1);

% Try to preallocate xx and t with their final size, xx=zeros(...,...)
u0 = zeros(N,n_controls);
X0 = repmat(x0,1,N+1); % Initialization of the states

args.p   = [x0;xs]; % set the values of the parameters vector - STATES
args.x0 = [reshape(X0',n_states*(N+1),1); reshape(u0',n_controls*N,1)]; % initial value of the optimization VARIABLES
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
% Get controls from the solution
u = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';

u_cl= u(1,:); %I only take the first control step
end