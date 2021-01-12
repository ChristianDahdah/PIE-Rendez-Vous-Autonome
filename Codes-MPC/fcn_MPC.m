function U = fcn_MPC(X0, Xf, R0, Rf, Rp0, Rpf, output_sat, T, N, A, B)
% Function that returns the value of the next control step by using an MPC
% controller with the specified parameters
%
%  Example of state matrices:
%   A= [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
%       3*n0^2 0 0 0 2*n0 0; 0 0 0 -2*n0 0 0; 0 0 0 0 0 -n0^2];
%   B= [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
%   with: 
%        n0= sqrt(mu/R0^3)
%        mu= 398600.4418  (km^3/s^2)
%        R0= 400  (km) 
%
%  output_sat = [sat_min, sat_max]
%  X0 = [x0, y0, z0, vx0, vy0, zy0]
%  Xf = [xf, yf, zf, vxf, vyf, vzf]
%
% Rotation is NOT YET included in the function.

u_max = output_sat(2); u_min= output_sat(1);

x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z');
xp = SX.sym('xp'); yp = SX.sym('yp'); zp = SX.sym('zp');
states = [x;y;z;xp;yp;zp]; n_states = length(states);

ux = SX.sym('ux'); uy = SX.sym('uy'); uz = SX.sym('uz');
controls = [ux;uy;uz]; n_controls = length(controls);

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
%P_lyap= dlyap(A,Q);

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

%% MPC resolution
u0 = zeros(N,n_controls);  % two control inputs 
X0 = repmat(X0,1,N+1); % Initialization of the states

args.p   = [X0;Xf]; % set the values of the parameters vector - STATES
args.x0 = [reshape(X0',n_states*(N+1),1); reshape(u0',n_controls*N,1)]; % initial value of the optimization VARIABLES
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
        
% Get controls from the solution
U_vec = reshape(full(sol.x(n_states*(N+1)+1:end))',n_controls,N)';
U= U_vec(1,:); %I only take the first fcking control step

end