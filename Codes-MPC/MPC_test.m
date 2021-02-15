clear all, close all, clc

% Physical model
mu= 398600.4418; % en km^3/s^2
R0= 400; % en km 
n0= sqrt(mu/R0^3);
A= [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    3*n0^2 0 0 0 2*n0 0;
    0 0 0 -2*n0 0 0;
    0 0 0 0 0 -n0^2];
n_states = length(A);
B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
s = size(B);
n_controls = s(2);
output_sat = [0; 50];
t0 = 0;
x0 = [-80 ;-40 ; 0; 0; 0; 0];    % initial condition.
xf = [0 ; 0 ; 0 ; 0; 0; 0]; % Reference posture.

% Optimisation parameters
T = 0.1; % sampling time [s]
N = 500; % prediction horizon
Q = zeros(n_states,n_states);
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;
Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrice (precision/time)
R = zeros(n_controls,n_controls);
R(1,1) = 0.05; R(2,2) = 0.05; R(3,3) = 0.05; % weighing matrice (controls)
sim_tim = 20; % Maximum simulation time
mpciter_max = 150;
precision = 0.05;

% Preallocation
t(1) = t0;
xx(:,1) = x0; % xx contains the history of states
u0 = zeros(N,n_controls);  % two control inputs 
X0 = repmat(x0,1,N+1); % Initialization of the state

% Start MPC
mpciter = 0; xx1 = []; ucl = [];

main_loop = tic;
while(norm((x0-xf),2) > precision && mpciter < mpciter_max)
    u_next = fcn_MPC(x0, xf, output_sat, Q, R, T, N, A, B)
    ucl = [ucl ; u_next];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift_init(T, t0, x0, ucl,f); % get the initialization of the next optimization step
    xx(:,mpciter+2) = x0;
    % Get solution trajectoy
    X0 = reshape(full(sol.x(1:n_states*(N+1)))',n_states,N+1)';
    %Shift trajectory to initialize next step
    X0 = [X0(2:end,:); X0(end,:)];
    
    mpciter
    mpciter = mpciter + 1;
end
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
