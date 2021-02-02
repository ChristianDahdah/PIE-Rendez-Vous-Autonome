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
x0 = [-80 ;-40 ; 0; 0; 0; 0];    % initial condition
x = x0;
X = [x]; % History of states
xf = [0 ; 0 ; 0 ; 0; 0; 0]; % Reference posture
U = []; % History of control steps

% Environment simulation parameters
t = 0;
dt = 0.01;

% Optimisation parameters
T = 0.1; % sampling time [s]
N = 500; % prediction horizon
Q = zeros(n_states,n_states);
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;
Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrice (precision/time)
R = zeros(n_controls,n_controls);
R(1,1) = 0.05; R(2,2) = 0.05; R(3,3) = 0.05; % weighing matrice (controls)
mpciter_max = 200;
precision = 0.05;


mpciter = 0;

main_loop = tic;
while(norm((x-xf),2)>precision && mpciter<mpciter_max)
    
    % MPC control step
    if(t-mpciter*T>=0):
        x_mpc = NL2MPC(x);
        
        u = fcn_MPC(x_mpc, xf, output_sat, Q, R, T, N, A, B)
        
        mpciter
        mpciter = mpciter + 1;
    end
    
    U = [U u];
    
    % Environment propagation vith RK4
    k1 = nonlinearModel(x,u,t);
    k2 = nonlinearModel(x+k1*dt/2,u,t+dt/2);
    k3 = nonlinearModel(x+k2*dt/2,u,t+dt/2);
    k4 = nonlinearModel(x+k3*dt,u,t+dt);
    
    x = x + (dt/6)*(k1+2*k2+2*k3+k4);
    t = t + dt;
    X = [X x];
end
main_loop_time = toc(main_loop)

U = [U zeros(n_controls,1)];

simiter = (t/dt);

'Simulation iterations:'
simiter
'Simulated time:'
t
'Control steps taken:'
mpciter

figure
subplot(211)
plot(1:simiter+1,X(1,:),1:simiter+1,X(2,:),1:simiter+1,X(3,:)), grid on
title('MPC simulation over axes x, y and z','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('x,y,z','FontSize',15)
legend('x','y','z')

subplot(212)
plot(1:simiter,U(:,1),1:simiter,U(:,2),1:simiter,U(:,3)), grid on
title('Control singals','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('Control signals','FontSize',15)
legend('u_x','u_y','u_z')