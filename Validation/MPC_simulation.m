% This script simulates a targer-chaser dynamic environment for testing the MPC law.
% It relies on a more realistic, nonlinear version of the linear model of
% the chaser-target dynamics internaly used by the MPC law ; and it provide
% the simulated measurement used by the MPC law and enables to analyse its results.
% The environment should be simulated with a (much) smaller time step (dt)
% than the MPC time step (T).

% TO DO:
%   * add simulated measurement noise to the simulated measurements given to the MPC
%   * add environmental disturbances to the nonlinear model (atmosphericdrag, etc)
%   * add uncertainties and model errors to the nonlinear model (error on the position of the center of mass,..) to run robustness campains (monte carlo)
%   * define metrics to quantify the quality of the trajectory provided by the MPC law (energy consumed, time, distance to approach corridor, etc)

clear all, close all, clc

addpath('../Codes-MPC');
addpath('../models');

%% Overall simulation setup
output_sat = [0; 50];
x = [-80 ;-40 ; 0; 0; 0; 0];    % initial state
X = [x]; % History of states
xf = [0 ; 0 ; 0 ; 0; 0; 0]; % reference state
U = []; % History of control steps
t = 0; % reference simulation current time
dt = 0.01; % time step of progression of simulation time t (time step of numerical integration of nonlinear model)

%% MPC setup
% Linear model to be used by the MPC
mu= 398600.4418; % en km^3/s^2
R0= 400; % en km 
n0= sqrt(mu/R0^3);
A= [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    3*n0^2 0 0 0 2*n0 0;
    0 0 0 -2*n0 0 0;
    0 0 0 0 0 -n0^2];
B = [0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];
linear_model_order = length(A);
s = size(B);
nb_controls = s(2);

% Optimisation parameters for the MPC
T = 0.1; % sampling time [s]
N = 500; % prediction horizon
Q = zeros(linear_model_order,linear_model_order);
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;
Q(4,4) = 10; Q(5,5) = 10; Q(6,6) = 10; % weighing matrice (precision and duration)
R = zeros(nb_controls,nb_controls);
R(1,1) = 0.05; R(2,2) = 0.05; R(3,3) = 0.05; % weighing matrice (controls and energy)
mpciter_max = 500; % Maximum iteration of the MPC control, to avoid an infinite running time if the MPC law can't provide convergence
precision = 0.05; % Precision threshold beyond which we consider to have reached the goal and stop simulation

%% Main simulation loop (defines the simulation current time t, which increases by dt every loop run)
% Note : the simulation internaly relies on quaternions for the relative
% attitude, which are converted to euler angle only when calling the MPC
% function

mpciter = 0;
main_loop = tic;
while(norm((x-xf),2)>precision && mpciter<mpciter_max)
    
    % Determine next control step with MPC --- not at every simulation step : period of T based on the simulation current time t
    if((t-mpciter*T)>=0)
        x_mpc = x; % Extraction of euler-angles information from simulation state (expressed in quaternions)
        
        % add measurement noise to x_mpc here
        
        u = fcn_MPC(x_mpc, xf, output_sat, Q, R, T, N, A, B); % Next input to be used by the simulation until the next call to MPC (period T)
        u = u';
        
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

simiter = (t/dt); % Total number of simulation steps (used for the plots)

%% Results and plots
'Simulation iterations:'
simiter
'Simulated time:'
t
'Control steps taken:'
mpciter

figure
subplot(211)
plot(1:round(simiter)+1,X(1,:),1:round(simiter)+1,X(2,:),1:round(simiter)+1,X(3,:)), grid on
title('MPC simulation over axes x, y and z','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('x,y,z','FontSize',15)
legend('x','y','z')

subplot(212)
plot(1:round(simiter)+1,U(1,:),1:round(simiter)+1,U(2,:),1:round(simiter)+1,U(3,:)), grid on
title('Control singals','FontSize',18)
xlabel('iteration','FontSize',15)
ylabel('Control signals','FontSize',15)
legend('u_x','u_y','u_z')