% This script simulates a targer-chaser dynamic environment, without
% closed-loop architecture. Its purpose is to compare the behaviour of the
% nonlinear model with respect to the linearized model, to (i) validate the
% nonlinear code and (ii) conclude on the impact of linearization.

% IMPORTANT NOTE : To change the simulation parameters (e.g. inertia of the
% chaser, ...), the following function should be altered : CreateModel.m in the initialization folder.

% This code should NOT contain declarations of variables that may be re-used
% in other codes, such as inertias, orbital parameters, etc.


clearvars; close all; clc

addpath('../Codes-MPC');
addpath('../models');
%load('../models/fullmatrices.mat');

%% Import of the simulation parameters

% A and B matrices of the linearized model (working point : final situation
% with perfect docking)
load('../initialization/linear_model');

% Constants and Chaser, target parameters (change the
% initialization/CreateModel.m routine to change them):
load('../initialization/parameters');


%% MPC-specific parameters
output_sat = [0; 50]; 

% Reference variables (_ref subscript)
eulerDCDT_ref = [0;0;0]; % Relative euler angles between docking ports
omegaDCDT_ref = [0;0;0]; % Relative rotational speed between docking ports
sDT_ref = [0;0;0]; % Position vector of DC wrt DT, expressed in DT
dsDT_ref = [0;0;0]; % Relative speed between docking ports, expressed in DT



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
A = Ar;
B = Br;
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
        euler = quat2euler(x(1:4)); % Extraction of euler-angles information from simulation state (expressed in quaternions)
        euler = euler';
        
        x_mpc = [euler; x(5:13)];
        
        % add measurement noise to x_mpc here
        
        u = fcn_MPC(x_mpc, xf, output_sat, Q, R, T, N, A, B); % Next input to be used by the simulation until the next call to MPC (period T)
        u = u';
        
        mpciter
        mpciter = mpciter + 1;
    end
    
    U = [U u];
    
    % Environment propagation vith RK4
    k1 = P2P_DynCin(x,u);
    k2 = P2P_DynCin(x+k1*dt/2,u);
    k3 = P2P_DynCin(x+k2*dt/2,u);
    k4 = P2P_DynCin(x+k3*dt,u);
    
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