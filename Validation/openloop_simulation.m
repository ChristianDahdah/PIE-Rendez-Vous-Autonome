% This script simulates a targer-chaser dynamic environment, without
% closed-loop architecture. Its purpose is to compare the behaviour of the
% nonlinear model with respect to the linearized model, to (i) validate the
% nonlinear code and (ii) conclude on the impact of linearization.

% IMPORTANT NOTE : To change the simulation parameters (e.g. inertia of the
% chaser, ...), the following function should be altered : CreateModel.m in the initialization folder.

% This code should NOT contain declarations of variables that may be re-used
% in other codes, such as inertias, orbital parameters, etc.


clearvars; close all; format long;

addpath('./Codes-MPC');
addpath('./models');
addpath('./subfunctions');
addpath('./initialization');
%load('../models/fullmatrices.mat');

%% Import of the simulation parameters

% A and B matrices of the linearized model (working point : final situation
% with perfect docking)
load('./initialization/linear_model');

% Constants and Chaser, target parameters (change the
% initialization/CreateModel.m routine to change them):
load('./initialization/parameters');



dt = 0.05; % time step in seconds
Tmax = 50; % max. simulation time in seconds

t = 0:dt:Tmax;

nStates = 13;
nControls = 6;

QDCDT_i = euler2quat(eulerDCDT_i); 


XX_ss = zeros(nStates-1,length(t)); %% States history of the LINEAR SIMULATION (one less state than the non-linear as a quaternion is not used)
XX_nl = zeros(nStates,length(t)); %% States history of the NON-LINEAR SIMULATION
XX_nl_nq = zeros(nStates-1,length(t)); 

UU = zeros(nControls, length(t));


XX_ss(:,1) = [eulerDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i ];
%XX_nl(:,1) = [QDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i ];
XX_nl_nq(:,1) =  [eulerDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i ];
XX_nl(:,1) =  [QDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i ];
for i = 1:length(t)-1
    
    linear_fun= @(x,u)A*x+B*u;
    
    XX_ss(:,i+1) = RK4(linear_fun, dt, XX_ss(:,i), UU(:,i));
    
    
    XX_nl(:,i+1) = RK4(@P2P_DynCin, dt, XX_nl(:,i), UU(:,i));
    
    XX_nl_nq(:,i+1) = RK4(@P2P_DynCin_noquaternions, dt, XX_nl_nq(:,i), UU(:,i));
    
end

% We extract the positions
sX_ss = XX_ss(7:9,:);
%sX_nl = XX_nl(8:10,:);
sX_nl_debug = XX_nl (8:10,:);
sX_nl_nq = XX_nl_nq(7:9,:);


% We plot the positions
figure()

subplot(3,1,1)

plot(t,sX_ss(1,:), 'r'); hold on;
plot(t,sX_nl_debug(1,:), 'b'); hold on;
plot(t,sX_nl_nq(1,:), 'm');

xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('$X^{t}$ [m]' ,'interpreter', 'latex', 'fontsize', 13);
title('Relative position vector $s^{d_cd_t}$, free motion','interpreter', 'latex', 'fontsize', 13)
grid on;


subplot(3,1,2)
plot(t,sX_ss(2,:), 'r');
hold on;
plot(t,sX_nl_debug(2,:), 'b');
hold on;
plot(t,sX_nl_nq(2,:), 'm');
grid on;
xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('$Y^{t}$ [m]' ,'interpreter', 'latex', 'fontsize', 13);


subplot(3,1,3)
plot(t,sX_ss(3,:), 'r');
hold on;
plot(t,sX_nl_debug(3,:), 'b');
hold on;
plot(t,sX_nl_nq(3,:), 'm');
grid on;
xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('$Z^{t}$ [m]' ,'interpreter', 'latex', 'fontsize', 13);


legend('linear model', 'non-linear model')

% We extract the positions
eulerDCDT_ss = XX_ss(1:3,:);
eulerDCDT_nl_debug = zeros(size(eulerDCDT_ss));

for k = 1:length(XX_nl)
    
    eulerDCDT_nl_debug(1:3,k)  = quat2euler(XX_nl(1:4,k));
    
end

%eulerDCDT_nl_debug =  XX_nl(1:3,:);

eulerDCDT_nl_nq =  XX_nl_nq(1:3,:);

% We plot the positions

figure()
subplot(3,1,1)

plot(t,eulerDCDT_ss(1,:), 'r');
hold on;
plot(t,eulerDCDT_nl_debug(1,:), 'b');
hold on;
plot(t,eulerDCDT_nl_nq(1,:), 'm');
title('Relative Euler angles (XYZ seq.), free motion','interpreter', 'latex', 'fontsize', 13)
xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('X angle [rad]' ,'interpreter', 'latex', 'fontsize', 13);
grid on;

subplot(3,1,2)
plot(t,eulerDCDT_ss(2,:), 'r');
hold on;
plot(t,eulerDCDT_nl_debug(2,:), 'b');
hold on;
plot(t,eulerDCDT_nl_nq(2,:), 'm');
hold on;
xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('Y angle [rad]' ,'interpreter', 'latex', 'fontsize', 13);
grid on;

subplot(3,1,3)
plot(t,eulerDCDT_ss(3,:), 'r');
hold on;
plot(t,eulerDCDT_nl_debug(3,:), 'b');
hold on;
plot(t,eulerDCDT_nl_nq(3,:), 'm');
hold on;
xlabel('Time [s]','interpreter', 'latex', 'fontsize', 13);
ylabel('Z angle [rad]' ,'interpreter', 'latex', 'fontsize', 13);
grid on;

legend('linear model', 'non-linear model, quat', 'non-linear model, euler')





% 
% %% Results and plots
% 'Simulation iterations:'
% simiter
% 'Simulated time:'
% t
% 'Control steps taken:'
% mpciter
% 
% figure
% subplot(211)
% plot(1:round(simiter)+1,X(1,:),1:round(simiter)+1,X(2,:),1:round(simiter)+1,X(3,:)), grid on
% title('MPC simulation over axes x, y and z','FontSize',18)
% xlabel('iteration','FontSize',15)
% ylabel('x,y,z','FontSize',15)
% legend('x','y','z')
% 
% subplot(212)
% plot(1:round(simiter)+1,U(1,:),1:round(simiter)+1,U(2,:),1:round(simiter)+1,U(3,:)), grid on
% title('Control singals','FontSize',18)
% xlabel('iteration','FontSize',15)
% ylabel('Control signals','FontSize',15)
% legend('u_x','u_y','u_z')