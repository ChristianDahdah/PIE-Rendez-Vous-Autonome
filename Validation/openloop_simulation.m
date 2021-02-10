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


dt = 0.1; % time step in seconds
Tmax = 20; % max. simulation time in seconds

t = 0:dt:Tmax;

nStates = 13;
nControls = 6;


XX_ss = zeros(nStates,length(t)); %% States history of the LINEAR SIMULATION
XX_nl = zeros(nStates,length(t)); %% States history of the NON-LINEAR SIMULATION

UU = zeros(nControls, length(t));

XX(:,1) = [QDCDT_i ; omegaDCDT_i ; sDT_i ; dsDT_i ]






for i = 1:length(t)
    
    
    
    
    
    
end







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