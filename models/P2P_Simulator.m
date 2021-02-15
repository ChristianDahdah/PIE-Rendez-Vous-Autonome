clear all; close all;
format long;

addpath '../subfunctions'

dt = 0.1; % Time step in s
T = 50; % Max. simulation time in s


%Kinematics P2P (euler angles, directly converted into quaternions)

load linear_model.mat A B % State-space representation of the coupled 6 dof system
load parameters.mat mu rT w0 ICDC mC ITDT eulerDTo_i eulerDCDT_i omegaDCDT_i sDT_i dsDT_i % Other parameters (inertia, constants) 

% Initial angles (used for declaration ONLY)
%alphaDTo_i= 50*pi/180; betaDTo_i = 50*pi/180; gammaDTo_i= 50*pi/180;
%alphaDCDT_i = 60*pi/180; betaDCDT_i = 40*pi/180; gammaDCDT_i=20*pi/180;

QDTo_i = euler2quat(eulerDTo_i);
QDCDT_i = euler2quat(eulerDCDT_i);
















