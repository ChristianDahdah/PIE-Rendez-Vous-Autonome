
clear all, close all, clc
sDT_i = [-8; 0.3; -0.2];
T = 2;
N = 70;
radius = 1;
[X,dX,Theta,dTheta,u_cl] = final_approach_cylinder(sDT_i,T,N,radius);

plot()
