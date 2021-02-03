close all;
% clear all;
%%
altitude = 400;
Rt = 6371; %km
mu = 3.986004418*10^5; %km^3s^-2
Torb = 2*pi*sqrt((altitude +Rt)^3/mu);
% Subsitution w par sa valeur
w = 2*pi/Torb ;

A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     3*w*w 0 0 0 -2*w 0;
     0 0 0 2*w 0 0;
     0 0 -w*w 0 0 0];
B = [0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1]; % on utilise les 2 commandes
C = [1 0 0 0 0 0;
     0 1 0 0 0 0
     0 0 1 0 0 0];
D=zeros(3);
%%
Q = [0.01 0 0 0 0 0;
     0 0.01 0 0 0 0;
     0 0 0.01 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
R = eye(3)*100000;
W = 0.01*(B*B');
V = eye(3);
Ninterval = 10;
hold_points = [
    0,-500,0,0,0,0;
    0,-200,0,0,0,0;
    0,-100,0,0,0,0;
    100,0,0,0,0,0;
    0,100,0,0,0,0;
    0,50,0,0,0,0;
    0,10,0,0,0,0];
%%
[ts_full_analytique,ts_full,Kf,Kc] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);

%%
mod = 1; % two burns
% mod = -1; % PMP

SimOut = sim('../closing/obj_atteint');
u = SimOut.get('yout').get('commande');
etat = SimOut.get('yout').get('etat');
etat_est = SimOut.get('yout').get('etat_est');

etat_final = SimOut.yout{1}.Values.Data(end,:)