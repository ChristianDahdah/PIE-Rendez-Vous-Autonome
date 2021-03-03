close all;
clear all;
path(path,'../closing')
%%
altitude = 400;
Rt = 6371; %km
mu = 3.986004418*10^5; %km^3s^-2
Torb = 2*pi*sqrt((altitude +Rt)^3/mu);
% Subsitution w par sa valeur
w = 2*pi/Torb ;
% X = [z,x,y,vz,vx,vy]

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
Q = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0];
R = eye(3)*1e10;
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;
% hold_points = [
%     0,-3500,0,0,0,0;
%     0,-3500,0,0,0,0;
%     0,-250,0,0,0,0;
%     0,-250,0,0,0,0];
hold_points = [
    0,-1500,0,0,0,0; % le départ à -1500 ne passe pas
    250,-1000,0,0,0,0; % point intermédiaire : z = detax/4. Probleme car vie supossée nulle aux hold points
    0,-500,0,0,0,0;
    250,0,0,0,0,0;
    0,500,0,0,0,0;
    -62.5,375,0,0,0,0;
    0,250,0,0,0,0;
    -50,150,0,0,0,0;
    0,50,0,0,0,0;
    -10,30,0,0,0,0;
    0,10,0,0,0,0
    ];
% hold_points = [
%     0,-500,0,0,0,0;
%     0,-200,0,0,0,0;
%     0,-100,0,0,0,0;
%     100,0,0,0,0,0;
%     0,100,0,0,0,0;
%     0,50,0,0,0,0;
%     0,10,0,0,0,0];

duree_totale_mission = (length(hold_points)-1)*Torb/4/60 % en min, doit etre inferieur a 480 min
%%
[ts_full_analytique,ts_full,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);

%% LQ intégrateur

Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) 1e-4*eye(3)]],1e10*eye(3));


%%
close all;
mod = 1; % two burns
% mod = -1; % PMP
tau = 100;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
mesure_error = .1;
acc_pert = 1e-4;
time_integ = (length(hold_points) + 1)*Torb/4; % time of start of integrated law
%%
% SimOut = sim('../closing/obj_atteint_precis_2020a');
SimOut = sim('../closing/obj_atteint_precis_2020a');
u = SimOut.get('yout').get('commande');
etat = SimOut.get('yout').get('etat');
etat_est = SimOut.get('yout').get('etat_est');

etat_final = SimOut.yout{1}.Values.Data(end,:)

figure()
plot(SimOut.tout,SimOut.yout{1}.Values.Data)
hold on
plot(SimOut.tout,SimOut.yout{5}.Values.Data)
title etat
ylabel m
xlabel s
figure()
plot(SimOut.tout,SimOut.yout{2}.Values.Data)
title commande
ylabel('m/s²')
xlabel s
figure()
plot(SimOut.tout,SimOut.yout{3}.Values.Data)
title estimation
ylabel m
xlabel s
figure()
plot(SimOut.tout,SimOut.yout{4}.Values.Data)
title('commande avant corrections')
ylabel('m/s²')
xlabel s

