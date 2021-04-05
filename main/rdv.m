close all;
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
gain_integ = 10^(-4);
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;

% les hold_points sont atteints tous les quarts d'orbite. Il faut ainsi en
% utiliser deux pour définir un saut : un au sommet et un à l'arrivée.

hold_points = [
    0,-1500,0,0,0,0; 
    250,-1000,0,0,0,0; % point intermédiaire : z = detax/4., x = x0 + deltax/2
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
duree_totale_mission = (length(hold_points)-1)*Torb/4/60 % en min, doit etre inferieur a 480 min pour être acceptable par les standards (vol habité etc)
%%
[ts_full_analytique,ts_full,Kf,Kc,Kc_aug,manoeuvres] = closing(altitude, A, B, C, Q, R, gain_integ, W, V, Ninterval, hold_points);

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
SimOut = sim('../closing/boucle_LQG_finale');
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

