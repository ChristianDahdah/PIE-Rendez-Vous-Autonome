%% Tests performance

%% initialisation

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

hold_points = [
    0,-1500,0,0,0,0;
    250,-1000,0,0,0,0;
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
%% test 1: convergence Kc

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
mesure_error = 0;
acc_pert = 0;
time_integ = 20000; % time of start of integrated law

res2 = [];
Kc_aug = [0 0 0 0 0 0 0 0 0];
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;

Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
for k=1:20
    k
    R = eye(3)*10^k;
    
    [traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
    SimOut = sim('../closing/boucle_LQG_finale');
    etat_final = SimOut.yout{1}.Values.Data(end,:);
    res2 = [res2 etat_final'];
    
    figure(1)
    hold on
    plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1))
end
figure(1)
legend
ylim([-100,300])
xlabel time
ylabel x
% on garde 10^10 qui est le moins pire


%% test 3: resistance perturbation mesure

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
mesure_error = 0;
acc_pert = 0;
time_integ = 20000; % time of start of integrated law

res3 = [];
Kc_aug = [0 0 0 0 0 0 0 0 0];

Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
R = eye(3)*1e10;
V = eye(3);
Ninterval = 50;


for k=-8:-1
    k
    W = 10^k*(B*B');
    [traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
    for i=1:6
        i
        mesure_error = 0.05*i;
        
        for j=3:3
            j
            acc_pert = 10^(-j);
            
            SimOut = sim('../closing/boucle_LQG_finale');
            etat_final = SimOut.yout{1}.Values.Data(end,:);
            res3 = [res3 [k;i;j;etat_final']];
            
            figure(2)
            hold on
            plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1))
        end
    end
end
xlabel time
ylabel x
% bilan :
% pour acc_pert = 1e-4
% on a moins de 20% de décalage avec la consigne sur les deux premiers
% jumps( ie < 300)
% k = -1 ==> jusqu'à 5% erreur de mesure
% k = -2  ==> jusqu'à 10% erreur de mesure
% k = -3 ou -4 ==> jusqu'à 15% erreur de mesure
% k = -5 ==> jusqu'à 25% erreur de mesure
% k = -6 ou moins ==> jusqu'à 30% erreur de mesure

% pour acc_pert = 1e-3
% on a moins de 20% de décalage avec la consigne sur les deux premiers
% jumps( ie < 300)
% k = -1 ==> jusqu'à 5% erreur de mesure
% k = -2  ==> jusqu'à 10% erreur de mesure
% k = -3 ou -4 ==> jusqu'à 15% erreur de mesure
% k = -5 ==> jusqu'à 25% erreur de mesure
% k = -6 ou moins ==> jusqu'à 30% erreur de mesure

%% test 4: perfo

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
mesure_error = 0.3;
acc_pert = 1e-4;
time_integ = 20000; % time of start of integrated law

res2 = [];
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;

Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
R = eye(3)*1e10;
res4 = [];
[traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
time_integ = 16000; % time of start of integrated law

for k=0:6
    Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) 10^(-k)*eye(3)]],1e10*eye(3));
    
    k
    SimOut = sim('../closing/boucle_LQG_finale');
    etat_final = SimOut.yout{1}.Values.Data(end,:);
    res4 = [res4 [k;i;j;etat_final']];
    figure(3)
    hold on
    plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1))
end
legend(["1","2","3","4","5","6","7"])
xlabel time
ylabel x

% pour 1e-6 ou 1e-5 sur l'intégrale, ok jusqu'à démarage à 4000s (ie au cours du
% deuxième jump)

% en prenant un démarage à la fin des manoeuvres (à 16000), on ne va pas
% sous -1m pour k = 4 ie un rappot de 1e-4 dans Q.

%% récupération de graphiques de perfomance en perturbation

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
mesure_error = 0;
res2 = [];
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
R = eye(3)*1e10;
res4 = [];
[traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
time_integ = (length(hold_points) + 1)*Torb/4; % time of start of integrated law
Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) 10^(-4)*eye(3)]],1e10*eye(3));

for k=0:10
    k
    acc_pert = k*5e-4;
    SimOut = sim('../closing/boucle_LQG_finale');
    figure(4)
    hold on
    plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1:3))
end
xlabel time
ylabel x
%% récupération de graphiques de perfomance en mesure

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
acc_pert = 0;
res2 = [];
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
R = eye(3)*1e10;
res4 = [];
[traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
time_integ = (length(hold_points) + 1)*Torb/4; % time of start of integrated law
Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) 10^(-4)*eye(3)]],1e10*eye(3));

for k=0:10
    k
    mesure_error = k*0.1;
    SimOut = sim('../closing/boucle_LQG_finale');
    figure(5)
    hold on
    plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1:3))
end
xlabel time
ylabel x
%% récupération de graphiques de perfomance combinées

mode = 1; % two burns
tau = 10;% temps de réponse caractéristique de l'actionneur
umax = 0.001; % seuil de saturation de la commande
acc_pert = 0;
res2 = [];
W = 1e-6*(B*B');
V = eye(3);
Ninterval = 50;
Q = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
R = eye(3)*1e10;
res4 = [];
[traj_analytique,traj_PMP,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points);
time_integ = (length(hold_points) + 1)*Torb/4; % time of start of integrated law
Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) 10^(-4)*eye(3)]],1e10*eye(3));

legends = [];
for k=0:10
    k
    mesure_error = k*0.1;
    for j=0:10
        j
        acc_pert = k*5e-4;
        SimOut = sim('../closing/boucle_LQG_finale');
        figure(6)
        hold on
        plot(SimOut.tout,SimOut.yout{1}.Values.Data(:,1:3))
        legends=[legends k + "," + j];
    end
end
legend(legends)
xlabel time
ylabel("x,y,z")
