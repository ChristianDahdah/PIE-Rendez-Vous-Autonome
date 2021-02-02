function [ts_full_analytique,ts_full,Kf,Kc,SimOut] = closing(altitude, Q, R, W, V, Ninterval, hold_points)
% chaque jump est effectué en un quart d'orbite
% ts_full_analytique = trajectoire en deux poussées
% ts_full = trajectoire avec PMP
% Kf = gain de l'estimateur de Kalman
% Kc = gain du correcteur LQ
%
% altitute en km
% Q, R pour Kc
% W, V pour Kf
% Ninterval : nombre d'intervalles pour la discrétisation de la trajectoire
% hold_points liste des points de passage, en m (x,y,vx,vy)

%% initialisation
% altitude = 400; %km
Rt = 6371; %km
mu = 3.986004418*10^5; %km^3s^-2
Torb = 2*pi*sqrt((altitude +Rt)^3/mu);
% Subsitution w par sa valeur
w = 2*pi/Torb ;

A=[0 0 1 0;
    0 0 0 1;
    3*w*w 0 0 -2*w;
    0 0 2*w 0];
B = [0 0;0 0;1 0;0 1]; % on utilise les 2 commandes
C=[1 0 0 0;0 1 0 0];
D=zeros(2);

%% Calcul Kc

% Q=[0.01 0 0 0;0 0.01 0 0;0 0 0 0;0 0 0 0.0];
% R=eye(1)*100000;
Kc=lqr(A,B,Q,R);


%% Calcul Kf
% Réglage du filtre:
% W=0.01*B*B'; % bruit d'état avec DSP=0.1
% V=eye(2);   % bruit de mesure avec DSP=1
Kf=lqr(A',C',W,V);
Kf=Kf';

%% Trajectoire Optimale
syms t T real
Ea_t = eye(4)+A*t+(A*t)^2/2+(A*t)^3/6;%expm(A*t);
Ema_t = eye(4)-A*t+(A*t)^2/2-(A*t)^3/6;% expm(-A*t);
D_t = Ema_t*B;
C_T = int(D_t*D_t',t,0,T);
C_t = subs(C_T,T,t);

%% Calcul P_0, X_t et U_t (boucle ouverte)(premiere condition initiale)

% Ninterval = 10;
NPT = Ninterval + 1;

% Subsitution T par sa valeur
T = 2*pi/w/4;
C_T = subs(C_T);
Ema_T = subs(Ema_t, t, T);

dt = T/Ninterval;
time_list = dt*[0:Ninterval];
C_t_list = [];
Ea_t_list = [];

for k = 1:NPT
    C_t_list = [C_t_list subs(C_t,t,time_list(k))];
    Ea_t_list = [Ea_t_list subs(Ea_t,t,time_list(k))];
end

%% final

% hold_points = [
%     0,-500,0,0;
%     0,-200,0,0;
%     0,-100,0,0;
%     100,0,0,0;
%     0,100,0,0;
%     0,50,0,0;
%     0,10,0,0];

ts_full = timeseries();

for i =1:length(hold_points(:,1))-1
    X_0 = hold_points(i,:)';
    X_1 = hold_points(i+1,:)';
    
    
    
    P_0 = -C_T\(X_0-Ema_T*X_1);              % Eq. (4)
    %X_t = Ea_t*(X_0 + C_t*P_0);  % Eq. (3)
    
    
    trajX = zeros(4,NPT);
    
    for k = 1:NPT
        trajX(:,k) = double(Ea_t_list(:,(4*k-(4-1)):(4*k))*(X_0 + C_t_list(:,(4*k-(4-1)):(4*k))*P_0));
    end
    
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    ts_full = append(ts_full,ts);
end
figure()
plot(ts_full.Data(:,1),ts_full.Data(:,2))

%% methode analytique

% il faut faire une modif car le repère est différetn (axe y inversé)



ts_full_analytique = timeseries();


for i =1:length(hold_points(:,1))-1
    
    x0 = hold_points(i,1);
    y0 = hold_points(i,2);
    xf = hold_points(i+1,1);
    yf = hold_points(i+1,2);
    
    trajX = zeros(4,NPT);
    
    vy0 = w*(yf-6*(1-pi/2)*-x0-y0+2*(-xf-4*-x0))/(8-3*pi/2);
    vx0 = w * (-xf-4*-x0)-2*vy0;
    
    for k = 1:NPT
        [x,y,vx,vy] = analytical_inverted(-x0,y0,vx0,vy0,0,0,w,dt*(k-1));
        trajX(:,k) = [-x,y,-vx,vy];
    end
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    ts_full_analytique = append(ts_full_analytique,ts);
end

figure()
plot(ts_full_analytique.Data(:,1),ts_full_analytique.Data(:,2))

%%
mod = 1; % two burns
% mod = -1; % PMP

SimOut = sim('../closing/obj_atteint')
u = SimOut.get('yout').get('commande');
etat = SimOut.get('yout').get('etat');
etat_est = SimOut.get('yout').get('etat_est');
end

