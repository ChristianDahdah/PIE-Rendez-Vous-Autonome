function [ts_full_analytique,ts_full,Kf,Kc,manoeuvres] = closing(altitude, A, B, C, Q, R, W, V, Ninterval, hold_points)
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
Ea_t = exp(A*t);%eye(6)+A*t+(A*t)^2/2+(A*t)^3/6+(A*t)^4/24+(A*t)^5/96+(A*t)^6/factorial(6)+(A*t)^7/factorial(7)+(A*t)^8/factorial(8)+(A*t)^9/factorial(9)+(A*t)^10/factorial(10);%expm(A*t);
Ema_t = exp(-A*t);%eye(6)-A*t+(A*t)^2/2-(A*t)^3/6+(A*t)^4/24-(A*t)^5/96+(A*t)^6/factorial(6)-(A*t)^7/factorial(7)+(A*t)^8/factorial(8)-(A*t)^9/factorial(9)+(A*t)^10/factorial(10);% expm(-A*t);
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
    
    
    trajX = zeros(6,NPT);
    
    for k = 1:NPT
        point = double(Ea_t_list(:,(6*k-(6-1)):(6*k))*(X_0 + C_t_list(:,(6*k-(6-1)):(6*k))*P_0));
        trajX(:,k) = point;
    end
    
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    ts_full = append(ts_full,ts);
end
figure()
plot(ts_full.Data(:,1),ts_full.Data(:,2))

%% methode analytique

% il faut faire une modif car le repère est différent (axe y inversé)

manoeuvres = [];

ts_full_analytique = timeseries();


for i =1:length(hold_points(:,1))-1
    
    z0 = hold_points(i,1);
    x0 = hold_points(i,2);
    zf = hold_points(i+1,1);
    xf = hold_points(i+1,2);
    
    trajX = zeros(6,NPT);
    
    vx0 = w*(xf+6*(1-pi/2)*z0-x0+2*(-zf+4*z0))/(8-3*pi/2);
    vz0 = -w * (-zf+4*z0)+2*vx0;
    manoeuvres = [manoeuvres [vz0;vx0]];
    
    for k = 1:NPT
        [x,z,vx,vz] = analytical_xz(x0,z0,vx0,vz0,0,0,w,dt*(k-1));
        trajX(:,k) = [z,x,0,vz,vx,0];
    end
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    ts_full_analytique = append(ts_full_analytique,ts);
end
ts = timeseries(hold_points(end,:),Ninterval*dt +(NPT+1) * dt * (length(hold_points(:,1))-2 + dt));
ts_full_analytique = append(ts_full_analytique,ts);
% figure()
% plot(ts_full_analytique.Data(:,1),ts_full_analytique.Data(:,2))


end

