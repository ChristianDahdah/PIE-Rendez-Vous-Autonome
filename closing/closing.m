function [traj_analytique,traj_PMP,Kf,Kc,Kc_aug,manoeuvres] = closing(altitude, A, B, C, Q, R, gain_integ, W, V, Ninterval, hold_points)
% Calcule la trajectoire de closing passant par les hold_points ainsi que
% les différents gains du correcteur et de l'estimateur.
% chaque jump est effectué en un quart d'orbite.
% 
% traj_analytique = trajectoire en deux poussées selon la solution
% analytique de Clohessy Wiltshire
% traj_PMP = trajectoire optimale selon le principe du maximum de
% Pontryangin
% Kf = gain de l'estimateur de Kalman
% Kc = gain du correcteur LQ
% Kc_aug = gain du correcteur LQ avec intégrateur.
% manoeuvre = manoeuvres impulsionnelles théoriques, pour tester la faisabilité
% en éléctrique de la trajectoire en deux poussée analytique.
%
% altitute en km
% Q, R pour Kc
% gain_integ : gain de l'intégration de l'erreur dans le matrice Q du clcul
% de Kc_aug
% W, V pour Kf
% Ninterval : nombre d'intervalles pour la discrétisation de la trajectoire
% hold_points liste des points de passage, en m (z,x,y,vz,vx,vy)

%% initialisation
Rt = 6371; %km
mu = 3.986004418*10^5; %km^3s^-2
Torb = 2*pi*sqrt((altitude +Rt)^3/mu);
% Subsitution w par sa valeur
w = 2*pi/Torb ;


%% Calcul Kc

Kc=lqr(A,B,Q,R);


%% Calcul Kf
Kf=lqr(A',C',W,V);
Kf=Kf';

%% Calcul de Kc_aug

Kc_aug = lqr([[A;[eye(3) zeros(3,3)]] zeros(9,3)],[B;zeros(3,3)], [[Q zeros(6,3)];[zeros(3,6) gain_integ*eye(3)]],R);
%% Trajectoire Optimale
syms t T real
Ea_t = eye(6)+A*t+(A*t)^2/2+(A*t)^3/6+(A*t)^4/24+(A*t)^5/96+(A*t)^6/factorial(6)+(A*t)^7/factorial(7);%+(A*t)^8/factorial(8)+(A*t)^9/factorial(9)+(A*t)^10/factorial(10)+(A*t)^11/factorial(11)+(A*t)^12/factorial(12)+(A*t)^13/factorial(13)+(A*t)^14/factorial(14)+(A*t)^15/factorial(15)+(A*t)^16/factorial(16)+(A*t)^17/factorial(17)+(A*t)^18/factorial(18)+(A*t)^19/factorial(19)+(A*t)^20/factorial(20);%expm(A*t);
Ema_t = eye(6)-A*t+(A*t)^2/2-(A*t)^3/6+(A*t)^4/24-(A*t)^5/96+(A*t)^6/factorial(6)-(A*t)^7/factorial(7);%+(A*t)^8/factorial(8)-(A*t)^9/factorial(9)+(A*t)^10/factorial(10)-(A*t)^11/factorial(11)+(A*t)^12/factorial(12)-(A*t)^13/factorial(13)+(A*t)^14/factorial(14)-(A*t)^15/factorial(15)+(A*t)^16/factorial(16)-(A*t)^17/factorial(17)+(A*t)^18/factorial(18)-(A*t)^19/factorial(19)+(A*t)^20/factorial(20);% expm(-A*t);
D_t = Ema_t*B;
C_T = int(D_t*D_t',t,0,T);
C_t = subs(C_T,T,t);

%% Calcul P_0, X_t et U_t (boucle ouverte)(premiere condition initiale)

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
    C_t_list = [C_t_list subs(C_t,t,time_list(k))]; % pour ne pas avoir à effectuer le même calcul plusieurs fois
    Ea_t_list = [Ea_t_list subs(Ea_t,t,time_list(k))]; % pour ne pas avoir à effectuer le même calcul plusieurs fois
end

%% méthode optimale

traj_PMP = timeseries();

for i =1:length(hold_points(:,1))-1
    X_0 = hold_points(i,:)';
    X_1 = hold_points(i+1,:)';
    
    P_0 = -C_T\(X_0-Ema_T*X_1);
    %X_t = Ea_t*(X_0 + C_t*P_0);
    
    trajX = zeros(6,NPT);
    
    for k = 1:NPT
        point = double(Ea_t_list(:,(6*k-(6-1)):(6*k))*(X_0 + C_t_list(:,(6*k-(6-1)):(6*k))*P_0));
        trajX(:,k) = point;
    end
    
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    traj_PMP = append(traj_PMP,ts);
end
figure()
plot(traj_PMP.Data(:,1),traj_PMP.Data(:,2))

%% methode analytique

manoeuvres = [];

traj_analytique = timeseries();

for i =1:length(hold_points(:,1))-1
    
    z0 = hold_points(i,1);
    x0 = hold_points(i,2);
    zf = hold_points(i+1,1);
    xf = hold_points(i+1,2);
    
    trajX = zeros(6,NPT);
    
    vx0 = 2*z0*w;
    vz0 = ((xf-x0)-2*z0)*w/2;
    
    % les formules générales sans supposer que vx0 = 2*z0*w donnent:
    % vx0 = w*(xf+6*(1-pi/2)*z0-x0+2*(-zf+4*z0))/(8-3*pi/2);
    % vz0 = -w*(-zf+4*z0)+2*vx0;
    manoeuvres = [manoeuvres [vz0;vx0]];
    
    for k = 1:NPT
        [x,z,vx,vz] = analytical_xz(x0,z0,vx0,vz0,0,0,w,dt*(k-1));
        trajX(:,k) = [z,x,0,vz,vx,0];
    end
    ts = timeseries(trajX',[0:Ninterval]*dt + ones(1,NPT) * (NPT+1) * dt * (i-1));
    traj_analytique = append(traj_analytique,ts);
end
ts = timeseries(hold_points(end,:),Ninterval*dt +(NPT+1) * dt * (length(hold_points(:,1))-2 + dt));
traj_analytique = append(traj_analytique,ts);

figure()
plot(traj_analytique.Data(:,1),traj_analytique.Data(:,2))


end

