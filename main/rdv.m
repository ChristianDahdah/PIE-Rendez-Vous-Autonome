altitude = 400;
Q = [0.01 0 0 0;0 0.01 0 0;0 0 0 0;0 0 0 0.0];
R = eye(1)*100000;
B = [0 0;0 0;1 0;0 1];
W = 0.01*(B*B');
V = eye(2);
Ninterval = 10;
hold_points = [
    0,-500,0,0;
    0,-200,0,0;
    0,-100,0,0;
    100,0,0,0;
    0,100,0,0;
    0,50,0,0;
    0,10,0,0];

[ts_full_analytique,ts_full,Kf,Kc] = closing(altitude, Q, R, W, V, Ninterval, hold_points);

mod = 1; % two burns
% mod = -1; % PMP

[u, etat, etat_est] = sim('./closing/obj_atteint');

