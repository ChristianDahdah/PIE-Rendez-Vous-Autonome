% Syntaxe:
% patch([X1,X2,X3,X4], [Y1,Y2,Y3,Y4], [Z1, Z2, Z3, Z4])
% Where 1 2 3 4 are the four points of the rectangle
% patch([0,3,3,0], [0,0,2,2], [0,0,0,0], 'blue')
% patch([0,3,3,0], [0,0,2,2], [2,2,2,2], 'red')
% patch([0,0,0,0], [0,0,2,2], [0,2,2,0], 'green')
% patch([0,3,3,0], [0,0,0,0], [0,0,2,2], 'yellow')
% patch([0,0,3,0], [2,2,2,2], [2,2,0,0], 'pink')


x = 2
y = 3
z = 3

xt = 1
yt = 1
zt = 1

dimension = [x, y, z]

translation = [xt, yt, zt]

centre = dimension/2 + translation


% Bedak tshouf shi khasso bel alphaDCDT wel alphaDT0
% Awle bten2elne men repere chaser lal target, wel tenye bten2elne men
% repère target lal orbital
% Shuf p.50 Pirat
