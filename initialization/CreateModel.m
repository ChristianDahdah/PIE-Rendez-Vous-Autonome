

% Loading Pirat generated matrices
load("FullMatrices");

%% Chaser dimensions
% Mass and Dimensions 

mC = 100; % chaser mass in kg

x = 2; y = 1; z = 1; % Meters
% for a homogeneous parallelepiped satellite :

ICDC = (mC/12)* [ y^2+z^2   0   0;
                 0  x^2+z^2  0
                 0   0  x^2+y^2 ]; % en kg.m^2

ICDC11 = ICDC(1,1); ICDC12 = ICDC(1,2); ICDC13 = ICDC(1,3);
ICDC21 = ICDC(2,1); ICDC22 = ICDC(2,2); ICDC23 = ICDC(2,3);
ICDC31 = ICDC(3,1); ICDC32 = ICDC(3,2); ICDC33 = ICDC(3,3);

%% Target dimensions
% Mass and Dimensions for a homogeneous parallelepiped satellite

mT = 7000; % Kg

x = 4; y = 1.5; z = 1; % Meters - doesn't matter if the target is fixed

ITDT = (mT/12)* [ y^2+z^2   0   0;
                 0  x^2+z^2  0
                 0   0  x^2+y^2 ]; % en kg.m^2

ITDT11 = ITDT(1,1); ITDT12 = ITDT(1,2); ITDT13 = ITDT(1,3);
ITDT21 = ITDT(2,1); ITDT22 = ITDT(2,2); ITDT23 = ITDT(2,3);
ITDT31 = ITDT(3,1); ITDT32 = ITDT(3,2); ITDT33 = ITDT(3,3);

%% Initial conditions (_i subscript)
alphaDCDT_i = 0.2; betaDCDT_i = 0.1; gammaDCDT_i = 0; % DC -> DT initial Euler angles
eulerDCDT_i = [alphaDCDT_i; betaDCDT_i; gammaDCDT_i];

sxDT_i = -10; syDT_i = 0; szDT_i = 0; % Initial chaser position wrt target (in target docking frame)
sDT_i = [sxDT_i; syDT_i; szDT_i];

omegaDCDT_i = [0;0;0]; % Immobile chaser at t = 0
dsDT_i = [0;0;0]; % Immobile target at t = 0

%% Reference variables (_ref subscript)
% eulerDCDT_ref = [0;0;0]; % Relative euler angles between docking ports
% omegaDCDT_ref = [0;0;0]; % No relative motion
% sDT_ref = [0;0;0]; %  
% dsDT_ref = [0;0;0];

%% Working point selection 
mu= 398600.4418e9; % en m^3/s^2
rT= 400e3 + 6378e3; % en m 
w0= sqrt(mu/rT^3);

% State initial values (we have to give them the same names as in Pirat's
% symbolic implementation)

% Initial chaser position wrt target (in target docking frame)
sxDT = sxDT_i; syDT = syDT_i; szDT =  szDT_i;

% DC -> DT initial Euler angles
alphaDCDT = alphaDCDT_i; betaDCDT = betaDCDT_i; gammaDCDT = gammaDCDT_i;

% Position of the docking port of the chaser in the chaser's frame
rxDCDC = .1; ryDCDC = .1; rzDCDC = .1;

% Position of the docking port of the target in the target's frame
rxDTDT = -.1; ryDTDT = .1; rzDTDT = .1;

% Target docking port orientation
aDT0 = 50*pi/180; bDT0 = 50*pi/180; cDT0 = 50*pi/180;

A  = eval(Ar);
B = eval(Br);

save('../initialization/linear_model.mat', 'A', 'B')
save('../initialization/parameters.mat', 'mu','rT','w0','ICDC', 'mC', 'ITDT',...
    'eulerDCDT_i', 'sDT_i', 'omegaDCDT_i', ...
    'dsDT_i')