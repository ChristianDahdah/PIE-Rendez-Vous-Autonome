% Loading Pirat generated matrices
load("fullmatrices")

%% Initial conditions (_i subscript)
alphaDCDT_i = 0.2; betaDCDT_i = 0.1; gammaDCDT_i = 0; % DC -> DT initial Euler angles
eulerDCDT_i = [alphaDCDT_i; betaDCDT_i; gammaDCDT_i];

sxDT_i = -10; syDT_i = 0; szDT_i = 0; % Initial chaser position wrt target (in target docking frame)
sDT_i = [sxDT_i; syDT_i; szDT_i];

omegaDCDT_i = [0;0;0]; % Immobile chaser at t = 0
dsDT_i = [0;0;0]; % Immobile target at t = 0

%% Reference variables (_ref subscript)
eulerDCDT_ref = [0;0;0]; % Relative euler angles between docking ports
omegaDCDT_ref = [0;0;0]; % No relative motion
sDT_ref = [0;0;0]; %  
dsDT_ref = [0;0;0];

%% Working point selection 
mu= 398600.4418e9; % en m^3/s^2
rT= 400e3 + 6371e3; % en m 
w0= sqrt(mu/rT^3);

mC = 12 ; % chaser mass in kg

% State initial values (ce have to give them the same names as in Pirat's
% symbolic implementation)
sxDT = sxDT_i; syDT = syDT_i; szDT =  szDT_i; % Initial chaser position wrt target (in target docking frame)
alphaDCDT = alphaDCDT_i; betaDCDT = betaDCDT_i; gammaDCDT = gammaDCDT_i; % DC -> DT initial Euler angles

rxDCDC = .1; ryDCDC = .1; rzDCDC = .1;
rxDTDT = .1; ryDTDT = .1; rzDTDT = .1;
aDT0 = 50*pi/180; bDT0 = 50*pi/180; cDT0 = 50*pi/180; % Docking port orientation

ICDC11 = 2; ICDC22 = 4; ICDC33 = 6; % en kg.m^2 (refine these values)
ITDT11 = 2; ITDT22 = 4; ITDT33 = 6;

A  = eval(Ar);
B = eval(Br);

save("model", "A", "B")
