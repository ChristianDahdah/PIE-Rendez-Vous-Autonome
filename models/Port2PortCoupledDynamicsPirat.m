% Define variables for symbolic toolbox
% P2P relative attitude angle variables
syms alphaDCDT betaDCDT gammaDCDT dalphaDCDT dbetaDCDT dgammaDCDT real
% P2P relative angular velocity variables
syms wxDCDT wyDCDT wzDCDT dwxDCDT dwyDCDT dwzDCDT real
% Target Docking port attitude Variables
syms alphaDTo betaDTo gammaDTo dalphaDTo dbetaDTo dgammaDTo real
% Target relative angular velocity variables
syms wxDTo wyDTo wzDTo dwxDTo dwyDTo dwzDTo real
% Chaser Inertia parametres expressed in docking portm frame
syms ICDC11 ICDC12 ICDC13 ICDC21 ICDC22 ICDC23 ICDC31 ICDC32 ICDC33 mC real
% Target Inertia parameters expressed in docking port frame
syms ITDT11 ITDT12 ITDT13 ITDT21 ITDT22 ITDT23 ITDT31 ITDT32 ITDT33 real
% Docking ports positions expressed in docking port frame
syms rxDTDT ryDTDT rzDTDT rxDCDC ryDCDC rzDCDC real
% relative position variables
syms sxDT syDT szDT rT dsxDT dsyDT dszDT mu real
% Control Input
syms TxDT TyDT TzDT TxDC TyDC TzDC FxDC FyDC FzDC real
% Other parameters
syms mu w0 real
% Linerisation point
syms aDT0 bDT0 cDT0 real

%Kinematics P2P
AngleDC=[alphaDCDT; betaDCDT;gammaDCDT] ;

cg=cos(gammaDCDT) ;
sg= sin(gammaDCDT) ;
cb=cos(betaDCDT) ;
sb= sin(betaDCDT) ;

B_angle=1/cb *[cg -sg 0 ;
cb*sg cb*cg 0 ;
-sb*cg sb*sg cb];

dAngleDC=B_angle*[wxDCDT;wyDCDT;wzDCDT];

%Kinematics Target Orbital
AngleDTo=[alphaDTo ; betaDTo ;gammaDTo ] ;

cg= cos(gammaDTo);
sg= sin(gammaDTo);
cb= cos(betaDTo);
sb= sin(betaDTo);

B_angle=1/cb*[cg -sg 0;
cb*sg cb*cg 0;
-sb*cg sb*sg cb];

dAngleDTo=B_angle*[wxDTo;wyDTo;wzDTo];

%Relative Dynamics
ICDC=[ ICDC11 ICDC12 ICDC13;
    ICDC21 ICDC22 ICDC23;
    ICDC31 ICDC32 ICDC33];
ITDT=[ ITDT11 ITDT12 ITDT13;
    ITDT21 ITDT22 ITDT23;
    ITDT31 ITDT32 ITDT33];

TDC=[TxDC TyDC TzDC]';
TDT=[TxDT TyDT TzDT]';
ADTo=R3(gammaDTo)*R2(betaDTo)*R1(alphaDTo);
ADCDT=R3(gammaDCDT)*R2(betaDCDT)*R1(alphaDCDT) ;
wDTo=[wxDTo;wyDTo;wzDTo];
wDCDT=[wxDCDT;wyDCDT;wzDCDT];
wo=[0 -w0 0]';
wIT=wDTo+ADTo*wo;

%Dynamics for attitude DTO
dwDTo = [0;0;0];
% dwDTo=ITDT\(TDT-skew(wIT)*(ITDT*wIT));
dwxDTo=dwDTo(1);
dwyDTo=dwDTo(2);
dwzDTo=dwDTo(3);

%Dynamics for attitude DCDT
dwDC=ICDC\(TDC- skew(wDCDT+ADCDT*wIT)*(ICDC*(wDCDT+ADCDT*wIT)))...
-(skew(-wDCDT)*(ADCDT*wDTo)+ADCDT*dwDTo);
dwxDCDT=dwDC(1);
dwyDCDT=dwDC(2);
dwzDCDT=dwDC(3);

%P2P Translation dynamics
rTo=[0; 0; -rT];
sDCDT=[sxDT; syDT; szDT];
dsDCDT=[dsxDT; dsyDT; dszDT];
rDCDC=[rxDCDC; ryDCDC; rzDCDC];
rDTDT=[rxDTDT; ryDTDT; rzDTDT];
rDCDT=ADCDT'*rDCDC;
rxCDT=rDCDT(1);
ryCDT=rDCDT(2);
rzCDT=rDCDT(3);
FDC=[FxDC FyDC FzDC]';
rcDT=ADTo*rTo+sDCDT-rDCDT+rDTDT;

accDT=mu*ADTo*rTo/norm(ADTo*rTo)^3-mu*(rcDT)/norm(rcDT)^3+ADCDT'*FDC/mC;
s=sDCDT-rDCDT+rDTDT;
ddsDCDT=-skew(dwDTo)*s...
    - skew(wDTo)*skew(wDTo)*s...
    - skew(ADTo*wo)*skew(ADTo*wo)*s...
    -2*skew(wDTo)*dsDCDT...
    -2*skew(ADTo*wo)*dsDCDT...
    -2*skew(ADTo*wo)*skew(wDTo)*s...
    +2*skew(ADTo*wo+wDTo)*skew(ADCDT'*wDCDT)*(rDCDT)...
    +accDT...
    +skew(ADCDT'*dwDC)*(rDCDT)...
    +2*skew(ADCDT'*wDCDT)*skew(ADCDT'*wDCDT)*rDCDT;

%Compute jacobian
ftot =[dAngleDC ;dwDC; dAngleDTo ;dwDTo;dsDCDT;ddsDCDT] ;
Atot= jacobian(ftot, [AngleDC' wDCDT' AngleDTo' wDTo' sDCDT' dsDCDT']);
Btot= jacobian(ftot, [TDC' TDT' FDC']);

%Linearisation
alphaDCDT=0;
betaDCDT=0;
gammaDCDT=0;
wxDCDT=0;
wyDCDT=0;
wzDCDT=0;
alphaDTo=aDT0;
betaDTo=bDT0;
gammaDTo=cDT0;
wxDTo=0;
wyDTo=0;
wzDTo=0;
TxDC=0;
TyDC=0;
TzDC=0;
TxDT=0;
TyDT=0;
TzDT=0;
sxDT=0;
syDT=0;
szDT=0;
dsxDT=0;
dsyDT=0;
dszDT=0;
FxDC=0;
FyDC=0;
FzDC=0;

Atot=eval(Atot);
Btot=eval(Btot);
Atot=simplify(Atot);
Btot=(simplify(Btot));