function [dX] = P2P_DynCin_noquaternions(X, U)
    % Nonlinear, 6-dof model of the chaser-target relative coupled dynamics
    % (assuming a fixed target in the orbital local frame). dc is the
    % chaser docking port frame and dt the target docking port frame.
    % X is a 12 x 1 vector whose components are as follows (BE CAREFUL OF THE FRAMES):
    %   1:3 =  Attitude euler angles 
    %   4:6 = rotational speed of DC with respect to DT, in DC frame
    %   7:9 = relative position vector of DC with respect to DT, in DT
    %   frame
    %   10:12 = relative velocity of DC with respect to DT , in DT frame
    %
    %  dX is the time derivative of vector X
    %  U is the command vector whose components are as follows : 
    %       1:3 Control input torque 
    %       4:6 Control input force
    %   
    

    TDC =  U(1:3);
    FDC =  U(4:6);


    eulerDCDT = X(1:3);
    wDCDT= X(4:6);
    sDCDT = X(7:9);
    dsDCDT = X(10:12); 



% Define variables for symbolic toolbox
% P2P relative attitude angle variables
%syms alphaDCDT betaDCDT gammaDCDT dalphaDCDT dbetaDCDT dgammaDCDT real
% P2P relative angular velocity variables
%syms wxDCDT wyDCDT wzDCDT dwxDCDT dwyDCDT dwzDCDT real
% Target Docking port attitude Variables
% syms alphaDTo betaDTo gammaDTo dalphaDTo dbetaDTo dgammaDTo real
% % Target relative angular velocity variables
% syms wxDTo wyDTo wzDTo dwxDTo dwyDTo dwzDTo real
% % Chaser Inertia parametres expressed in docking portm frame
% syms ICDC11 ICDC12 ICDC13 ICDC21 ICDC22 ICDC23 ICDC31 ICDC32 ICDC33 mC real
% % Target Inertia parameters expressed in docking port frame
% syms ITDT11 ITDT12 ITDT13 ITDT21 ITDT22 ITDT23 ITDT31 ITDT32 ITDT33 real
% % Docking ports positions expressed in docking port frame
% syms rxDTDT ryDTDT rzDTDT rxDCDC ryDCDC rzDCDC real
% % relative position variables
% syms sxDT syDT szDT rT dsxDT dsyDT dszDT mu real
% % Control Input
% syms TxDT TyDT TzDT TxDC TyDC TzDC FxDC FyDC FzDC real
% % Other parameters
% syms mu w0 real
% % Linerisation point
% syms aDT0 bDT0 cDT0 real


 load parameters.mat

%Kinematics P2P
 
alphaDCDT = eulerDCDT(1);
betaDCDT = eulerDCDT(2);
gammaDCDT = eulerDCDT(3);

cg=cos(gammaDCDT) ;
sg= sin(gammaDCDT) ;
cb=cos(betaDCDT) ;
sb= sin(betaDCDT) ;

B_angle=1/cb *[cg -sg 0 ;
cb*sg cb*cg 0 ;
-sb*cg sb*sg cb];

deulerDCDT=B_angle*[wDCDT(1);wDCDT(2);wDCDT(3)];

%Kinematics Target Orbital
%AngleDTo=[alphaDTo ; betaDTo ;gammaDTo ] ;
% eulerDT0 = [aDT0 ; bDT0 ; cDT0 ];
% 
% cg= cos(gammaDTo);
% sg= sin(gammaDTo);
% cb= cos(betaDTo);
% sb= sin(betaDTo);
% 
% B_angle=1/cb*[cg -sg 0;
% cb*sg cb*cg 0;
% -sb*cg sb*sg cb];

% dAngleDTo=B_angle*[wxDTo;wyDTo;wzDTo]; UNUSED

%Relative Dynamics


ADTo=R3(eulerDTo_i(3))*R2(eulerDTo_i(2))*R1(eulerDTo_i(1));
ADCDT=R3(gammaDCDT)*R2(betaDCDT)*R1(alphaDCDT) ;
wDTo=[0;0;0];   % Fixed target docking port w.r.t. LVLH frame
%wDCDT=[wxDCDT;wyDCDT;wzDCDT]; 
wo=[0 -w0 0]';
wIT=wDTo+ADTo*wo;

%Dynamics for attitude DTO
%dwDTo=ITDT\(TDT-skew(wIT)*(ITDT*wIT));
dwDTo = [0;0;0];

%Dynamics for attitude DCDT
dwDCDT=ICDC\(TDC- skew(wDCDT+ADCDT*wIT)*(ICDC*(wDCDT+ADCDT*wIT)))...
-(skew(-wDCDT)*(ADCDT*wDTo)+ADCDT*dwDTo);


%P2P Translation dynamics
rTo=[0; 0; -rT];
% sDCDT=[sxDT; syDT; szDT];
% dsDCDT=[dsxDT; dsyDT; dszDT];

rDCDT=ADCDT'*rDCDC;
rxCDT=rDCDT(1);
ryCDT=rDCDT(2);
rzCDT=rDCDT(3);

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
    +skew(ADCDT'*dwDCDT)*(rDCDT)...
    +2*skew(ADCDT'*wDCDT)*skew(ADCDT'*wDCDT)*rDCDT;





    dX  = [deulerDCDT; dwDCDT; dsDCDT; ddsDCDT];

end


