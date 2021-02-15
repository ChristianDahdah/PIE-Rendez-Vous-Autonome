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
    %syms alphaDTo betaDTo gammaDTo dalphaDTo dbetaDTo dgammaDTo real
%     alphaDTo = 20*pi/180;
%     betaDTo =  20*pi/180;
%     gammaDTo =  20*pi/180;
    
   
    
    % Target relative angular velocity variables
    %syms wxDTo wyDTo wzDTo dwxDTo dwyDTo dwzDTo real
    % Chaser Inertia parametres expressed in docking portm frame
    %syms ICDC11 ICDC12 ICDC13 ICDC21 ICDC22 ICDC23 ICDC31 ICDC32 ICDC33 mC real
    
    load parameters.mat
    

   
%Kinematics P2P
%AngleDC=[alphaDCDT; betaDCDT;gammaDCDT] ;

alphaDCDT= eulerDCDT(1) ; betaDCDT= eulerDCDT(2);gammaDCDT= eulerDCDT(3);

cg=cos(gammaDCDT) ;
sg= sin(gammaDCDT) ;
cb=cos(betaDCDT) ;
sb= sin(betaDCDT) ;

B_angle=1/cb *[cg -sg 0 ;
cb*sg cb*cg 0 ;
-sb*cg sb*sg cb];

deulerDCDT=B_angle*wDCDT;

%Kinematics Target Orbital
alphaDTo = eulerDTo_i(1) ; betaDTo = eulerDTo_i(2) ;gammaDTo = eulerDTo_i(3) ;

cg= cos(gammaDTo);
sg= sin(gammaDTo);
cb= cos(betaDTo);
sb= sin(betaDTo);

wDTo=[0 0 0]';

B_angle=1/cb*[cg -sg 0;
cb*sg cb*cg 0;
-sb*cg sb*sg cb];

dAngleDTo=B_angle*wDTo;

%Relative Dynamics
% ICDC=[ ICDC11 ICDC12 ICDC13;
%     ICDC21 ICDC22 ICDC23;
%     ICDC31 ICDC32 ICDC33];
% ITDT=[ ITDT11 ITDT12 ITDT13;
%     ITDT21 ITDT22 ITDT23;
%     ITDT31 ITDT32 ITDT33];

% TDC=[TxDC TyDC TzDC]';
 TDT=[0 0 0]';
ADTo=R3(gammaDTo)*R2(betaDTo)*R1(alphaDTo);
ADCDT=R3(gammaDCDT)*R2(betaDCDT)*R1(alphaDCDT) ;



%wDCDT=[wxDCDT;wyDCDT;wzDCDT];
wo=[0 -w0 0]';
wIT=wDTo+ADTo*wo;

%Dynamics for attitude DTO
dwDTo=ITDT\(TDT-skew(wIT)*(ITDT*wIT));
%dwxDTo=dwDTo(1);
%dwyDTo=dwDTo(2);
%dwzDTo=dwDTo(3);


%Dynamics for attitude DCDT
dwDCDT=ICDC\(TDC- skew(wDCDT+ADCDT*wIT)*(ICDC*(wDCDT+ADCDT*wIT)))...
-(skew(-wDCDT)*(ADCDT*wDTo)+ADCDT*dwDTo);
%dwxDCDT=dwDC(1);
%dwyDCDT=dwDC(2);
%dwzDCDT=dwDC(3);

%P2P Translation dynamics
rTo=[0; 0; -rT];
%sDCDT=[sxDT; syDT; szDT];
%dsDCDT=[dsxDT; dsyDT; dszDT];
%rDCDC=[rxDCDC; ryDCDC; rzDCDC];
%rDTDT=[rxDTDT; ryDTDT; rzDTDT];
rDCDT=ADCDT'*rDCDC;
%rxCDT=rDCDT(1);
%ryCDT=rDCDT(2);
%rzCDT=rDCDT(3);
%FDC=[FxDC FyDC FzDC]';
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


