function [dX] = P2P_DynCin(X, U)
    % Nonlinear, 6-dof model of the chaser-target relative coupled dynamics
    % (assuming a fixed target in the orbital local frame). dc is the
    % chaser docking port frame and dt the target docking port frame.
    % X is a 13 x 1 vector whose components are as follows (BE CAREFUL OF THE FRAMES):
    %   1:4 =  Attitude quaternion Q^{DCDT}
    %   5:7 = rotational speed of DC with respect to DT, in DC frame
    %   8:10 = relative position vector of DC with respect to DT, in DT
    %   frame
    %   11:13 = relative velocity of DC with respect to DT , in DT frame
    %
    %  dX is the time derivative of vector X
    %  U is the command vector whose components are as follows : 
    %       1:3 Control input torque 
    %       4:6 Control input force
    %   
    

    TDC =  U(1:3);
    FDC =  U(4:6);


    QDCDT = X(1:4);
    wDCDT= X(5:7);
    sDCDT = X(8:10);
    dsDCDT = X(11:13); 



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
    
     QDTo = euler2quat(eulerDTo_i);
   

    %% load ICDC.mat
    %ICDC = 5*eye(3);
    %mC = 50; %kg

    %% load ITDT.mat
    %ITDT = 40*eye(3);

    %% load constants.mat % mu
    %mu = 398600.4418e9; % m^3/s^2
    

    %% load orbitalParams.mat % w0
   % rT = 6378e3 + 400e3; % en m 
    
   % w0= sqrt(mu/rT^3);
    
    % Target Inertia parameters expressed in docking port frame
    %syms ITDT11 ITDT12 ITDT13 ITDT21 ITDT22 ITDT23 ITDT31 ITDT32 ITDT33 real
    % Docking ports positions expressed in docking port frame
   % syms rxDTDT ryDTDT rzDTDT rxDCDC ryDCDC rzDCDC real
    % relative position variables
   % syms sxDT syDT szDT rT dsxDT dsyDT dszDT mu real
    % Control Input
   % syms TxDT TyDT TzDT TxDC TyDC TzDC FxDC FyDC FzDC real
    % Other parameters
    %syms mu w0 real
    % Linerisation point
    %syms aDT0 bDT0 cDT0 real

    % Fixed target docking port w.r.t. LVLH frame
    wxDTo=0;
    wyDTo=0;
    wzDTo=0;

    %% Relative kinematics Target (DT) - Chaser (DC)

    dQDCDT = -1/2*ProdQuat(QDCDT,[0;ChgtRep(QDCDT,wDCDT)]); % wDCDT is by default in chaser frame, the formula requires it in target + we need wDTDC and not wDCDT
    %dQDCDT = -1/2*ProdQuat(QDCDT,[0;wDCDT]);
    
    %dQDCDT = -1/2*ProdQuat(invQuat(QDCDT),[0;ChgtRep(invQuat(QDCDT),wDCDT)]); 

    %AngleDC=[alphaDCDT; betaDCDT;gammaDCDT] ;

    %cg=cos(gammaDCDT) ;
    %sg= sin(gammaDCDT) ;
    %cb=cos(betaDCDT) ;
    %sb= sin(betaDCDT) ;

    %B_angle=1/cb *[cg -sg 0;
    %cb*sg cb*cg 0 ;
    %-sb*cg sb*sg cb];

    %dAngleDC=B_angle*[wxDCDT;wyDCDT;wzDCDT];

    %% Kinematics Target Orbital
    %AngleDTo=[alphaDTo ; betaDTo ;gammaDTo];

    %cg= cos(gammaDTo);
    %sg= sin(gammaDTo);
    %cb= cos(betaDTo);
    %sb= sin(betaDTo);

    %B_angle=1/cb*[cg -sg 0;
    %cb*sg cb*cg 0;
    %-sb*cg sb*sg cb];

    %dAngleDTo=B_angle*[wxDTo;wyDTo;wzDTo];



    %Relative Dynamics
    % ICDC=[ ICDC11 ICDC12 ICDC13;
    %     ICDC21 ICDC22 ICDC23;
    %     ICDC31 ICDC32 ICDC33];
    % ITDT=[ ITDT11 ITDT12 ITDT13;
    %     ITDT21 ITDT22 ITDT23;
    %     ITDT31 ITDT32 ITDT33];

    %TDC=[TxDC TyDC TzDC]';
    TDT=[0 0 0]';

    %ADTo=R3(gammaDTo)*R2(betaDTo)*R1(alphaDTo);
    %ADCDT=R3(gammaDCDT)*R2(betaDCDT)*R1(alphaDCDT);



    wDTo=[wxDTo;wyDTo;wzDTo];
    %wDCDT=[wxDCDT;wyDCDT;wzDCDT];
    wo=[0 -w0 0]';

    wIT=wDTo+ChgtRep(QDTo,wo);
    %wIT2=wDTo+ADTo*wo;

    % alphaDTo = 0.2;
    % betaDTo = 0.1;
    % gammaDTo = 0.6;
    % 
    % 
    % wIT = vpa(eval(wIT),5);
    % wIT2 = vpa(eval(wIT2),5);



    %Dynamics for attitude DTO
    dwDTo=ITDT\(TDT-skew(wIT)*(ITDT*wIT));
    % dwxDTo=dwDTo(1);
    % dwyDTo=dwDTo(2);
    % dwzDTo=dwDTo(3);

    %Dynamics for attitude DCDT
    dwDC=ICDC\(TDC- skew(wDCDT+ChgtRep(QDCDT,wIT))*(ICDC*(wDCDT+ChgtRep(QDCDT,wIT))))...
    -(skew(-wDCDT)*(ChgtRep(QDCDT,wDTo))+ChgtRep(QDCDT,dwDTo));
    % dwxDCDT=dwDC(1);
    % dwyDCDT=dwDC(2);
    % dwzDCDT=dwDC(3);

    %P2P Translation dynamics
    rTo=[0; 0; -rT];
   % sDCDT=[sxDT; syDT; szDT];
   % dsDCDT=[dsxDT; dsyDT; dszDT];
%    rDCDC=[rxDCDC; ryDCDC; rzDCDC];
%    rDTDT=[rxDTDT; ryDTDT; rzDTDT];
    rDCDT=ChgtRep(invQuat(QDCDT),rDCDC);
    % rxCDT=rDCDT(1);
    % ryCDT=rDCDT(2);
    % rzCDT=rDCDT(3);
    %FDC=[FxDC FyDC FzDC]';
   
    rcDT=ChgtRep(QDTo,rTo)+sDCDT-rDCDT+rDTDT;

    accDT=mu*ChgtRep(QDTo,rTo)/norm(ChgtRep(QDTo,rTo))^3-mu*(rcDT)/norm(rcDT)^3+ChgtRep(invQuat(QDCDT),FDC)/mC;
    s=sDCDT-rDCDT+rDTDT;
    ddsDCDT=-skew(dwDTo)*s...
        - skew(wDTo)*skew(wDTo)*s...
        - skew(ChgtRep(QDTo,wo))*skew(ChgtRep(QDTo,wo))*s...
        -2*skew(wDTo)*dsDCDT...
        -2*skew(ChgtRep(QDTo,wo))*dsDCDT...
        -2*skew(ChgtRep(QDTo,wo))*skew(wDTo)*s...
        +2*skew(ChgtRep(QDTo,wo)+wDTo)*skew(ChgtRep(invQuat(QDCDT),wDCDT))*(rDCDT)...
        +accDT...
        +skew(ChgtRep(invQuat(QDCDT),dwDC))*(rDCDT)...
        +2*skew(ChgtRep(invQuat(QDCDT),wDCDT))*skew(ChgtRep(invQuat(QDCDT),wDCDT))*rDCDT;


    dX  = [dQDCDT; dwDC; dsDCDT; ddsDCDT];

end



% %Compute jacobian
% ftot =[dAngleDC ;dwDC; dAngleDTo ;dwDTo;dsDCDT;ddsDCDT] ;
% Atot= jacobian (ftot, [ AngleDC' wDCDT' AngleDTo' wDTo' sDCDT' dsDCDT']);
% Btot= jacobian(ftot, [TDC' TDT' FDC']);
% 
% %Linearisation
% alphaDCDT=0;
% betaDCDT=0;
% gammaDCDT=0;
% wxDCDT=0;
% wyDCDT=0;
% wzDCDT=0;
% alphaDTo=aDT0;
% betaDTo=bDT0;
% gammaDTo=cDT0;
% wxDTo=0;
% wyDTo=0;
% wzDTo=0;
% TxDC=0;
% TyDC=0;
% TzDC=0;
% TxDT=0;
% TyDT=0;
% TzDT=0;
% sxDT=0;
% syDT=0;
% szDT=0;
% dsxDT=0;
% dsyDT=0;
% dszDT=0;
% FxDC=0;
% FyDC=0;
% FzDC=0;
% 
% Atot=eval(Atot);
% Btot=eval(Btot);
% Atot=simplify(Atot);
% Btot=(simplify(Btot));