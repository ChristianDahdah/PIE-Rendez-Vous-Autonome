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

    %eulerDCDT = X(1:3);
    wDCDT= X(5:7);
    sDCDT = X(8:10);
    dsDCDT = X(11:13); 


% Import simulation parameters
 load('parameters.mat');
 paramMu = load('parameters.mat', 'mu');
 %w0 = params

 
 
 %load parameters.mat

% Kinematics P2P
dQDCDT = -1/2*ProdQuat(invQuat(QDCDT),[0;ChgtRep(invQuat(QDCDT),-wDCDT)]); 

% Kinematics Target wrt. LVLH (local orbital)
QDTo = euler2quat(eulerDTo_i);

wDTo=[0;0;0];   % Fixed target docking port w.r.t. LVLH frame

%Relative Dynamics

wo=[0 -w0 0]';


wIT=wDTo+ChgtRep(QDTo,wo); %

%Dynamics for attitude DTO

dwDTo = [0;0;0];

%Dynamics for attitude DCDT
%dwDCDT=ICDC\(TDC- skew(wDCDT+ADCDT*wIT)*(ICDC*(wDCDT+ADCDT*wIT)))...
%-(skew(-wDCDT)*(ADCDT*wDTo)+ADCDT*dwDTo);
dwDCDT=ICDC\(TDC- skew(wDCDT+ChgtRep(QDCDT,wIT))*(ICDC*(wDCDT+ChgtRep(QDCDT,wIT))))...
-(skew(-wDCDT)*(ChgtRep(QDCDT,wDTo))+ChgtRep(QDCDT,dwDTo));

%P2P Translation dynamics
rTo=[0; 0; -rT];
% sDCDT=[sxDT; syDT; szDT];
% dsDCDT=[dsxDT; dsyDT; dszDT];

%rDCDT=ADCDT'*rDCDC;
rDCDT=ChgtRep(invQuat(QDCDT),rDCDC);

rxCDT=rDCDT(1);
ryCDT=rDCDT(2);
rzCDT=rDCDT(3);

rcDT=ChgtRep(QDTo,rTo)+sDCDT-rDCDT+rDTDT;

accDT=paramMu.mu*ChgtRep(QDTo,rTo)/norm(ChgtRep(QDTo,rTo))^3-paramMu.mu*(rcDT)/norm(rcDT)^3+ChgtRep(invQuat(QDCDT),FDC/mC);
s=sDCDT-rDCDT+rDTDT;
ddsDCDT=-skew(dwDTo)*s...
    - skew(wDTo)*skew(wDTo)*s...
    - skew(ChgtRep(QDTo,wo))*skew(ChgtRep(QDTo,wo))*s...
    -2*skew(wDTo)*dsDCDT...
    -2*skew(ChgtRep(QDTo,wo))*dsDCDT...
    -2*skew(ChgtRep(QDTo,wo))*skew(wDTo)*s...
    +2*skew(ChgtRep(QDTo,wo)+wDTo)*skew(ChgtRep(invQuat(QDCDT),wDCDT))*(rDCDT)...
    +accDT...
    +skew(ChgtRep(invQuat(QDCDT),dwDCDT))*(rDCDT)...
    +2*skew(ChgtRep(invQuat(QDCDT),wDCDT))*skew(ChgtRep(invQuat(QDCDT),wDCDT))*rDCDT;

    dX  = [dQDCDT; dwDCDT; dsDCDT; ddsDCDT];

end



