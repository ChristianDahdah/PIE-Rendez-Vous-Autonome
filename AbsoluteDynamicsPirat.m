%Define variablesfor symbolic toolbox
% Euler angles and their  derivatives
syms alpha beta gamma dalpha dbeta dgamma real
% Rotation rates and their derivatives
syms wx wy wz dwx dwy dwz w0 real
% Torques Input
syms Tx Ty Tz real
% Inertia tensor
syms I11 I12 I13 I21 I22 I23 I31 I32 I33 real
% Linearisation points for the attitude
syms a b c real
%Kinematics
Angle=[alpha ; beta ; gamma] ;

cg=cos(gamma) ;
sg=sin(gamma) ;
cb=cos(beta) ;
sb=sin(beta) ;
B_angle=1/cb *[ cg -sg 0 ;
cb*sg cb*cg 0 ;
-sb*cg sb*sg cb ] ;
dAngle=B_angle*[wx ;wy ;wz] ;
%Dynamics
I=[ I11 I12 I13 ;
I21 I22 I23 ;
I31 I32 I33 ] ;
T=[Tx Ty Tz ]';
Abo=R3(gamma)*R2(beta)*R1(alpha) ; %o r b i t a l to body a t t i t u d e matrix
Omega=[wx ;wy ;wz] ;
Omega0=[0 -w0 0]';
w=Omega+Abo*Omega0 ;
dOmega=I \(T- skew(w) *( I*w) ) ;
dwx=dOmega ( 1 ) ;
dwy=dOmega ( 2 ) ;