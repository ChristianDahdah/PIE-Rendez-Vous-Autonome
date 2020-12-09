s=tf('s'); %define the Laplace parameter
%% Define the parameters for the slohsing linear model
% tank rotation rate linerasitation point
wx=0;
wy=0; %-TARGET_MEAN_MOTION;
wz=0;
%tank position
rx=0.1;
ry=0;
rz=0;
% natural frequency and damping coefficient
cs=0.33 ;
fs=.025 ;
%Filling ratio
tau=0.44 ;
alphas=0.6 ;
lambda=tau*(4* alphas - 1)+tau ^2*(2 -4* alphas) ;
mprop=tau *2;
ml=(1-lambda)*mprop ;
%% Build the Plant with the Sloshing
%Select only the position
C=[1 0 0 0 0 0 0 0 0 0 0 0 ;
   0 1 0 0 0 0 0 0 0 0 0 0 ;
   0 0 1 0 0 0 0 0 0 0 0 0 ;
   0 0 0 0 0 0 1 0 0 0 0 0 ;
   0 0 0 0 0 0 0 1 0 0 0 0 ;
   0 0 0 0 0 0 0 0 1 0 0 0 ] ;
B=[zeros(3,6);
    eye(3), zeros(3);
    zeros(3,6);
    zeros(3) eye(3)];

B_in=B_DCDT([4:6 10:12],:);

if ml==0
    G_P2P=ss(F_DCDT,B,C, 0);
    G1=G_P2P*B_in*D;
else
    ks=4*pi^2*tau*ml*fs ^2;
    As = A_sloshing(cs ,ks, ml, wx, wy , wz);
    Bs = B_sloshing( rx , ry , rz ) ;
    Cs = C_sloshing( cs , ks , rx , ry , rz);
    G_P2P= ss (F_DCDT,B,C, 0 ) ;
    Gs=ss (As , Bs , Cs , 0 ) ;
    G1=G_P2P*( eye ( 6 )+Gs )*B_in ;
end
%% Define the scales
Y=diag([ones(1,3)*1, ones(1,3 )*.5 ]);
U=diag([ones(1,3)*2e-3 , ones(1,3)*30e-3]);
G=inv(Y)*G1*U;
%% Input weight
%Actuators errors
wdt=1e-1;
wdr=1e-1;
wdrt=0.05;
A=abs(skew([1 1 1]*wdrt));
Wd=ss(diag([ones(1,3)*wdt ones(1,3)*wdr]));
Wd(1:3,4:6)=A;
%Navigation noise
wnt=1*pi/180;
wnr=1e-2;
Wn=ss(diag([ones(1,3)*wnt ones(1,3)*wnr]));
%Reference trajectory weight
Wr=eye(6);
%% Output weight
%Performance S
M11=2;
A11=0.05 ;
wc11=4/80;
w11=(1/M11*s+wc11 ) /( s+A11*wc11 ) ;
M12=2;
A12=0.05 ;
wc12=4/80;
w12=(1/M12*s+wc12)/( s+A12*wc12 ) ;
W1=tf(eye(6));
W1(1:3,1:3)=eye(3)*w11 ;
W1(4:6,4:6)=eye(3)*w12 ;
% Performance T
M21=10;
A21=0.05;
wc21=wc11*20;
w21=(s+A21*wc21)/(1/M21*s+wc21 ) ;
M22=10;
A22=0.05 ;
wc22=wc12 *20;
w22=(s+A22*wc22)/(1/M22*s+wc22 ) ;
W2=tf(eye(6));
W2(1:3,1:3)=eye(3)*w21;
W2(4:6,4:6)=eye(3)*w22;

% Noise sensitivity KS
M31=0.01;
A31=1000;
wc31=wc21*1;
w31=(1/M31*s+wc31)/( s+A31*wc31 ) ;
M32=0.01;
A32=1000;
wc32=wc22*1;
w32=(1/M32*s+wc32)/( s+A32*wc32 ) ;
W3=tf(eye(6));
W3(1:3,1:3)=eye(3)*w31 ;
W3( 4 : 6 , 4 : 6 )=eye ( 3 )*w32 ;
%% Build the generalised Plant
systemnames = 'Wr Wd Wn W1 W2 W3 G' ;
inputvar = '[ref(6); dist(6) ; noise(6) ; u(6)]' ;
outputvar = '[W1;W2;W3;Wr-G-Wn]';
input_to_G= '[u+Wd]';
input_to_Wr='[ref]' ;
input_to_Wd='[dist]' ;
input_to_Wn='[noise]' ;
input_to_W1='[Wr-G-Wn]';
input_to_W2='[G]';
input_to_W3='[u]' ;
cleanupsysic = 'yes' ;
P = sysic ;
%% Hinf synthesis
[K,CL, gamma, Info] = hinfsyn(P, 6 , 6 , 'Display' ,'on') ;
%Unscale the controller
K1=(U)*K/(Y) ;