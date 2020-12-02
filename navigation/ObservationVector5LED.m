%state variables
syms x y z vx vy vz real
syms alpha beta gamma wx wy wz real
%Camera v a r i a b l e s
syms f D zmax ymax Azmax Elmax real
ADCDT=R3(gamma)*R2( beta )*R1( alpha );
sCT=[x y z ]'; %Position from chaser to target in chaser frame
dsCT=[vx vy vz]';
Omega=[wx wy wz]';
Angle=[alpha beta gamma]';
X=-(ADCDT)*sCT; %Position from target to chaser in t a r g e t frame
xC=X( 1 ) ;
yC=X( 2 ) ;
zC=(X( 3 ) ) ;
Az=atan2 (yC, xC) ;
El=atan2 (-zC, sqrt(xC*xC+yC*yC)) ;
Xcentre =[0; yC/xC*ymax/tan(Azmax) ; zC/sqrt(xC*xC+yC*yC)*zmax/tan(Elmax)];
R=sqrt(xC*xC+yC*yC+zC*zC) ;
x1=[0 1 0]'*D*f/R;
x2=[0 0 1]'*D*f/R;
x3=[0 -1 0]'*D*f/R;
x4=[0 0 -1]'*D*f/R;
x5=[ -1 0 0]'*D*f/R;
ALED=R3(gamma+Az)*R2(beta+El)*R1(alpha);
Y1=ALED*x1+Xcentre;
Y2=ALED*x2+Xcentre;
Y3=ALED*x3+Xcentre;
Y4=ALED*x4+Xcentre;
Y5=ALED*x5+Xcentre;
h1=Y1(2);
h2=Y1(3);
h3=Y2(2);
h4=Y2(3);
h5=Y3(2);
h6=Y3(3);
h7=Y4(2);
h8=Y4(3);
h9=Y5(2);
h10=Y5(3);
h=[h1 ; h2 ; h3 ; h4 ; h5 ; h6 ; h7 ; h8 ; h9 ; h10 ] ;
H=jacobian(h,[Angle ; Omega ; sCT; dsCT]);