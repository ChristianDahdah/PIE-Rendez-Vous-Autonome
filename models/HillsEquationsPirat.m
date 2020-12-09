%Define variables for symbolic toolbox
%Forces Input
syms Fx Fy Fz real
% Chaser mass and gravitational param
syms m mu real
%Target parameter s
syms rt w0 real
%Relative position
syms sx sy sz real
%Relative velocity
syms dsx dsy dsz real
%Vectors in orbital frame
r=[0 0 -rt]';
w=[0 -w0 0]';
s=[sx sy sz]';
ds=[dsx dsy dsz]';
F=[Fx Fy Fz]';
acc=-skew(w)*skew(w)*s-2*skew(w)*ds+w0^2*r-mu*(s+r)/norm(s+r)^3+F/m;
A=jacobian([ds ; acc] , [s;ds]);
B=jacobian([ds ; acc] , F);
%Linearisation points
sx=0;
sy=0;
sz=0;
dsx=0;
dsy=0;
dsz=0;
Fx=0;
Fy=0;
Fz=0;
A=eval(A) ;
B=eval(B) ;
A=subs(A,mu*rt*abs(rt)*sign(rt)/(abs(rt)^2)^(5/2) ,w0^2) ;
A=subs(A,(mu)/(abs(rt)^2)^(3/2),w0^2) ;