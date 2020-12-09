%Define Input and Output
K.InputName='e' ; %LQR controller
K.OutputName='u' ;
G.InputName='u' ; %P2P dynamics
G.OutputName='x' ;
H.InputName='x' ; % Observation Matrix
H.OutputName='y' ;
Kf.InputName=' ef ' ; %Kalman gain
Kf.OutputName='w1' ;he
Bf.InputName='u' ; %Plant Input matrix
Bf.OutputName='w2' ;
Gf.InputName='w' ; %Dynamics for Kalmn filter
Gf.OutputName='xf' ;
Hf.InputName='xf' ; %Observation matrix for Kalman filter
Hf.OutputName='yf' ;
Sum1=sumblk (' ef=y - yf' , a ) ;
Sum2=sumblk ('e=r - xf' ,12) ;
Sum3=sumblk ('w=w1+w2' ,12) ;
%Build the closed-loop
G_LQG=connect (G,Gf,Kf,Bf,K,H,Hf,Sum1,Sum2,Sum3,'r','x','u') ;
%Get the input gain function
Li=getLoopTransfer(G_LQG,'u',- 1 );