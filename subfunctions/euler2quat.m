function quat = euler2quat(euler)

alpha = euler(1);
beta = euler(2);
gamma = euler(3);

ca = cos(alpha*0.5);
sa = sin(alpha*0.5);
cb = cos(beta*0.5);
sb = sin(beta*0.5);
cg = cos(gamma*0.5);
sg = sin(gamma*0.5);



%qw = ca*cb*cg-sa*sb*sg;
%qx=

q1 = [ca; sa; 0; 0];
q2 = [cb; 0; sb; 0];
q3 = [cg; 0; 0; sg];

quat = ProdQuat(q1, ProdQuat(q2,q3));

quat = quat';


% qw = ca * cb * cg - sa * sb * sg;
% 
% qx = sa * cb * cg + ca * sb * sg;
% 
% qy = ca * sb * cg + sa * cb * sg;
% 
% qz = ca * cb * sg - sa * sb * cg;
% 
% %To verifie the order of qz and qy
% quat = [qw qx qz qy]

end