function Q=Vect2Quat(UR1, UR2, VR1, VR2)
% UR1, UR2 : Components of vector U in R1 and R2
% VR1, VR2 : Components of vector V in R1 and R2
% Q : Return the quaternion from R1 to R2
UR1=UR1/norm(UR1);UR2=UR2/norm(UR2);
VR1=VR1/norm(VR1);VR2=VR2/norm(VR2);
DU=UR2-UR1;DV=VR2-VR1;
SU=UR2+UR1;SV=VR2+VR1;
switch sign([norm(DU) norm(DV)])
    case [0 0]
        Q=[1;0;0;0];
    case [0 1]
        Q=[cross(SV,SU)'*DV;DV'*DV*SU];
    case [1 0]
        Q=[cross(SU,SV)'*DU;DU'*DU*SV];
    else
        Q=[DU'*SV;cross(DV,DU)];
end
end