%ProdQuat(R,S) : return the product of quaternions R and S
function Q=ProdQuat(R, S)
    Rr=R(1);Ri=R(2:4);
    Sr=S(1);Si=S(2:4);
    Q=[Rr*Sr-Ri'*Si;Rr*Si+Sr*Ri+cross(Ri,Si)];
end

