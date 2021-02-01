function Euler=Quat2Euler(Q)
% Returns a vector whose norm corresponds to the rotation angle
% (between 0 and pi of quaternion Q
    Q=Q/norm(Q);
    if Q(1)<0 then Q=-Q; end   %Positive real part
    w=Q(1);q=Q(2:4);
    if (w==1)
        Euler=[0;0;0];
    else
        Euler=2*atan(norm(q),w)*q/norm(q);  % Euler vector
    end
end

