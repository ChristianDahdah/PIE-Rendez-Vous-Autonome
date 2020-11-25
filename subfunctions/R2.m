function [R] = R2(alpha)
%R1 Returns the direction cosine matrix (DCM) associated with a rotation
%arounf the second (y) axis 
% alpha is the rotation anfle in radians

R = [cos(alpha) 0 -sin(alpha);
    0 1 0;
    sin(alpha) 0 cos(alpha)];

end

