function [R] = R1(alpha)
%R1 Returns the direction cosine matrix (DCM) associated with a rotation
%arounf the first (x) axis 
% alpha is the rotation anfle in radians

R = [1 0 0;
    0 cos(alpha) sin(alpha);
    0 -sin(alpha) cos(alpha)];

end

