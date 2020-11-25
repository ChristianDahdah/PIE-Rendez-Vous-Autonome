function [R] = R3(alpha)
%R1 Returns the direction cosine matrix (DCM) associated with a rotation
%arounf the third (y) axis 
% alpha is the rotation anfle in radians

R = [cos(alpha) sin(alpha) 0;
    -sin(alpha) cos(alpha) 0;
    0 0 1];

end

