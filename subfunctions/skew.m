function [skewA] = skew(A)
%skew returns the skew symmetric matrix of vector A
%It is useful for vector cross product: A x B = skew(A).B

skewA = [0 -A(3) A(2);
    A(3) 0 -A(1);
    -A(2) A(1) 0];

end
