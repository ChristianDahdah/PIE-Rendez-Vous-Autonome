function [euler]=quat2euler(q)

    A = (q(1)^2 - norm(q(2:4))^2)*eye(3)-2*q(1)*skew(q(2:4))+2*q(2:4)*q(2:4).';
    
%     alpha = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
%     beta = asin(2*(q0*q2-q3*q1));
%     gamma = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    
    beta = asin(A(3,1));
    gamma = atan2(-A(2,1),A(1,1));
    alpha = atan2(-A(3,2),A(3,3));

    euler = [alpha beta gamma];
    
    

end

