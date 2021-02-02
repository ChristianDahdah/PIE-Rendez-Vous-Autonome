function [euler]=Quat2EulerAngles(R)
    q0 = R(1);
    q1 = R(2);
    q2 = R(3);
    q3 = R(4);
    
    alpha = atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
    beta = asin(2*(q0*q2-q3*q1));
    gamma = atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    
    euler = [alpha;beta;gamma];
end