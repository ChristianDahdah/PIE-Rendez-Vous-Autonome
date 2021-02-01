function [r, V]=Kep2Car(mu, a, e, i, W, w, v)
    % This function gives the vector radius and the velocity for
    % the corresponding orbital parameters
    % mu is the gravitational constant (m^3/s^2)
    C=sqrt(mu*a*(1-e^2));
    ci=cos(i);si=sin(i);
    cW=cos(W);sW=sin(W);
    cwpv=cos(w+v);swpv=sin(w+v);
    er=[cW*cwpv-ci*sW*swpv ; sW*cwpv+ci*cW*swpv ; si*swpv];
    eteta=[-cW*swpv-ci*sW*cwpv ; -sW*swpv+ci*cW*cwpv ; si*cwpv];
    r=a*(1-e^2)/(1+e*cos(v))*er;
    V=e*mu/C*sin(v)*er+C/norm(r)*eteta;
end


