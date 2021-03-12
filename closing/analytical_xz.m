function [x,z,vx,vz] = analytical_xz(x0,z0,vx0,vz0,fpx,fpz,w,t)
% propagation du modèle linéaire sur une durée t avec des accélérations
% constante fpx et fpz.
z = 4*z0 - 2*vx0/w + fpz/(w*w) + (2*vx0/w - 3*z0 - fpz/(w*w))*cos(w*t) + (vz0/w + 2*fpx/(w*w))*sin(w*t) - 2*fpx/w*t;
x = x0 + 2*vz0/w + 4*fpx/(w*w) - 3*vx0*t + 6*z0*w*t + 2*fpz/w*t - (2*vz0/w + 4*fpx/(w*w))*cos(w*t) + (4*vx0/w - 6*z0 - 2*fpz/(w*w))*sin(w*t) - 3/2*fpx*t*t;
vz = (-2*vx0 + 3*z0*w + fpz/w)*sin(w*t) + (vz0 + 2*fpx/w)*cos(w*t) - 2*fpx/w;
vx = -3*vx0 + 6*z0*w + 2*fpz/w + (2*vz0 + 4*fpx/w)*sin(w*t) + (4*vx0 - 6*z0*w - 2*fpz/w)*cos(w*t) - 3*fpx*t;

end

