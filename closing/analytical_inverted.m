function [x,y,vx,vy] = analytical(x0,y0,vx0,vy0,fpx,fpy,n,t)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

x = 4*x0 + 2*vy0/n + fpx/(n*n) - (2*vy0/n + 3*x0 + fpx/(n*n))*cos(n*t) + (vx0/n - 2*fpy/(n*n))*sin(n*t) + 2*fpy/n*t;
y = y0 - 2*vx0/n + 4*fpy/(n*n) - 3*vy0*t - 6*x0*n*t - 2*fpx/n*t + (2*vx0/n - 4*fpy/(n*n))*cos(n*t) + (4*vy0/n + 6*x0 + 2*fpx/(n*n))*sin(n*t) - 3/2*fpy*t*t;
vx = (2*vy0 + 3*x0*n + fpx/n)*sin(n*t) + (vx0 - 2*fpy/n)*cos(n*t) + 2*fpy/n;
vy = -3*vy0 - 6*x0*n - 2*fpx/n - (2*vx0 - 4*fpy/n)*sin(n*t) + (4*vy0 + 6*x0*n + 2*fpx/n)*cos(n*t) - 3*fpy*t;

end

