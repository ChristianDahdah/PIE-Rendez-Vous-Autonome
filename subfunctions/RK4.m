
function Enext= RK4(f,t,E,dt)
    f1 = f(t,E);
    f2=f(t+dt/2,E+f1*dt/2);
    f3=f(t+dt/2,E+f2*dt/2);
    f4=f(t+dt/2,E+f3*dt);
    
    Enext = E + dt*(1/6*f1+1/3*f2+1/3*f3+1/6*f4);  
end

