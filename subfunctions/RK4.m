
function Enext= RK4(f, dt,E,U)
    f1 = f(E, U);
    f2=f(E+f1*dt/2,U);
    f3=f(E+f2*dt/2,U);
    f4=f(E+f3*dt,U);
    
    Enext = E + dt*(1/6*f1+1/3*f2+1/3*f3+1/6*f4);  
end

