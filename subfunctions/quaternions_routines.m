function Enext = RK1(f,t,E,dt)
    Enext = E + dt*f(t,E);
end
