function EE = MySolver(f,T,E0,RK)
    EE =zeros(length(E0),length(T));
    EE(:,1) = E0;
    for n=1:length(T)-1
        EE(:,n+1) = RK(f,T(n),EE(:,n), T(n+1)-T(n) );
    end
end

