function [t0, x0, u0] = shift_init(T, t0, x0, u,f)
st = x0;
con = u(1,:)';
f_value = f(st,con);
st = st+ (T*f_value);
x0 = full(st);

t0 = t0 + T;
u0 = [u(2:size(u,1),:);u(size(u,1),:)]; %I update the initial guess for u
end

%u_guess=[u2, u3, u4, ..., uN-1, uN, uN] -- The first entry is trimmed
% Every ui is a VECTOR of 3 entries