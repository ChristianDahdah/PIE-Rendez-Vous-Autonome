function out = B123(beta, gamma)
% Function B123 (Pirat p. 50) allowing to link alpha_dot with the
% associated rotational speed e.g. alpha_dot = B_123 * omega


out = [sec(beta)*cos(gamma) -sec(beta)*sin(gamma) 0;
    sin(gamma) cos(gamma) 0;
    -tan(beta)*cos(gamma) tan(beta)*sin(gamma) 1]

end

