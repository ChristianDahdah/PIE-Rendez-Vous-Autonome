function IQ=invQuat(Q)
    % Calcul l'inverse d'un quaternion
IQ=[Q(1);-Q(2);-Q(3);-Q(4)]/norm(Q)^2;
end


