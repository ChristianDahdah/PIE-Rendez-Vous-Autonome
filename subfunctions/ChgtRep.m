function UR2=ChgtRep(QR1versR2, UR1)
    % Expression d'un vecteur dans un autre repère
  QUR2=ProdQuat(ProdQuat(invQuat(QR1versR2),[0;UR1]),QR1versR2);
  UR2=QUR2(2:4);
end

