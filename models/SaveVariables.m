Ar = [Atot(1:6,1:6) Atot(13:18,1:6);
          Atot(13:18,1:6) Atot(13:18,13:18)]
      
Br = [Btot(1:6,1:3) Btot(1:6,7:9);
          Btot(13:18,1:3) Btot(13:18,7:9)]

save('matrices', 'Atot', 'Ar', 'Btot', 'Br')
