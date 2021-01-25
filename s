[33mcommit a12b70d3e855d7e9925a4edca5a649c0e8e4346b[m[33m ([m[1;36mHEAD -> [m[1;32mmodelisation-sat[m[33m, [m[1;31morigin/modelisation-sat[m[33m)[m
Author: gastondeboni <gaston.deboni@gmail.com>
Date:   Wed Dec 2 12:20:05 2020 +0100

    [Add shift] Adding shift function necessary for executing MPC

[33mcommit 603257db33a84a9f260880d9395f4de1fd9a34e7[m
Merge: 4404923 574efa4
Author: gastondeboni <gaston.deboni@gmail.com>
Date:   Wed Dec 2 12:12:37 2020 +0100

    Merge branch 'modelisation-sat' of https://github.com/ChristianDahdah/PIE-Rendez-Vous-Autonome into modelisation-sat

[33mcommit 4404923b17255c07a8fecb5cd268a878dbd9958a[m
Author: gastondeboni <gaston.deboni@gmail.com>
Date:   Wed Dec 2 12:09:16 2020 +0100

    [Add MPC code] Adding MPC optimization code with Multiple Shooting, Runge Kutta 4 and obstacle avoidance. Depending on the control limits and time step, the obstacle avoidance may not work and even destabilize the system.

[33mcommit 574efa4281ceef7aeaf809b80e7c8d47d5315005[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Dec 2 11:36:39 2020 +0100

    Correction de typos

[33mcommit 047d20645752581739e31df639e6d067040ee804[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Dec 2 11:09:17 2020 +0100

    Ajout de catégories : models, control, et navigation

[33mcommit cdfffb0efd51cbaa2159a158de9538bc0e77008e[m
Author: KevinMillien97 <kevimili97@hotmail.com>
Date:   Wed Dec 2 10:08:24 2020 +0100

    [Files added] P2P dynamics, H_inf and LQG synthesis but the latter two aren't 100% operational

[33mcommit 6772463d43b0d43d40857ff9d55c10df60f78775[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 12:16:02 2020 +0100

    [Upload] Ajout des filtres de navigation basée vision à 3 et 5 LEDs

[33mcommit 47aae40f35e7ad979a45d4711e5f2b45a9342de2[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 11:57:05 2020 +0100

    Typos 2

[33mcommit f2db4d8650481fdcfebd4fd0723ffbf509553f9a[m
Author: Chris <christiandahdah@gmail.com>
Date:   Wed Nov 25 11:39:33 2020 +0100

    [Adding skew.m] Adding skew.m necessary for AbsoluteDynamicsPirat.m

[33mcommit d29b29d3aae864bf26ab6f55f7e9b8c529bdbaf9[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 11:36:02 2020 +0100

    Correction typo dans HillsEquations

[33mcommit e4af751fa4543a9564c6f1f21fe84e671663f691[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 11:30:19 2020 +0100

    [Upload] Ajout du modèle de Hill utilisé par Pirat

[33mcommit 2b9b20f9780f569fd326514cac6432d0bcca7b98[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 11:15:05 2020 +0100

    Rajout de fonctions annexes dans un répertoire subfunctions

[33mcommit ead4ffcc4551e0cd7f392f7d37de0a5b6cf71dc8[m
Author: Charles Dehombreux <charles.dehombreux@student.isae-supaero.fr>
Date:   Wed Nov 25 10:38:29 2020 +0100

    [Upload] Chargement modèle Pirat Absolute Dynamics

[33mcommit b4c4ac170721cbdf7aad893add47e6af073c764a[m[33m ([m[1;31morigin/master[m[33m, [m[1;31morigin/HEAD[m[33m, [m[1;32mmaster[m[33m)[m
Author: ChristianDahdah <63374139+ChristianDahdah@users.noreply.github.com>
Date:   Mon Oct 5 21:05:25 2020 +0200

    Update README.md
    
    [Fix README.md] Oublie d'une </b>

[33mcommit 4efbc63d6b5860592892b739e43d13668790f72f[m
Author: ChristianDahdah <63374139+ChristianDahdah@users.noreply.github.com>
Date:   Mon Oct 5 21:03:11 2020 +0200

    Update README.md
    
    [README.md git commands] Version 1 du Tutoriel Github. Il faut ajouter un tuto sur .gitignore et mettre plus en relief le code que les commentaires.

[33mcommit 815ac134988f6beb14742eb7f91207cd7b3022ae[m
Author: ChristianDahdah <63374139+ChristianDahdah@users.noreply.github.com>
Date:   Wed Sep 30 10:11:39 2020 +0200

    Initial commit
