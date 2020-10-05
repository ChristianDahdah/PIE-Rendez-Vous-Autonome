# PIE-Rendez-Vous-Autonome
Projet ingénieurie entreprise pour le développement d'un système de rendez-vous autonome entre deux satellites


## I - Initialisation du répertoire globale and un répertoire local
<b>  1.1 - Mode facile: </b> <br>
git clone https://github.com/ChristianDahdah/PIE-Rendez-Vous-Autonome <br> <br>
<b> 1.2 - Pour se mettre dans le répertoire local </b> <br>
cd PIE-Rendez-Vous-Autonome <br>

<b> 2 - Mode coder </b> <br>
git init <br>
git remote add origin https://github.com/ChristianDahdah/PIE-Rendez-Vous-Autonome <br>
git pull <br> <br>

## II- Démarche habituelle
<b> 0 - A tout temps on peut voir les modifications ou le status de la branche (pour voir les fichiers où il y'a un changement/conflit...) </b> <br>
git status <br>

<b> 1 - Mis-à-jour du répertoire LOCAL (sur machine) </b> <br>
git pull <br>

<b> 2 - Ajouter un fichier pour être suivi par git (optionnel) </b> <br>
git add *FICHIER* <br>
<b> Astuce pour ajouter tous les fichiers présents dans le répertoire locale: </b> <br>
git add . <br>

<b> 3 - Versionner les modifications faites dans le répertoire LOCAL </b><br>
git commit -am "[Bug fixes] fix d'un bug dans le fichier Modelisation.m et Api.c" <br> <br>
<b> le paramètre -a est optionnelle et équivaut à la commande "git add ." <br>
le paramètre -m est très recommandé, voire obligatoire, pour ajouter un message au commit. <br>
le paramètre -m est important pour tracker les modifications dans l'historique et revenir à une version favorable. </b> <br>

<b> 4 - Avant de télécharger les modifications du répertoire LOCAL dans le répertoire GLOBAL, il vaut mieux mettre à jour le répertoire LOCAL pour régler des conflits éventuels. En cas de conflits, référer à la section IV - Conflicts </b> <br>
git pull <br>

<b> 5 - Une fois les conflits réglés, vous êtes prêt à versionner le répertoire GLOBAL </b> <br>
git push <br>


## III - Branch
<b> 1- Voir dans quelle branche on travail <br>
Méthode 1: </b> <br>
git checkout <br> <br>
<b> Méthode 2: l'étoile * désigne dans quelle branche on est </b> <br>
git branch <br>

<b> 2- Se positionner dans une branche existante </b> <br>
git checkout Branch_name <br>

<b> 3- Créer une nouvelle branche et se positionner dedans </b> <br>
git checkout -b New_Branch <br>
git push --set-upstream origin NewBranch <br>
  

## IV - Conflicts when pulling/pushing commits or merging 
<b> 1- Un message sera afficher du type:  <i> "CONFLICT (content): Merge conflict in New_folder/New Read Me.txt" </i> <br>
Pour régler ça, accéder au(x) fichier(s) concerné(s) et régler les conflits manuellement dans chaque fichier. Vour trouverez les conflits de la façon suivante: </b> <br> <br>
<<<<<<< HEAD <br>
Lignes dans ma branche/version que j'ai modifié, encore LOCAL <br>
Ligne 1 <br>
Ligne 2 ... <br>
======= <br>
Lignes dans la branche/version qui est sur le répertoire GLOBAL <br>
Ligne a <br>
Ligne b ... <br>
'>>>>>>> master <br> <br>
<b> "=======" est un séparateur séparant les sections qui causent le conflit. </b> <br> <br>
<b> Il faut choisir quelles lignes garder et quelles lignes supprimer/modifier pour rendre le code compatible. <br>
  Quand on finit on enlève les entêtes "<<<<<<< HEAD", "=======" et ">>>>>>> master" par exemple: </b> <br>
Ligne a <br>
Ligne 1 <br>
<del> Ligne 2 </del> <br>
Ligne b <br> <br>
<b> 2- A la fin pour saveguarder les changements on fait un commit puis un push </b> <br>
git commit -am "[Conflicts resolved] Conflicts between " <br>
git push <br>


## V - Merge branches
<b> 1- Avant de faire merge, il est conseillé de se mettre dans la branche NON principale où on a fait les chagements par rapport au master </b> <br>
git checkout BranchToMerge <br>
<b> 2- On apporte les changements de la branche "master" principale à la branche de travail </b> <br>
git merge master <br> <br>
<b> Dans la majorité des cas il y'aura un conflit, il faut donc se reférer à la section IV - Conflicts. </b> <br> <br>
<b> 3- Enfin après avoir assuré que le code fonctionne bien dans la branche de travail, on peut maintenant apporter ces changements dans la branche master </b> <br>
git checkout master <br>
git merge BranchToMerge <br>


## VI - Other useful commands
<b> 1- Signin in git's terminal: </b> <br>
git config --global user.name "Votre nom ou pseudo" <br>
git config --global user.email "votre@email.com" <br>

<b> 2- Voir l'historique des commits dont le SHA et les descriptions (presser "q" pour sortir) <b> <br>
git log <br>

<b> 3- Pour mettre une version précédente (revenir à un vieux commit) </b> <br>
git checkout *SHA* <br>

<b> 4- Pour revenir à l'état courant et sortir d'une version/commit ancienne </b> <br>
git checkout master <br>

<b> 5- Changer le message du commit (seulement si on a pas fait push encore) </b> <br>
git commit --amend -m "New message" <br>

<b> 6- Créer l'inverse fait jusqu'à un commit déterminé (revenir à un état ancien) </b> <br>
git reverse *SHA* <br>

<b> 7- Effacer tout les changements qui n'ont pas été "commité" encore </b> <br>
git reset --hard

<b> 8- Voir qui est responsable de la modification d'un fichier. La commande affiche le SHA du commit avant chaque ligne qui peut servir de voir la version du code et le message laissé durant le commit </b> <br>
git blame *FILE NAME* 

<b> 9- Voir la description d'un commit et les changements faites </b> <br>
git show *SHA beginning OR entire SHA* <br>

<b> 10 -Sauvegarder les changements sans faire un commit pour être recharger plus tard (donc pas de version faite). <br>
C'est utile pour revenir à un état précédent plus facilement en cas d'urgence en sauvegardant les modifications faites. <br>
Par exemple la commande peut être utiliser pour changer de branche sans avoir besoin d'un commit et sauvegarder les changements pour un autre temps. </b> <br>
git stash <br>

<b> 11- Recharger le stash (les changements sauvegardés mais pas commités) </b> <br>
git stash apply <br>
