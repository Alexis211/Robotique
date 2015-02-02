Projet de robotique pour le cours de JP Laumond 2014-2015.

Alex Auvolat--Bernstein, Jean Fabre-Monplaisir.


## Présentation

Pour ce projet nous avons implémenté une méthode de recherche de chemin pour le
robot Hilare composé d'une voiture qui tire une remorque simple. Notre solution
se base sur des trajectoires composées d'arcs de cercle et de lignes droites.


## Compilation et lancement

Il est nécessaire d'installer la bibliothèque graphique SFML
<http://www.sfml-dev.org> avant de compiler.

Sous Linux, taper simplement :

	make

Sous Mac OS X, taper :

	g++ -framework sfml-window -framework sfml-graphics -framework sfml-system \
			ui.cpp problem.cpp main.cpp -o pathfind

Ensuite, pour lancer l'application, invoquer le binaire avec :

	./pathfind


## Utilisation de l'interface graphique

Dans l'interface graphique un certain nombre de touches permettent d'accéder aux
fonctionnalités du logiciel :

### Déplacement

Les touches `h`, `j`, `k`, `l` servent à se déplacer respectivement à gauche, en bas, en
haut et à droite.

Les touches `i`, `o` servent à zoomer et dézoomer.

### Définition du problème

Les touches `s`, `e` permettent de définir les positions de départ (start) et
d'arrivée (end) du robot. Lors de la sélection, cliquer une fois pour
positionner le robot, une fois pour l'orienter, puis une fois pour orienter la
remorque. À tout moment un clic droit annule la sélection.

La touche `a` permet d'ajouter un obstacle, et la touche `d` permet de supprimer
l'obstacle sous la souris.

### Résolution du problème

La touche `g` permet de lancer le solveur. Les données intermédiaires (positions
aléatoires et graphe de connection) sont affichées en gris claire, et lorsqu'une
solution est trouvée, celle-ci s'affiche en vert.

La touche `f` permet de faire se déplacer le robot le long de la courbe solution,
lorsque celle-ci est définie.


## Bugs connus

- Ne pas redimentionner la fenêtre d'affichage
- La position de la souris est parfois mal interprétée (résolu ?)
- Pas de commande pour arêter l'algorithme de recherche de chemin (on peut
  néanmoins le relancer, ce qui a pour effet d'interrompre l'exécution
  précédente)



