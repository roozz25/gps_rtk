# GPS RTK

Ce projet a pour but de fournir la position et l'orientation d'un robot mobile utilisant les données d'un GPS RTK ardusimple rtk2b

### Hardware

* 2 simpleRTK2B Budget boards (ZED-F9P)
* 1 simpleRTK2B Lite board (ZED-F9P)
* 3 u-blox GNSS Multiband antenna ANN-MB-00
* 2 Radio Modules LR
* 2 cables micro-usb alimentation/transfert de données

### Fichiers de configuration

* Base
* 5Hz simpleRTK2Blite (Moving Base)
* 5Hz simpleRTK2B (Rover)

### Utilisation
#### Setup
Brancher la carte rtk2b lite sur la carte rtk2b budget qui servira de rover (cf image) 

Si ce n'est pas déja le cas, il faut charger les bon fichier de configuration dans les cartes correspondantes.
le fichier Base dans la carte seule qui servira de base. Connexion via le port usb (power + gps).

*Le fichier 5Hz simpleRTK2Blite (Moving Base) dans la carte budget qui servira de rover.  Connexion via le port usb (power + gps).
*Le fichier 5Hz simpleRTK2B (Rover) dans la carte lite. Connexion via le deuxieme port usb de la carte budget rover.
*Le chargement de fichier dans les cartes se fait via le logiciel u-center.
[Tutoriel de la marque](https://www.ardusimple.com/configuration-files/)

Pour le rover, l'antenne se situant à l'avant du robot doit etre branchée sur la carte budget et l'antenne se situant à l'arrière doit etre branchée sur la carte lite. (cf photo)

Alimenter la base puis le rover via le port usb (power + gps) 
(Si le rover est alimenté avant, il ne communiquera pas les bons messages et le bon fonctionnement du code sera compromis)

#### Application








