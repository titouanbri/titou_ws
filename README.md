# UR30-admittance

## Connecter le robot
- prendre l'ip du robot (sur la tablette du robot, cliquer en haut a droite, puis "info"), puis configurer le pc : dans paramètre/réseau/filaire/IPV4/manuel : mettre dans "adresse" le même IP que le robot, en changeant le dernier chiffre (exemple : robot=192.168.3.200 ->pc=192.168.3.203) et masque de réseau = 255.255.255.0

## Étapes pour configurer le pc pour travailler avec l’ur30 et faire fonctionner l’admittance 

- remplacer « titouan » par username avec vscode (dans ws), ctrl maj f
- sudo apt update
- sudo apt install ros-noetic-desktop-full (suivre un tuto pour installer ros)
- sudo apt install ros-noetic-soem      
- pip install --upgrade scipy
- sudo apt install ros-noetic-kdl-parser-py


## Etapes pour le capteur ethercat
    

- installation : https://gitlab.com/botasys/bota_driver.git bota_driver si jamais ça marche pas
- chat gpt pour les erreurs 

- echo "/opt/ros/noetic/lib" | sudo tee /etc/ld.so.conf.d/ros-noetic.conf
- sudo ldconfig 


- changer le nom du port 
- changer nom du capteur dans le launch 
## Commande finale pour lancer l'acquisition ( se lance avec setup.launch )
- roslaunch rokubimini_ethercat rokubimini_ethercat.launch

## Lancement

- commande pour lancer driver UR + driver ethercat ( à modifier avec les infos du bon robot) : roslaunch force_sensor_node setup.launch
- commande pour lancer l'admittance : rosrun force_sensor_node sensor_test.py (bien attendre que les 2 capteurs soient marqués "OK" dans le terminal)


## chemin des fichiers utilisés :
~/catkin_ws/src/universal_robot/force_sensor_node

## mail du goat, du "broyeur de pins", "foudroyeur d'ur30" :
titouan.briancon@sigma-clermont.fr

## mail du "démagnétiseur", "froisseur de Bota" :
matteo.proverbio@sigma-clermont.fr
