# UR30-admittance

## Connecter le robot
- dans paramètre/réseau/filaire/IPV4/manuel : mettre dans "adresse" le même IP que le robot, en changeant le dernier chiffre donc 192.168.3.20X avec X différent de 0 et masque de réseau = 255.255.255.0

## Étapes pour configurer le pc pour travailler avec l’ur30 et faire fonctionner l’admittance 

- remplacer « titouan » par username avec vscode (dans ws), ctrl maj f
- sudo apt update
- sudo apt install ros-noetic-desktop-full (suivre un tuto pour installer ros)
- sudo apt install ros-noetic-soem      
- pip install --upgrade scipy
- sudo apt install ros-noetic-kdl-parser-py


## Etapes pour le capteur ethercat
    

- installation : https://gitlab.com/botasys/bota_driver.git bota_driver si jamais ça marche pas
- sudo apt install ros-noetic-ethercat-grant

- echo "/opt/ros/noetic/lib" | sudo tee /etc/ld.so.conf.d/ros-noetic.conf
- sudo ldconfig 

dans bota_driver/rokubi_ethercat :
- changer le nom du port 
- changer nom du capteur dans le launch
- chat gpt pour les erreurs

## Commande finale pour lancer l'acquisition ( se lance avec setup.launch )
- roslaunch rokubimini_ethercat rokubimini_ethercat.launch

## Lancement

- commande pour lancer driver UR + driver ethercat ( à modifier avec les infos du bon robot) : roslaunch carnicero setup.launch
- commande pour lancer l'admittance : rosrun carnicero sensor_test.py (bien attendre que les 2 capteurs soient marqués "OK" dans le terminal)


## chemin des scripts/launch utilisés :
~/catkin_ws/src/universal_robot/carnicero

## mail du goat, du "broyeur de pins", "foudroyeur d'ur30" :
titouan.briancon@sigma-clermont.fr

## mail du "démagnétiseur", "froisseur de Bota" :
matteo.proverbio@sigma-clermont.fr
