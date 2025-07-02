# UR30-admittance

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
- important 


- changer le nom du port 
- changer nom du capteur dans le launch 
## Commande finale pour lancer l'acquisition ( se lance avec setup.launch )
- roslaunch rokubimini_ethercat rokubimini_ethercat.launch
