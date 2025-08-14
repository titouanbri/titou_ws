#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from ctypes import CDLL, c_void_p, c_int, POINTER, c_uint8
from ctypes.util import find_library

def main():
    rospy.init_node('ethercat_sensor_node_py')
    pub = rospy.Publisher('capteur/valeur', Float32, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    # Chargement de la bibliothèque utilisateur IgH EtherCAT
    libname = find_library('ecrt')
    if not libname:
        raise OSError("Impossible de trouver la bibliothèque libecrt")
    lib = CDLL(libname)
    # Déclarations des prototypes des fonctions que nous utilisons
    lib.ecrt_request_master.argtypes = [c_int]
    lib.ecrt_request_master.restype  = c_void_p
    lib.ecrt_master_create_domain.argtypes = [c_void_p]
    lib.ecrt_master_create_domain.restype  = c_void_p
    lib.ecrt_master_activate.argtypes = [c_void_p]
    lib.ecrt_master_activate.restype  = c_int
    lib.ecrt_master_receive.argtypes = [c_void_p]
    lib.ecrt_master_receive.restype  = c_int
    lib.ecrt_domain_process.argtypes = [c_void_p]
    lib.ecrt_domain_process.restype  = c_int
    lib.ecrt_domain_data.argtypes = [c_void_p]
    lib.ecrt_domain_data.restype  = POINTER(c_uint8)
    lib.ecrt_master_send.argtypes = [c_void_p]
    lib.ecrt_master_send.restype  = c_int

    # 1. Initialisation EtherCAT (maître + domaine)
    master = lib.ecrt_request_master(0)
    domain = lib.ecrt_master_create_domain(master)
    # → Inscrire ici vos PDOs d’entrée (vendor-id, product-code, offset, taille…) :contentReference[oaicite:4]{index=4}

    # 2. Activation (création de la zone mémoire projetée) :contentReference[oaicite:5]{index=5}
    if lib.ecrt_master_activate(master) < 0:
        rospy.logfatal("Impossible d’activer le maître EtherCAT")
        return

    # Offset du PDO de votre capteur dans l’image (en octets)
    offset = 0  # <– à ajuster selon votre mapping

    # 3. Boucle cyclique de lecture et publication ROS
    while not rospy.is_shutdown():
        lib.ecrt_master_receive(master)     # lecture trames entrantes
        lib.ecrt_domain_process(domain)     # mise à jour mémoire projetée

        pd = lib.ecrt_domain_data(domain)   # pointeur vers le début de l’image
        # Lecture d’un entier 16 bits little-endian à l’offset
        raw = pd[offset] | (pd[offset+1] << 8)
        # Conversion en signed int16
        if raw & 0x8000:
            raw -= 0x10000

        # Conversion selon calibrage (ex. -> unité physique)
        value = float(raw) * 0.001

        # Publication sur le topic ROS
        msg = Float32(data=value)
        pub.publish(msg)

        lib.ecrt_master_send(master)        # envoi trames sortantes
        rate.sleep()

if __name__ == '__main__':
    main()
