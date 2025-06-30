import serial
import time


def send_ascii(ser, cmd):
    ser.write((cmd + '\n').encode())
    ser.flush()
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting or 100).decode(errors='ignore').strip()
    print(f"[SENT] {cmd} → [RESPONSE] {resp}")
    return resp

def force_single_offset(port="/dev/ttyUSB0"):
    print("[INFO] Initialisation du capteur sur", port)
    ser = serial.Serial(port, 460800, timeout=0.2)
    
    # Attendre l'App Init
    print("[INFO] Attente du démarrage (App Init)...")
    init = ser.read_until(b'App Init')
    print(f"[INIT] {init.decode(errors='ignore').strip()}")
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Passage mode ASCII
    ser.write(b'C')
    time.sleep(0.3)

    # Reset offset à 0
    send_ascii(ser, 'b,0,0,0,0,0,0')
    
    # Sauvegarde
    send_ascii(ser, 's')
    
    # Lecture de la config active pour vérification
    send_ascii(ser, 'w')

    print("[INFO] Débranchement physique recommandé (cold boot) pour assurer stabilité.")
    ser.close()

if __name__ == "__main__":
    force_single_offset("/dev/ttyUSB0")
