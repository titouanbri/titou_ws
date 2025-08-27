#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 node: OptoForce/OnRobot F/T via EtherCAT (pysoem)
→ Publie les valeurs F/T ET des diagnostics détaillés + log CSV optionnel.

Objectif: aider au debug quand le capteur s'arrête après 2–3 s.

Principes clefs:
- Toutes les opérations pysoem (send/receive, états, écritures PDO) restent dans
  un UNIQUE thread "bus" (worker). Le thread ROS lit des snapshots protégés par verrou
  et publie sur /sensor_data et /sensor_diagnostics (diagnostic_msgs).
- Keep-alive CMD_START optionnel à chaque cycle.
- Reconnexion automatique si sortie d'OP; dump d'état et comptage de dropouts.
- Journalisation périodique (diagnostics) + CSV optionnel avec wkc/état/longueur/temps.

Topics:
- /sensor_data                (std_msgs/Float32MultiArray)
- /sensor_diagnostics         (diagnostic_msgs/DiagnosticArray) — paramétrable

Params ROS (remap via _param:=value):
- interface (str)                  : nom de l'interface EtherCAT (ex: 'enx00133bb25956')
- slave_position (int)             : index de l'esclave dans la chaîne (def: 0)
- byte_offset (int)                : offset de départ dans le PDO in (def: 2)
- num_values (int)                 : nb de valeurs à décoder (def: 6)
- data_type ('int16'|'float32')    : format de données (def: 'int16')
- scale (float)                    : facteur d'échelle (def: 1.0)
- topic_name (str)                 : topic des données (def: '/sensor_data')
- publish_rate (float)             : Hz publication ROS (def: 250.0)
- ecat_rate (float)                : Hz thread bus (def: 1000.0)
- keepalive_start (bool)           : renvoyer CMD_START chaque cycle (def: True)
- use_config_cmds (bool)           : envoyer filtre/période/bias à l'init (def: False)
- filter_level (int)               : 0..6 pour filtre numérique (def: 2)
- period_ms (int)                  : période capteur en ms (def: 4 → ~250 Hz)
- apply_bias (bool)                : appliquer un bias (tare) à l'init (def: False)
- diagnostics_topic (str)          : nom du topic diag (def: '/sensor_diagnostics')
- diag_hz (float)                  : fréquence des diagnostics (def: 2.0 → toutes 0.5s)
- log_csv_path (str)               : chemin CSV ('' pour désactiver)
- csv_flush_every (int)            : flush disque toutes N lignes (def: 1)
- raw_hex_bytes (int)              : nb d'octets d'input à dumper lors d'un dropout (def: 16)

CSV: timestamp_iso, monotonic, wkc, op_state_hex, al_status_hex, input_len, loop_dt_ms,
     overrun, dropouts, note, raw_hex (sur dropout)
"""

import os
import time
import csv
import struct
import threading
import rospy
import pysoem
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ---- États (compat pysoem) ----
INIT_STATE     = getattr(pysoem, "INIT_STATE",     0x01)
PRE_OP_STATE   = getattr(pysoem, "PRE_OP_STATE",   0x02)
SAFE_OP_STATE  = getattr(pysoem, "SAFE_OP_STATE",  getattr(pysoem, "SAFEOP_STATE", 0x04))
OP_STATE       = getattr(pysoem, "OP_STATE",       0x08)

# ---- Commandes capteur ----
CMD_STOP            = 0x0000
CMD_START           = 0x0002
CMD_SET_BIAS        = 0x0042
CMD_SET_FILTER      = 0x0081
CMD_SET_READ_PERIOD = 0x0082


def main():
    rospy.init_node('sensor_publisher', anonymous=False)

    # ----- Paramètres ROS -----
    iface           = rospy.get_param('~interface', 'enx00133bb25956')
    slave_pos       = rospy.get_param('~slave_position', 0)
    byte_offset     = rospy.get_param('~byte_offset', 2)
    num_values      = rospy.get_param('~num_values', 6)
    data_type       = rospy.get_param('~data_type', 'int16')
    scale           = float(rospy.get_param('~scale', 1.0))
    topic_name      = rospy.get_param('~topic_name', '/sensor_data')
    publish_hz      = float(rospy.get_param('~publish_rate', 250.0))
    ecat_hz         = float(rospy.get_param('~ecat_rate', 1000.0))
    use_config_cmds = bool(rospy.get_param('~use_config_cmds', False))
    filter_level    = int(rospy.get_param('~filter_level', 2))
    period_ms       = int(rospy.get_param('~period_ms', 4))
    apply_bias      = bool(rospy.get_param('~apply_bias', False))
    keepalive_start = bool(rospy.get_param('~keepalive_start', True))

    diagnostics_topic = rospy.get_param('~diagnostics_topic', '/sensor_diagnostics')
    diag_hz           = float(rospy.get_param('~diag_hz', 2.0))
    log_csv_path      = rospy.get_param('~log_csv_path', '')
    csv_flush_every   = int(rospy.get_param('~csv_flush_every', 1))
    raw_hex_bytes     = int(rospy.get_param('~raw_hex_bytes', 16))

    # ----- Publisher ROS -----
    data_pub = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)
    diag_pub = rospy.Publisher(diagnostics_topic, DiagnosticArray, queue_size=10)

    # ----- Décodage -----
    if data_type == 'float32':
        step, fmt = 4, '<f'
    elif data_type == 'int16':
        step, fmt = 2, '<h'
    else:
        rospy.logerr("Unsupported data_type '%s'", data_type)
        return

    # ----- Master -----
    master = pysoem.Master()
    rospy.loginfo("Opening EtherCAT interface '%s'", iface)
    master.open(iface)

    if master.config_init() <= 0:
        rospy.logerr("No EtherCAT slaves found!")
        return

    # Rester en PRE-OP pour mapping
    master.state = PRE_OP_STATE
    master.write_state()
    master.state_check(PRE_OP_STATE, 2000)

    master.config_map()

    # Logs mapping
    for idx, s in enumerate(master.slaves):
        rospy.loginfo("Slave %d: name='%s', input PDO=%dB, output PDO=%dB",
                      idx, getattr(s, 'name', 'N/A'), len(s.input), len(s.output))

    try:
        slave = master.slaves[slave_pos]
    except Exception:
        rospy.logerr("Slave position %d out of range", slave_pos)
        return

    # ----- CSV (optionnel) -----
    csv_fp = None
    csv_writer = None
    csv_count_since_flush = 0
    if log_csv_path:
        try:
            csv_fp = open(log_csv_path, 'w', newline='')
            csv_writer = csv.writer(csv_fp)
            csv_writer.writerow([
                'timestamp_iso','monotonic','wkc','op_state_hex','al_status_hex','input_len',
                'loop_dt_ms','overrun','dropouts','note','raw_hex'
            ])
            rospy.loginfo("CSV logging enabled → %s", log_csv_path)
        except Exception as e:
            rospy.logwarn("Cannot open CSV '%s': %s", log_csv_path, e)
            csv_fp = None
            csv_writer = None

    # --- fonctions locales (UNIQUEMENT worker) ---
    def write_output_cmd(cmd, data=0):
        payload = struct.pack('<HH', cmd & 0xFFFF, data & 0xFFFF)
        if len(slave.output) > 4:
            payload += bytes(len(slave.output) - 4)
        slave.output = payload

    def reach_operational():
        # SAFE_OP
        master.state = SAFE_OP_STATE
        master.write_state()
        if master.state_check(SAFE_OP_STATE, 2000) != SAFE_OP_STATE:
            return False
        # sorties à zéro
        for s in master.slaves:
            if len(s.output):
                s.output = bytes(len(s.output))
        # un cycle
        master.send_processdata()
        master.receive_processdata(1000)
        # OP
        master.state = OP_STATE
        master.write_state()
        for _ in range(200):
            master.send_processdata()
            master.receive_processdata(1000)
            if master.state_check(OP_STATE, 0) == OP_STATE:
                return True
        return False

    # ----- Passer en OP -----
    if not reach_operational():
        rospy.logerr("Failed to reach OPERATIONAL at startup")
        return
    rospy.loginfo("Slaves in OPERATIONAL state, starting data loop…")

    # ----- Config capteur (optionnelle) -----
    if use_config_cmds:
        write_output_cmd(CMD_SET_FILTER,      filter_level)
        master.send_processdata(); master.receive_processdata(1000)
        write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
        master.send_processdata(); master.receive_processdata(1000)
        if apply_bias:
            write_output_cmd(CMD_SET_BIAS, 255)
            master.send_processdata(); master.receive_processdata(1000)

    # START une fois (et éventuellement en keep-alive)
    write_output_cmd(CMD_START, 0)
    master.send_processdata(); master.receive_processdata(1000)

    # --------- Partagé entre threads ---------
    last_input = b""
    ilock = threading.Lock()  # protège last_input et diag

    # stats/diag partagés
    diag = {
        'wkc': 0,
        'op_state': 0,
        'al_status': 0,
        'input_len': 0,
        'loop_dt_ms': 0.0,
        'overrun': False,
        'dropouts': 0,
        'note': '',
        'slave_name': getattr(slave, 'name', 'N/A'),
    }

    stop_evt = threading.Event()

    # Optionnel : tenter d'élever la priorité (nécessite sudo)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
    except Exception:
        pass

    # diag timing
    diag_period = 1.0 / max(diag_hz, 0.01)

    # CSV helper
    def csv_write_row(note='', raw_hex=''):
        nonlocal csv_count_since_flush
        if not csv_writer:
            return
        now_iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())
        csv_writer.writerow([
            now_iso,
            f"{time.monotonic():.6f}",
            diag['wkc'],
            f"0x{int(diag['op_state']) & 0xFF:02X}",
            f"0x{int(diag['al_status']) & 0xFFFF:04X}",
            diag['input_len'],
            f"{diag['loop_dt_ms']:.3f}",
            int(diag['overrun']),
            diag['dropouts'],
            note,
            raw_hex,
        ])
        csv_count_since_flush += 1
        if csv_count_since_flush >= max(csv_flush_every, 1):
            try:
                csv_fp.flush()
                os.fsync(csv_fp.fileno())
            except Exception:
                pass
            csv_count_since_flush = 0

    def bus_worker():
        nonlocal last_input, slave
        period = 1.0 / float(ecat_hz)
        next_t = time.monotonic()
        next_diag_t = next_t
        dropouts = 0
        while not stop_evt.is_set():
            try:
                # keep-alive sur PDO sortie
                if keepalive_start and len(slave.output) >= 4:
                    payload = struct.pack('<HH', CMD_START, 0)
                    if len(slave.output) > 4:
                        payload += bytes(len(slave.output) - 4)
                    slave.output = payload

                t0 = time.monotonic()
                master.send_processdata()
                wkc = master.receive_processdata(1000)  # ~1 ms timeout
                t1 = time.monotonic()

                # snapshot des entrées
                with ilock:
                    last_input = bytes(slave.input)
                    diag['wkc'] = int(wkc)
                    diag['input_len'] = len(last_input)
                    diag['loop_dt_ms'] = (t1 - t0) * 1000.0

                # recheck état rapide / reco
                if wkc <= 0 or master.state_check(OP_STATE, 0) != OP_STATE:
                    # lire états détaillés (dans worker)
                    master.read_state()
                    cur_state = getattr(master, 'state', 0)
                    cur_al = getattr(slave, 'al_status_code', 0)
                    dropouts += 1
                    raw_hex = last_input[:max(0, raw_hex_bytes)] .hex()
                    with ilock:
                        diag['op_state'] = cur_state
                        diag['al_status'] = cur_al
                        diag['dropouts'] = dropouts
                        diag['note'] = 'dropout'
                    rospy.logerr_throttle(1.0,
                        "[ECAT] Dropout #%d: wkc=%d state=0x%02X al_code=0x%04X input_len=%d",
                        dropouts, wkc, cur_state & 0xFF, cur_al & 0xFFFF, len(last_input))
                    csv_write_row(note='dropout', raw_hex=raw_hex)

                    # tentative de reco
                    if not reach_operational():
                        time.sleep(0.01)
                        continue
                    # rebind esclave et relancer START/config
                    try:
                        slave = master.slaves[slave_pos]
                    except Exception:
                        time.sleep(0.01)
                        continue
                    if use_config_cmds:
                        write_output_cmd(CMD_SET_FILTER,      filter_level)
                        master.send_processdata(); master.receive_processdata(1000)
                        write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
                        master.send_processdata(); master.receive_processdata(1000)
                        if apply_bias:
                            write_output_cmd(CMD_SET_BIAS, 255)
                            master.send_processdata(); master.receive_processdata(1000)
                    write_output_cmd(CMD_START, 0)
                    master.send_processdata(); master.receive_processdata(1000)

                # Publication diagnostics (timé par diag_hz), MAJ états détaillés
                now = time.monotonic()
                if now >= next_diag_t:
                    next_diag_t += diag_period
                    master.read_state()
                    cur_state = getattr(master, 'state', 0)
                    cur_al = getattr(slave, 'al_status_code', 0)
                    with ilock:
                        diag['op_state'] = cur_state
                        diag['al_status'] = cur_al
                        diag['note'] = ''
                    csv_write_row()

            except Exception as e:
                rospy.logdebug_throttle(1.0, "[ECAT] Exception dans worker: %s", e)
                time.sleep(0.001)

            # timing périodique du worker
            next_t += period
            dt = next_t - time.monotonic()
            overrun = False
            if dt > 0:
                time.sleep(dt)
            else:
                # on a raté l'échéance
                overrun = True
                next_t = time.monotonic()
            with ilock:
                diag['overrun'] = overrun

    t = threading.Thread(target=bus_worker, daemon=True)
    t.start()

    # ----- Boucle ROS (publication données + diagnostics) -----
    rate = rospy.Rate(publish_hz)
    try:
        while not rospy.is_shutdown():
            with ilock:
                data = last_input
                d = dict(diag)  # shallow copy
            # Publier données
            if data:
                values = []
                end = byte_offset + num_values * step
                for i in range(byte_offset, end, step):
                    if i + step <= len(data):
                        v = struct.unpack(fmt, data[i:i+step])[0]
                        values.append(float(v) * scale)
                    else:
                        break
                if values:
                    data_pub.publish(Float32MultiArray(data=values))
            # Publier diagnostics
            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = rospy.Time.now()
            st = DiagnosticStatus()
            st.name = f"ethercat_sensor/{d.get('slave_name','N/A')}"
            st.hardware_id = d.get('slave_name','N/A')
            # Niveau
            if d['wkc'] <= 0:
                st.level = DiagnosticStatus.ERROR
                st.message = "PDO exchange failed (WKC<=0)"
            elif (int(d['op_state']) & OP_STATE) != OP_STATE:
                st.level = DiagnosticStatus.WARN
                st.message = "Not in OPERATIONAL"
            else:
                st.level = DiagnosticStatus.OK
                st.message = "OPERATIONAL"
            # KVs
            kv = st.values
            kv.append(KeyValue(key='wkc', value=str(d['wkc'])))
            kv.append(KeyValue(key='op_state_hex', value=f"0x{int(d['op_state']) & 0xFF:02X}"))
            kv.append(KeyValue(key='al_status_hex', value=f"0x{int(d['al_status']) & 0xFFFF:04X}"))
            kv.append(KeyValue(key='input_len', value=str(d['input_len'])))
            kv.append(KeyValue(key='loop_dt_ms', value=f"{float(d['loop_dt_ms']):.3f}"))
            kv.append(KeyValue(key='worker_overrun', value=str(bool(d['overrun']))))
            kv.append(KeyValue(key='dropouts', value=str(d['dropouts'])))
            if d.get('note'):
                kv.append(KeyValue(key='note', value=str(d['note'])))
            diag_msg.status.append(st)
            diag_pub.publish(diag_msg)

            rate.sleep()
    finally:
        stop_evt.set()
        t.join(timeout=1.0)
        if csv_fp:
            try:
                csv_fp.flush(); os.fsync(csv_fp.fileno())
                csv_fp.close()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass









# #!/usr/bin/env python
# # -*- coding: utf-8 -*-
# """
# OptoForce/OnRobot F/T via EtherCAT (pysoem) -> /sensor_data (Float32MultiArray)

# - Thread "bus" dédié (par défaut 1000 Hz) qui fait send/receive en continu + keep-alive CMD_START.
# - Boucle ROS séparée qui ne fait que publier les dernières données décodées.
# - Séquence PRE_OP -> SAFE_OP -> OP robuste; Free-Run déjà actif (1C32:01=0, 1C33:01=0).
# - Reco auto si drop (tentative de repasser OP).

# Attention : pysoem n'est pas garanti thread-safe sur *plusieurs* threads → ici, TOUTES les
# opérations master/slave (send/receive/écritures PDO/états) restent dans UN SEUL thread (worker).
# Le thread ROS lit juste un snapshot bytes() protégé par un verrou.
# """

# import os
# import time
# import struct
# import threading
# import rospy
# import pysoem
# from std_msgs.msg import Float32MultiArray

# # Etats (compat)
# INIT_STATE     = getattr(pysoem, "INIT_STATE",     0x01)
# PRE_OP_STATE   = getattr(pysoem, "PRE_OP_STATE",   0x02)
# SAFE_OP_STATE  = getattr(pysoem, "SAFE_OP_STATE",  getattr(pysoem, "SAFEOP_STATE", 0x04))
# OP_STATE       = getattr(pysoem, "OP_STATE",       0x08)

# # Commandes
# CMD_STOP            = 0x0000
# CMD_START           = 0x0002
# CMD_SET_BIAS        = 0x0042
# CMD_SET_FILTER      = 0x0081
# CMD_SET_READ_PERIOD = 0x0082


# def main():
#     rospy.init_node('sensor_publisher', anonymous=False)

#     # ----- Paramètres ROS -----
#     iface          = rospy.get_param('~interface', 'enx00133bb25956')
#     slave_pos      = rospy.get_param('~slave_position', 0)
#     byte_offset    = rospy.get_param('~byte_offset', 2)        # sauter le mot statut (2B)
#     num_values     = rospy.get_param('~num_values', 6)
#     data_type      = rospy.get_param('~data_type', 'int16')    # 'int16' ou 'float32'
#     scale          = rospy.get_param('~scale', 1.0)
#     topic_name     = rospy.get_param('~topic_name', '/sensor_data')
#     publish_hz     = rospy.get_param('~publish_rate', 250.0)   # fréquence de publication ROS
#     ecat_hz        = rospy.get_param('~ecat_rate', 1000.0)     # fréquence du thread bus
#     # commandes optionnelles à l'init
#     use_config_cmds = bool(rospy.get_param('~use_config_cmds', False))
#     filter_level    = int(rospy.get_param('~filter_level', 2))
#     period_ms       = int(rospy.get_param('~period_ms', 4))
#     apply_bias      = bool(rospy.get_param('~apply_bias', False))
#     keepalive_start = bool(rospy.get_param('~keepalive_start', True))   # ON par défaut

#     pub = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)

#     # ----- Décodage -----
#     if data_type == 'float32':
#         step, fmt = 4, '<f'
#     elif data_type == 'int16':
#         step, fmt = 2, '<h'
#     else:
#         rospy.logerr("Unsupported data_type '%s'", data_type)
#         return

#     # ----- Master -----
#     master = pysoem.Master()
#     rospy.loginfo("Opening EtherCAT interface '%s'", iface)
#     master.open(iface)

#     if master.config_init() <= 0:
#         rospy.logerr("No EtherCAT slaves found!")
#         return

#     # Rester en PRE-OP pour mapper
#     master.state = PRE_OP_STATE
#     master.write_state()
#     master.state_check(PRE_OP_STATE, 2000)

#     master.config_map()

#     # Logs mapping
#     for idx, s in enumerate(master.slaves):
#         rospy.loginfo("Slave %d: name='%s', input PDO=%dB, output PDO=%dB",
#                       idx, getattr(s, 'name', 'N/A'), len(s.input), len(s.output))

#     try:
#         slave = master.slaves[slave_pos]
#     except Exception:
#         rospy.logerr("Slave position %d out of range", slave_pos)
#         return

#     # --- fonctions locales (UNIQUEMENT utilisées par le worker) ---
#     def write_output_cmd(cmd, data=0):
#         payload = struct.pack('<HH', cmd & 0xFFFF, data & 0xFFFF)
#         if len(slave.output) > 4:
#             payload += bytes(len(slave.output) - 4)
#         slave.output = payload

#     def reach_operational():
#         # SAFE_OP
#         master.state = SAFE_OP_STATE
#         master.write_state()
#         if master.state_check(SAFE_OP_STATE, 2000) != SAFE_OP_STATE:
#             return False
#         # sorties à zéro
#         for s in master.slaves:
#             if len(s.output):
#                 s.output = bytes(len(s.output))
#         # un cycle
#         master.send_processdata()
#         master.receive_processdata(1000)
#         # OP
#         master.state = OP_STATE
#         master.write_state()
#         for _ in range(200):
#             master.send_processdata()
#             master.receive_processdata(1000)
#             if master.state_check(OP_STATE, 0) == OP_STATE:
#                 return True
#         return False

#     # ----- Passer en OP -----
#     if not reach_operational():
#         rospy.logerr("Failed to reach OPERATIONAL at startup")
#         return
#     rospy.loginfo("Slaves in OPERATIONAL state, starting data loop...")

#     # ----- Config capteur (optionnelle) -----
#     if use_config_cmds:
#         write_output_cmd(CMD_SET_FILTER,      filter_level)
#         master.send_processdata(); master.receive_processdata(1000)
#         write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
#         master.send_processdata(); master.receive_processdata(1000)
#         if apply_bias:
#             write_output_cmd(CMD_SET_BIAS, 255)
#             master.send_processdata(); master.receive_processdata(1000)

#     # START une fois (et éventuellement en keep-alive)
#     write_output_cmd(CMD_START, 0)
#     master.send_processdata(); master.receive_processdata(1000)

#     # --------- Worker EtherCAT ---------
#     last_input = b""
#     ilock = threading.Lock()
#     stop_evt = threading.Event()

#     # Optionnel : tenter d'élever la priorité (nécessite sudo)
#     try:
#         os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
#     except Exception:
#         pass

#     def bus_worker():
#         nonlocal last_input, slave
#         period = 1.0 / float(ecat_hz)
#         next_t = time.monotonic()
#         while not stop_evt.is_set():
#             try:
#                 # keep-alive sur PDO sortie
#                 if keepalive_start and len(slave.output) >= 4:
#                     payload = struct.pack('<HH', CMD_START, 0)
#                     if len(slave.output) > 4:
#                         payload += bytes(len(slave.output) - 4)
#                     slave.output = payload

#                 master.send_processdata()
#                 wkc = master.receive_processdata(1000)  # ~1 ms timeout

#                 # snapshot des entrées
#                 with ilock:
#                     last_input = bytes(slave.input)

#                 # recheck état rapide
#                 if wkc <= 0 or master.state_check(OP_STATE, 0) != OP_STATE:
#                     # petite tentative de reco
#                     if not reach_operational():
#                         time.sleep(0.01)
#                         continue
#                     # rebind slave et relancer START/config
#                     try:
#                         slave = master.slaves[slave_pos]
#                     except Exception:
#                         time.sleep(0.01)
#                         continue
#                     if use_config_cmds:
#                         write_output_cmd(CMD_SET_FILTER,      filter_level)
#                         master.send_processdata(); master.receive_processdata(1000)
#                         write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
#                         master.send_processdata(); master.receive_processdata(1000)
#                         if apply_bias:
#                             write_output_cmd(CMD_SET_BIAS, 255)
#                             master.send_processdata(); master.receive_processdata(1000)
#                     write_output_cmd(CMD_START, 0)
#                     master.send_processdata(); master.receive_processdata(1000)

#             except Exception:
#                 # on ne spam pas les logs ici ; la boucle ROS remontera les états si besoin
#                 time.sleep(0.001)

#             # timing périodique
#             next_t += period
#             dt = next_t - time.monotonic()
#             if dt > 0:
#                 time.sleep(dt)
#             else:
#                 next_t = time.monotonic()

#     t = threading.Thread(target=bus_worker, daemon=True)
#     t.start()

#     # ----- Boucle ROS (lecture + publication) -----
#     rate = rospy.Rate(publish_hz)
#     try:
#         while not rospy.is_shutdown():
#             with ilock:
#                 data = last_input
#             if data:
#                 values = []
#                 end = byte_offset + num_values * step
#                 for i in range(byte_offset, end, step):
#                     if i + step <= len(data):
#                         v = struct.unpack(fmt, data[i:i+step])[0]
#                         values.append(float(v) * float(scale))
#                     else:
#                         break
#                 if values:
#                     pub.publish(Float32MultiArray(data=values))
#             rate.sleep()
#     finally:
#         stop_evt.set()
#         t.join(timeout=1.0)


# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


























# # """
# # ROS1 node : Lecture capteur F/T OnRobot/OptoForce via EtherCAT (pysoem)
# # et publication sur /sensor_data (std_msgs/Float32MultiArray).

# # - Séquence PRE_OP -> SAFE_OP -> OP robuste (watchdog sorties).
# # - PDO sortie = 4 octets : <cmd(uint16), data(uint16)> little-endian.
# # - Keep-alive : renvoi de CMD_START à chaque cycle (désactivable).
# # - Reconnexion auto si l'esclave sort d'OP.
# # """

# # import struct
# # import rospy
# # import pysoem
# # from std_msgs.msg import Float32MultiArray

# # # ---- Etats pysoem (compat) ----
# # INIT_STATE     = getattr(pysoem, "INIT_STATE",     0x01)
# # PRE_OP_STATE   = getattr(pysoem, "PRE_OP_STATE",   0x02)
# # SAFE_OP_STATE  = getattr(pysoem, "SAFE_OP_STATE",  getattr(pysoem, "SAFEOP_STATE", 0x04))
# # OP_STATE       = getattr(pysoem, "OP_STATE",       0x08)

# # # ---- Commandes OnRobot/OptoForce ----
# # CMD_STOP            = 0x0000
# # CMD_START           = 0x0002
# # CMD_SET_BIAS        = 0x0042  # 255=on, 0=off
# # CMD_SET_FILTER      = 0x0081  # 0..6
# # CMD_SET_READ_PERIOD = 0x0082  # période en ms

# # def log_slave_map(master):
# #     for idx, s in enumerate(master.slaves):
# #         rospy.loginfo("Slave %d: name='%s', input PDO length=%d, output PDO length=%d",
# #                       idx, getattr(s, 'name', 'N/A'), len(s.input), len(s.output))

# # def dump_status(master):
# #     master.read_state()
# #     for i, s in enumerate(master.slaves):
# #         rospy.logerr("Slave %d state=0x%02x al_status=0x%04x name=%s",
# #                      i, getattr(s, 'state', 0), getattr(s, 'al_status', 0), getattr(s, 'name', ''))

# # def reach_operational(master):
# #     # SAFE_OP
# #     master.state = SAFE_OP_STATE
# #     master.write_state()
# #     if master.state_check(SAFE_OP_STATE, 5000) != SAFE_OP_STATE:
# #         rospy.logerr("Impossible d'atteindre SAFE_OP")
# #         dump_status(master)
# #         return False
# #     # Pré-remplir sorties à zéro
# #     for s in master.slaves:
# #         if len(s.output):
# #             s.output = bytes(len(s.output))
# #     # Un cycle PDO
# #     master.send_processdata()
# #     master.receive_processdata(1000)
# #     # OP + quelques cycles
# #     master.state = OP_STATE
# #     master.write_state()
# #     for _ in range(100):
# #         master.send_processdata()
# #         master.receive_processdata(1000)
# #         if master.state_check(OP_STATE, 0) == OP_STATE:
# #             return True
# #     rospy.logerr("Failed to reach OPERATIONAL")
# #     dump_status(master)
# #     return False

# # def main():
# #     rospy.init_node('sensor_publisher', anonymous=False)

# #     # ---- Params ----
# #     iface         = rospy.get_param('~interface', 'enx00133bb25956')
# #     slave_pos     = rospy.get_param('~slave_position', 0)
# #     byte_offset   = rospy.get_param('~byte_offset', 2)       # sauter 1 mot de status
# #     num_values    = rospy.get_param('~num_values', 6)
# #     data_type     = rospy.get_param('~data_type', 'int16')   # 'int16' ou 'float32'
# #     scale         = rospy.get_param('~scale', 1.0)
# #     topic_name    = rospy.get_param('~topic_name', '/sensor_data')
# #     rate_hz       = rospy.get_param('~publish_rate', 500.0)

# #     # commandes optionnelles
# #     keepalive_start = bool(rospy.get_param('~keepalive_start', True))   # renvoyer CMD_START à chaque cycle
# #     use_config_cmds = bool(rospy.get_param('~use_config_cmds', False))  # envoyer filtre/période une fois
# #     filter_level    = int(rospy.get_param('~filter_level', 2))          # 0..6
# #     period_ms       = int(rospy.get_param('~period_ms', 2))             # 2 -> ~500 Hz
# #     apply_bias      = bool(rospy.get_param('~apply_bias', False))

# #     pub  = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)
# #     rate = rospy.Rate(rate_hz)

# #     # ---- Master ----
# #     master = pysoem.Master()
# #     rospy.loginfo("Opening EtherCAT interface '%s'", iface)
# #     master.open(iface)

# #     if master.config_init() <= 0:
# #         rospy.logerr("No EtherCAT slaves found!")
# #         return
# #     master.config_map()
# #     log_slave_map(master)

# #     if not reach_operational(master):
# #         return
# #     rospy.loginfo("Slaves in OPERATIONAL state, starting data loop...")

# #     # ---- Esclave ciblé ----
# #     try:
# #         slave = master.slaves[slave_pos]
# #     except Exception:
# #         rospy.logerr("Slave position %d out of range", slave_pos)
# #         return

# #     # ---- Helper sortie <HH> ----
# #     def write_output_cmd(cmd, data=0):
# #         """Écrit tout le PDO sortie en une fois (bytes immuable)."""
# #         payload = struct.pack('<HH', cmd & 0xFFFF, data & 0xFFFF)
# #         # si le PDO sortie est > 4, complèter avec des zéros
# #         if len(slave.output) > 4:
# #             payload += bytes(len(slave.output) - 4)
# #         slave.output = payload

# #         master.send_processdata()
# #         master.receive_processdata(1000)

# #     # ---- Config optionnelle (envoyée une fois) ----
# #     if use_config_cmds:
# #         write_output_cmd(CMD_SET_FILTER,      filter_level)
# #         write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
# #         if apply_bias:
# #             write_output_cmd(CMD_SET_BIAS, 255)

# #     # ---- Décodage ----
# #     if data_type == 'float32':
# #         step, fmt = 4, '<f'
# #     elif data_type == 'int16':
# #         step, fmt = 2, '<h'
# #     else:
# #         rospy.logerr("Unsupported data_type '%s' (use 'int16' or 'float32')", data_type)
# #         return

# #     prev_hex = None

# #     # ---- Boucle ----
# #     while not rospy.is_shutdown():
# #         # Keep-alive : RÉ-ASSIGNER tout le buffer (pas d'indexation sur bytes)
# #         if keepalive_start and len(slave.output) >= 4:
# #             payload = struct.pack('<HH', CMD_START, 0)
# #             if len(slave.output) > 4:
# #                 payload += bytes(len(slave.output) - 4)
# #             slave.output = payload

# #         master.send_processdata()
# #         wkc = master.receive_processdata(1000)

# #         # Vérifier OP et tenter une reco si besoin
# #         if wkc == 0 or master.state_check(OP_STATE, 0) != OP_STATE:
# #             rospy.logerr_throttle(1.0, "Slave dropped out of OP (wkc=%d) -> tentative de reconnexion", wkc)
# #             dump_status(master)
# #             if not reach_operational(master):
# #                 rate.sleep()
# #                 continue
# #             # re-récupérer l'objet esclave (sécurité) et re-pousser config
# #             try:
# #                 slave = master.slaves[slave_pos]
# #             except Exception:
# #                 rospy.logerr("Slave position %d out of range after reconnect", slave_pos)
# #                 rate.sleep()
# #                 continue
# #             if use_config_cmds:
# #                 write_output_cmd(CMD_SET_FILTER,      filter_level)
# #                 write_output_cmd(CMD_SET_READ_PERIOD, period_ms)
# #                 if apply_bias:
# #                     write_output_cmd(CMD_SET_BIAS, 255)

# #         try:
# #             data = slave.input
# #             raw_hex = data.hex()
# #             if raw_hex != prev_hex:
# #                 rospy.logdebug("PDO raw: %s", raw_hex)
# #                 prev_hex = raw_hex

# #             values = []
# #             end = byte_offset + num_values * step
# #             for i in range(byte_offset, end, step):
# #                 if i + step <= len(data):
# #                     v = struct.unpack(fmt, bytes(data[i:i+step]))[0]
# #                     values.append(float(v) * float(scale))
# #                 else:
# #                     rospy.logwarn("Index %d out of range for PDO length %d", i, len(data))
# #                     break

# #             pub.publish(Float32MultiArray(data=values))
# #         except Exception as e:
# #             rospy.logwarn_throttle(1.0, "Error during read/publish: %s", e)

# #         rate.sleep()

# # if __name__ == '__main__':
# #     try:
# #         main()
# #     except rospy.ROSInterruptException:
# #         pass





# # """
# # ROS1 Node: Read EtherCAT sensor data and publish to a ROS topic.

# # This script uses the pysoem library to interface with an EtherCAT bus, reads input data from a specified slave and maps it to a ROS message.
# # Includes debug logs to inspect slave mapping and raw PDO bytes.

# # Usage:
# #   Place in your package's scripts/, make executable, then:
# #     rosrun <your_package> sensor_to_ros.py __name:=sensor_publisher

# # Parameters (rosparam or remapped via command line):
# #   interface     : EtherCAT network interface (string, default: "eth0")
# #   slave_position: Position/index of EtherCAT slave in the chain (int, default: 0)
# #   byte_offset   : Starting byte offset in the input PDO (int, default: 0)
# #   num_values    : Number of data values to read (int, default: 6)
# #   data_type     : Data format, either "float32" or "int16" (string, default: "int16")
# #   scale         : Scale factor to apply to raw values (float, default: 1.0)
# #   topic_name    : ROS topic to publish (string, default: "/sensor_data")
# #   publish_rate  : Rate in Hz for reading and publishing (float, default: 10.0)

# # Topic Type:
# #   std_msgs/Float32MultiArray
# # """
# # import rospy
# # from std_msgs.msg import Float32MultiArray
# # import pysoem
# # import struct


# # def main():
# #     # Initialize ROS node
# #     rospy.init_node('sensor_publisher', anonymous=False)

# #     # ROS parameters
# #     iface         = rospy.get_param('~interface', 'enx00133bb25956')
# #     slave_pos     = rospy.get_param('~slave_position', 0)
# #     byte_offset   = rospy.get_param('~byte_offset', 0)
# #     num_values    = rospy.get_param('~num_values', 6)
# #     data_type     = rospy.get_param('~data_type', 'int16')
# #     scale         = rospy.get_param('~scale', 1.0)
# #     topic_name    = rospy.get_param('~topic_name', '/sensor_data')
# #     rate_hz       = rospy.get_param('~publish_rate', 500.0)

# #     # Publisher
# #     pub = rospy.Publisher(topic_name, Float32MultiArray, queue_size=10)
# #     rate = rospy.Rate(rate_hz)

# #     # EtherCAT master setup
# #     master = pysoem.Master()
# #     rospy.loginfo("Opening EtherCAT interface '%s'", iface)
# #     try:
# #         master.open(iface)
# #     except Exception as e:
# #         rospy.logerr("Failed to open interface %s: %s", iface, e)
# #         return

# #     # Discover and configure slaves
# #     count = master.config_init()
# #     if count <= 0:
# #         rospy.logerr("No EtherCAT slaves found!")
# #         return
# #     rospy.loginfo("%d EtherCAT slave(s) found", count)

# #     # Map PDOs
# #     master.config_map()


# #     # Debug: list slave mapping after config_map
# #     for idx, slave in enumerate(master.slaves):
# #         rospy.loginfo("Slave %d: name='%s', input PDO length=%d bytes, output PDO length=%d bytes",
# #                       idx, getattr(slave, 'name', 'N/A'), len(slave.input), len(slave.output))

# #     # Set slaves to OPERATIONAL
# #     master.state = pysoem.OP_STATE
# #     master.write_state()
# #     print(master.state_check(pysoem.OP_STATE, timeout=1000))

# #     if not master.state_check(pysoem.OP_STATE, timeout=1000):
# #         rospy.logerr("Failed to set slaves to OPERATIONAL")
# #         return
# #     rospy.loginfo("Slaves in OPERATIONAL state, starting data loop...")

# #     # — send the “Start F/T Data Output” command (0x0B) once before reading
# #     slave = master.slaves[slave_pos]
# #     slave.output = b'\x0B\x00\x00\x00'
# #     master.send_processdata()
# #     master.receive_processdata(timeout=1000)
# #     # rospy.loginfo("Sent Start‐Output command to sensor")


# #     prev_raw = None
# #     # Main loop: read and publish
# #     while not rospy.is_shutdown():

# #             # ————— on redonne à chaque cycle la commande “Start‐FT” —————
# #         # slave = master.slaves[slave_pos]
# #         # slave.output = b'\x0B\x00\x00\x00'
# #         master.send_processdata()
# #         wkc = master.receive_processdata(timeout=1000)
# #         if wkc == 0:
# #             rospy.logwarn("WKC=0, échange PDO invalide")
# #             continue
        
# #         print(master.state_check(pysoem.OP_STATE, timeout=1000))
# #         if not master.state_check(pysoem.OP_STATE, timeout=1000):
# #             code = slave.al_status
# #             rospy.logerr("Basculement en PRE-OP, al_status=0x%02x", code)

# #         try:
# #             slave = master.slaves[slave_pos]
            
# #             data = slave.input  # bytearray of input PDO

# #             # Debug: raw data hex and change detection
# #             raw_hex = data.hex()
# #             if prev_raw is None or raw_hex != prev_raw:
# #                 # rospy.loginfo("Raw PDO data (hex): %s", raw_hex)
# #                 prev_raw = raw_hex
# #             else:
# #                 rospy.logdebug("Raw PDO data unchanged")

# #             # Unpack values
# #             values = []
# #             if data_type == 'float32':
# #                 step, fmt = 4, '<f'
# #             elif data_type == 'int16':
# #                 step, fmt = 2, '<h'
# #             else:
# #                 rospy.logerr("Unsupported data_type '%s'", data_type)
# #                 break

# #             for i in range(byte_offset, byte_offset + num_values*step, step):
# #                 if i + step <= len(data):
# #                     raw = struct.unpack(fmt, bytes(data[i:i+step]))[0]
# #                     values.append(raw * scale)
# #                 else:
# #                     rospy.logwarn("Index %d out of range for PDO length %d", i, len(data))
# #                     break

# #             # Publish data
# #             msg = Float32MultiArray()
# #             msg.data = values
# #             pub.publish(msg)
# #         except IndexError:
# #             rospy.logwarn("Slave position %d out of range", slave_pos)
# #         except Exception as e:
# #             rospy.logwarn("Error during read/publish: %s", e)

# #         rate.sleep()

# # if __name__ == '__main__':
# #     try:
# #         main()
# #     except rospy.ROSInterruptException:
# #         pass
