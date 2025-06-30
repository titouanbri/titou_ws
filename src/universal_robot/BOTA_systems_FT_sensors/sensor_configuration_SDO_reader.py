import struct
import time
from collections import namedtuple
import pysoem
import sys
import ctypes

class Sensor_Configuration_SDO_Reader:

    BOTA_VENDOR_ID = 0xB07A
    BOTA_PRODUCT_CODE = 0x00000001
    BOTA_SENSOR_NAME = 'BFT-SENS-ECAT-M8'

    def __init__(self, ifname):
        self._ifname = ifname

        # Initialize variables
        self.sensor_input_as_bytes = None
        self.time_step = 1.0
        self.receive_timeout_us = 2000

        # Initialise EtherCAT master and slaves
        self._master = pysoem.Master()
        SlaveSet = namedtuple('SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_layout = {0: SlaveSet(self.BOTA_SENSOR_NAME, self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}

        try:
            self._master.open(self._ifname)
        except Exception as e:
            print(f"Failed to open EtherCAT interface '{self._ifname}'.")
            raise

        if not self._master.config_init() > 0:
            print('No EtherCAT slave found.')
            self._master.close()
            raise

        for i, slave in enumerate(self._master.slaves):
            if not ((slave.man == self.BOTA_VENDOR_ID) and (slave.id == self._expected_slave_layout[i].product_code)):
                print('Unexpected slave layout.')
                self._master.close()
                raise
            slave.config_func = self._expected_slave_layout[i].config_func
        self._master.config_map()

        if self._master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            print('Not all slaves reached SAFEOP state.')
            self._master.close()
            raise
        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        self._master.state_check(pysoem.OP_STATE, 50000)
        if self._master.state_check(pysoem.OP_STATE, 50000) != pysoem.OP_STATE:
            print('Not all slaves reached OP state.')
            self._master.close()
            raise

    def bota_sensor_setup(self, slave_pos):
        """Configure Bota Systems slave device."""
        slave = self._master.slaves[slave_pos]
        ## Read sensor configuration
        print("\n--- Reading Default SDO Values ---")
        sdo_map = [
            (0x2000, 0, 'Calibration'),
            (0x6000, 0, 'Sensor Data'),
            (0x7000, 0, 'Digital Output'),
            (0x8000, 0, 'Force/Torque Offset'),
            (0x8001, 0, 'Acceleration Offset'),
            (0x8002, 0, 'Angular Velocities Offset'),
            (0x8003, 0, 'Force/Torque Range'),
            (0x8004, 0, 'Acceleration Range'),
            (0x8005, 0, 'Angular Velocities Range'),
            (0x8006, 0, 'Force/Torque Filter'),
            (0x8006, 1, 'SINC Length'),
            (0x8006, 2, 'FIR disable'),
            (0x8006, 3, 'FAST enable'),
            (0x8006, 4, 'CHOP enable'),
            (0x8007, 0, 'Acceleration Filter'),
            (0x8008, 0, 'Angular Velocities Filter'),
            (0x8010, 0, 'Device configuration'),
            (0x8010, 1, 'Calibration Matrix Active'),
            (0x8010, 2, 'Temperature Compensation'),
            (0x8010, 3, 'IMU Active'),
            (0x8010, 4, 'Coordinate System Cong. Active'),
            (0x8010, 5, 'Inertia and Gravity Comp. Active'),
            (0x8010, 6, 'Orientation Estimation Active'),
            (0x8011, 0, 'Sampling Rate'),
            (0x8030, 0, 'Control'),
            (0x8030, 1, 'Command'),
            (0x8030, 2, 'Status'),
        ]
        for index, subindex, name in sdo_map:
            try:
                raw_data = slave.sdo_read(index, subindex)
                value_hex = raw_data.hex()
                print(f"0x{index:04X}, Sub {subindex:02d} - {name}: 0x{value_hex.upper()}")
            except Exception as e:
                print(f"0x{index:04X}, Sub {subindex:02d} - {name}: ERROR reading ({str(e)})")
        
        ## Set sensor configuration
        # calibration matrix active
        slave.sdo_write(0x8010, 1, bytes(ctypes.c_uint8(1)))
        # temperature compensation
        slave.sdo_write(0x8010, 2, bytes(ctypes.c_uint8(0)))
        # IMU active
        slave.sdo_write(0x8010, 3, bytes(ctypes.c_uint8(1)))
        # FIR disable
        slave.sdo_write(0x8006, 2, bytes(ctypes.c_uint8(1)))
        # FAST enable
        slave.sdo_write(0x8006, 3, bytes(ctypes.c_uint8(0)))
        # CHOP enable
        slave.sdo_write(0x8006, 4, bytes(ctypes.c_uint8(0)))
        # Sinc filter size
        slave.sdo_write(0x8006, 1, bytes(ctypes.c_uint16(64)))
        ## Get sampling rate
        sampling_rate = struct.unpack('h', slave.sdo_read(0x8011, 0))[0]
        print(f"Sensor sampling rate: {sampling_rate} Hz.")
        if sampling_rate > 0:
            self.time_step = 1.0 / float(sampling_rate)
            self.receive_timeout_us = int(self.time_step * 1e6)
            print(f"Time step calculated: {self.time_step} seconds.")
            print(f"Time timeout: {self.receive_timeout_us} us.")

    def run(self):
        """Main loop for reading and sending sensor data."""
        try:
            while 1:
                # free run cycle
                self._master.send_processdata() # Exchange process data with the sensor
                self._master.receive_processdata(self.receive_timeout_us)
                self.sensor_input_as_bytes = self._master.slaves[0].input
                status = struct.unpack_from('B', self.sensor_input_as_bytes, 0)[0]
                print("Status {}".format(status))
                warningsErrorsFatals = struct.unpack_from('I', self.sensor_input_as_bytes, 1)[0]
                print("Warnings/Errors/Fatals {}".format(warningsErrorsFatals))
                Fx = struct.unpack_from('f', self.sensor_input_as_bytes, 5)[0]
                print("Fx {}".format(Fx))
                Fy = struct.unpack_from('f', self.sensor_input_as_bytes, 9)[0]
                print("Fy {}".format(Fy))
                Fz = struct.unpack_from('f', self.sensor_input_as_bytes, 13)[0]
                print("Fz {}".format(Fz))
                Mx = struct.unpack_from('f', self.sensor_input_as_bytes, 17)[0]
                print("Mx {}".format(Mx))
                My = struct.unpack_from('f', self.sensor_input_as_bytes, 21)[0]
                print("My {}".format(My))
                Mz = struct.unpack_from('f', self.sensor_input_as_bytes, 25)[0]
                print("Mz {}".format(Mz))
                forceTorqueSaturated = struct.unpack_from('H', self.sensor_input_as_bytes, 29)[0]
                print("Force-Torque saturated {}".format(forceTorqueSaturated))
                Ax = struct.unpack_from('f', self.sensor_input_as_bytes, 31)[0]
                print("Ax {}".format(Ax))
                Ay = struct.unpack_from('f', self.sensor_input_as_bytes, 35)[0]
                print("Ay {}".format(Ay))
                Az = struct.unpack_from('f', self.sensor_input_as_bytes, 39)[0]
                print("Az {}".format(Az))
                accelerationSaturated = struct.unpack_from('B', self.sensor_input_as_bytes, 43)[0]
                print("Acceleration saturated {}".format(accelerationSaturated))
                Rx = struct.unpack_from('f', self.sensor_input_as_bytes, 44)[0]
                print("Rx {}".format(Rx))
                Ry = struct.unpack_from('f', self.sensor_input_as_bytes, 48)[0]
                print("Ry {}".format(Ry))
                Rz = struct.unpack_from('f', self.sensor_input_as_bytes, 52)[0]
                print("Rz {}".format(Rz))
                angularRateSaturated = struct.unpack_from('B', self.sensor_input_as_bytes, 56)[0]
                print("Angular rate saturated {}".format(angularRateSaturated))
                temperature = struct.unpack_from('f', self.sensor_input_as_bytes, 57)[0]
                print("Temperature {}\n".format(temperature))
                time.sleep(self.time_step)
        except KeyboardInterrupt:
            # ctrl-C abort handling
            print('stopped')
        self._master.state = pysoem.INIT_STATE
        # request INIT state for all slaves
        self._master.write_state()
        self._master.close()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <ifname>")
        sys.exit(1)
    try:
        interface = sys.argv[1]
        Sensor_Configuration_SDO_Reader(interface).run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)