import rclpy
from rclpy.node import Node
import serial

from telemetry_interfaces.msg import RcChans, Telemetry
from telemetry_interfaces.srv import SetMotors, SetRc

MSP_IDENT = 100
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110

MSP_SET_RAW_RC = 200
MSP_SET_MOTOR = 214
MSP_ACC_CALIBRATION = 205

MSP_ARMING_CONFIG = 61
MSP_SET_ARMING_DISABLED = 99

#REQUESTS = [MSP_RAW_IMU, MSP_MOTOR, MSP_RC, MSP_RAW_GPS, MSP_COMP_GPS, MSP_ATTITUDE, MSP_ALTITUDE, MSP_ANALOG]
REQUESTS = [MSP_MOTOR, MSP_RC]
COMMANDS_INT16 = [MSP_SET_MOTOR, MSP_SET_RAW_RC]

class BetaflightTelemetryNode(Node):
    def __init__(self):
        super().__init__('betaflight_telemetry')

        port = "/dev/ttyACM0"  
        baud_rate = 115200
        self.request_index = 0
        self.telemetry = {}

        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=1)
            self.get_logger().info(f"Connected to {port} at {baud_rate} baud")
        except serial.SerialException:
            self.get_logger().error(f"Failed to connect to {port}")
            return
        
        self.telemetry_publisher = self.create_publisher(Telemetry, 'betaflight_telemetry', 100)

        self.timer = self.create_timer(0.001, self.read_telemetry)

        self.set_motors_srv = self.create_service(SetMotors, 'motor_control', self.set_motors_callback)
        self.set_rc_srv = self.create_service(SetRc, 'rc_control', self.set_rc_callback) 

    def create_msp_request(self, command, request_data=None):
        # Preamble ($M)
        preamble = bytearray([0x24, 0x4D])
        # Direction ('>') to the MWC (flight controller)
        direction = bytearray([0x3C]) 

        command_byte = bytearray([command])

        data_bytes = bytearray()
        if not request_data is None:
            for value in request_data:
                data_bytes.append(value)
        #self.get_logger().info(f"Data looks like this: {data_bytes.hex()}")
        
        size = bytearray([0x00]) if request_data is None else bytearray([len(data_bytes)]) 

        # Calculate checksum (XOR of size, command, and data)
        checksum = size[0] ^ command_byte[0]
        for byte in data_bytes:
            checksum ^= byte

        crc = bytearray([checksum])

        # Message format: [preamble][direction][size][command][data][crc]
        message = preamble + direction + size + command_byte + data_bytes + crc
        self.get_logger().info("Sending request to flight controller:")
        self.get_logger().info(f"{preamble.hex()} {direction.hex()} {size.hex()} {command_byte.hex()} {data_bytes.hex()} {crc.hex()}")
        return message

    def parse_msp(self, raw_data):
        for i in range(len(raw_data) - 4):
            if raw_data[i:i+3] == b'\x24\x4D\x3E':
                size = raw_data[i+3]
                command = raw_data[i+4]
                data_index = i + 5
                return self.parse_data(size, command, raw_data, data_index)
            elif raw_data[i:i+3] == b'\x24\x4D\x21':
                command = raw_data[i+4]
                self.get_logger().warn(f"Error occured when executing command - {command}")
                return {}
        return {}
    
    def parse_data(self, size, command, raw_data, data_index):
        payload = {}
        if command == MSP_RAW_IMU:
            keys = ['ACC X', 'ACC Y', 'ACC Z', 'GYR X', 'GYR Y', 'GYR Z', 'MAG X', 'MAG Y', 'MAG Z']
            j = 0
            for i in range(data_index, size + data_index, 2):
                byte = raw_data[i:i+2]
                data_unit = int.from_bytes(byte, byteorder="little", signed=True)
                payload[keys[j]] = data_unit
                j += 1

        if command == MSP_RC:
            keys = ['ROLL', 'PITCH', 'YAW', 'THROTTLE', 'AUX1', 'AUX2', 'AUX3', 'AUX4', 'AUX5']
            j = 0
            #for i in range(data_index, size + data_index - 14, 2):
            for i in range(data_index, size + data_index - 18, 2):
                byte = raw_data[i:i+2]
                data_unit = int.from_bytes(byte, byteorder="little")
                payload[keys[j]] = data_unit
                j += 1

        if command == MSP_MOTOR:
            keys = ['Motor1', 'Motor2', 'Motor3', 'Motor4']
            j = 0
            for i in range(data_index, size + data_index - 8, 2):
                byte = raw_data[i:i+2]
                data_unit = int.from_bytes(byte, byteorder="little")
                payload[keys[j]] = data_unit
                j += 1

        if command == MSP_RAW_GPS:
            keys = ['GPS_FIX', 'Sat', 'Lat', 'Lon', 'altitude', 'speed', 'ground_course']
            j = 0
            for i in range(data_index, data_index + 2, 1):
                byte = raw_data[i]
                data_unit = byte
                payload[keys[j]] = data_unit
                j += 1
                #self.get_logger().info(f"{j}. Parsing 8 bit integers")
            for i in range(data_index + 2, data_index + 10, 4):
                byte = raw_data[i:i+4]
                data_unit = int.from_bytes(byte, byteorder="little")
                payload[keys[j]] = data_unit
                j += 1
                #self.get_logger().info(f"{j}. Parsing 32 bit integers")
            for i in range(data_index + 10, data_index + size - 2, 2):
                byte = raw_data[i:i+2]
                data_unit = int.from_bytes(byte, byteorder="little")
                payload[keys[j]] = data_unit
                j += 1
                #self.get_logger().info(f"{j}. Parsing 16 bit integers")
        
        # Write COMP GPS payload parsing when GPS is attached

        if command == MSP_ATTITUDE:
            keys = ['ANG X', 'ANG Y', 'Heading']
            j = 0
            for i in range(data_index, size + data_index, 2):
                byte = raw_data[i:i+2]
                data_unit = int.from_bytes(byte, byteorder="little", signed=True)
                payload[keys[j]] = data_unit
                j += 1

        if command == MSP_ALTITUDE:
            keys = ['EstAlt', 'vario']
            j = 0
            
            byte32 = raw_data[data_index:data_index+4]
            data_unit32 = int.from_bytes(byte32, byteorder="little", signed=True)
            payload[keys[j]] = data_unit32

            byte16 = raw_data[data_index+4:data_index+6]
            data_unit16 = int.from_bytes(byte16, byteorder="little", signed=True)
            payload[keys[j]] = data_unit16

        return payload
    
    def package_data(self, data):
        msg = Telemetry()

        msg.accx = data.get("ACC X", 0)
        msg.accy = data.get("ACC Y", 0)
        msg.accz = data.get("ACC Z", 0)
        msg.gyrx = data.get("GYR X", 0)
        msg.gyry = data.get("GYR Y", 0)
        msg.gyrz = data.get("GYR Z", 0)
        msg.magx = data.get("MAG X", 0)
        msg.magy = data.get("MAG Y", 0)
        msg.magz = data.get("MAG Z", 0)

        msg.motor1 = data.get("Motor1", 0)
        msg.motor2 = data.get("Motor2", 0)
        msg.motor3 = data.get("Motor3", 0)
        msg.motor4 = data.get("Motor4", 0)

        msg.roll = data.get("ROLL", 0)
        msg.pitch = data.get("PITCH", 0)
        msg.yaw = data.get("YAW", 0)
        msg.throttle = data.get("THROTTLE", 0)
        msg.aux1 = data.get("AUX1", 0)
        msg.aux2 = data.get("AUX2", 0)
        msg.aux3 = data.get("AUX3", 0)
        msg.aux4 = data.get("AUX4", 0)
        msg.aux5 = data.get("AUX5", 0)

        msg.gpsfix = data.get('GPS_FIX', 0)
        msg.numsat = data.get('Sat', 0)
        msg.latitude = data.get('Lat', 0)
        msg.longitude = data.get('Lon', 0)
        msg.altitude = data.get('altitude', 0)
        msg.speed = data.get('speed', 0)
        msg.groundcourse = data.get('ground_course', 0)

        # Package RAW and COMP GPS params when GPS is attached

        msg.angx = data.get("ANG X", 0)
        msg.angy = data.get("ANG Y", 0)
        msg.heading = data.get("Heading", 0)

        msg.estalt = data.get("EstAlt", 0)
        msg.vario = data.get("vario", 0)

        return msg

    def read_telemetry(self):
        """Read raw MSP data from Betaflight and print it."""
        command = REQUESTS[self.request_index]
        if command == MSP_SET_MOTOR:
            #set_data = bytearray.fromhex("dc05 dc05 dc05 b907 e803 e803 e803 e803 e803 0000 0000 0000 0000 0000 0000 0000")
            set_data = bytearray.fromhex("5704 5704 5704 5704 0000 0000 0000 0000")
            msp_request = self.create_msp_request(command, set_data)
        else:
            msp_request = self.create_msp_request(command)
        
        self.serial_port.write(msp_request)
        
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.read(self.serial_port.in_waiting)
            self.get_logger().info(f'Received raw telemetry: {data.hex()}')
            self.telemetry.update(self.parse_msp(data))

        self.request_index = (self.request_index + 1) % len(REQUESTS)

        if self.request_index == 0:
            data_to_publish = self.package_data(self.telemetry)
            self.telemetry_publisher.publish(data_to_publish)
            self.get_logger().info(f'Publishing data: {data_to_publish}')
            self.telemetry = {}

    def set_motors_callback(self, request, response):
        """Offers a service that takes motor values from the user and sets the motor values of the drone"""
        set_data = bytearray()
        set_data += request.motor1.to_bytes(2, byteorder='little', signed=False)
        set_data += request.motor2.to_bytes(2, byteorder='little', signed=False)
        set_data += request.motor3.to_bytes(2, byteorder='little', signed=False)
        set_data += request.motor4.to_bytes(2, byteorder='little', signed=False)
        set_data += bytearray.fromhex("0000 0000 0000 0000")

        self.get_logger().info(f'\033[38;5;151m Received set data: {set_data.hex()} \033[0m')

        msp_request = self.create_msp_request(MSP_SET_MOTOR, set_data)
        self.serial_port.write(msp_request)

        response.success = True
        return response

    def set_rc_callback(self, request, response):
        """Offers a service that takes RC channel values from a client node and sets the RC values of the drone"""
        set_data = bytearray()
        set_data += request.roll.to_bytes(2, byteorder='little', signed=False)
        set_data += request.pitch.to_bytes(2, byteorder='little', signed=False)
        set_data += request.throttle.to_bytes(2, byteorder='little', signed=False)
        set_data += request.yaw.to_bytes(2, byteorder='little', signed=False)
        set_data += request.aux1.to_bytes(2, byteorder='little', signed=False)
        set_data += request.aux2.to_bytes(2, byteorder='little', signed=False)
        set_data += request.aux3.to_bytes(2, byteorder='little', signed=False)
        set_data += request.aux4.to_bytes(2, byteorder='little', signed=False)
        set_data += bytearray.fromhex("0000 0000 0000 0000 0000 0000 0000 0000")

        self.get_logger().info(f'\033[38;5;151m Received set RC data: {set_data.hex()} \033[0m')

        msp_request = self.create_msp_request(MSP_SET_RAW_RC, set_data)
        self.serial_port.write(msp_request)

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BetaflightTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
