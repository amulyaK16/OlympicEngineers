import struct
import serial
import serial.tools.list_ports
import json


def connect_to_HC05_COM_port():
    for p in serial.tools.list_ports.comports():
        if "(COM4)" in p.description:
            print("HC05 Connected Successfully!")
            return p.device
    raise IOError("Could not find STM device. Make sure it's plugged in?")

def create_serial():
    c_port = connect_to_HC05_COM_port()
    port = serial.Serial(c_port, baudrate=9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                         timeout=25)
    return port


def getAck(port, n):
    port.write(bytes(n, 'utf-8'))
    ack = port.read(1)
    ack_str = ack.decode("utf-8")

    if ack_str == '1':
        return True
    else:
        return False

def send_Ping(port):
    return getAck(port, '1')

def send_start_cmd(port):
    return getAck(port, '2')

def send_stop_cmd(port):
    return getAck(port, '3')

def receive_data(port):
    rcv = port.read(190)
    return rcv


'''
bytes_to_packet
----------------------------
Convert raw received byte array to a packt list
which is split based on packet_t struct on MCU

param:
    rcv_bytes: byte array from Serial receive

return: unpacked packet list
'''
def bytes_to_packet(rcv_bytes):
    return struct.unpack('IHHB32H32HfffffffB24c', rcv_bytes)

'''
packet_to_dict
----------------------------
Convert the received packet list to a dictionary
for server processing

param:
    packet: unpacked packet list

return: python dict corresponding to packet_t struct
'''
def packet_to_dict(packet):
    pack_dict = {}

    pack_dict["START"]    = packet[0]
    pack_dict["PKT_SIZE"] = packet[1]
    pack_dict["PKT_NUM"]  = packet[2]
    pack_dict["STATE"]    = packet[3]
    pack_dict["ECG"]      = packet[4:36]
    pack_dict["EMG"]      = packet[36:68]
    pack_dict["FORCE"]    = packet[68]
    pack_dict["GYRO_X"]   = packet[69]
    pack_dict["GYRO_Y"]   = packet[70]
    pack_dict["GYRO_Z"]   = packet[71]
    pack_dict["ACCEL_X"]  = packet[72]
    pack_dict["ACCEL_Y"]  = packet[73]
    pack_dict["ACCEL_Z"]  = packet[74]
    pack_dict["PL_SIZE"]  = packet[75]
    pack_dict["TIME"]     = b''.join(packet[76:])

    return pack_dict

'''
dict_to_json
----------------------------
Convert python dict to json for website or database
use

param:
    pack_dict: python dict corresponding to packet_t struct

return: json of received packet
'''
def dict_to_json(pack_dict):
    return json.dumps(pack_dict)


if __name__ == '__main__':
    port = create_serial()
    state = True
    isnum = False
    count = 0

    while state:
        if send_Ping(port):
            state = False;
            print('ping acknoledged')
        else:
            print('ping failed')

    if send_start_cmd(port):
        print('packet collection started')

    while not isnum:
        BtP = bytes_to_packet(receive_data(port))
        pack_dict = packet_to_dict(BtP)

        print(pack_dict["START"])
        print(pack_dict["PKT_SIZE"])
        print(pack_dict["PKT_NUM"])
        print(pack_dict["STATE"])
        print(pack_dict["ECG"])
        print(pack_dict["EMG"])
        print(pack_dict["FORCE"])
        print(pack_dict["GYRO_X"])
        print(pack_dict["GYRO_Y"])
        print(pack_dict["GYRO_Z"])
        print(pack_dict["ACCEL_X"])
        print(pack_dict["ACCEL_Y"])
        print(pack_dict["ACCEL_Z"])
        print(pack_dict["PL_SIZE"])
        print(pack_dict["TIME"])
        print("")
        print("")

        count += 1
        if count == 1000:
            isnum = True