import struct
import serial
import serial.tools.list_ports
import json


def connect_to_HC05_COM_port():
    for p in serial.tools.list_ports.comports():
        if "(COM3)" in p.description:
            print("HC05 Connected Successfully!")
            return p.device
    raise IOError("Could not find STM device. Make sure it's plugged in?")


def create_serial():
    c_port = connect_to_HC05_COM_port()
    port = serial.Serial(c_port, baudrate=9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                         timeout=8)
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
    p1 = struct.unpack('IHHB', rcv_bytes[0:9])
    p2 = struct.unpack_from('31H31H', rcv_bytes, 9)
    p3 = struct.unpack_from('fffffffB24c', rcv_bytes, 137)
    return p1 + p2 + p3


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

    pack_dict["START"] = packet[0]
    pack_dict["PKT_SIZE"] = packet[1]
    pack_dict["PKT_NUM"] = packet[2]
    pack_dict["STATE"] = packet[3]
    pack_dict["ECG"] = packet[4:35]
    pack_dict["EMG"] = packet[35:66]
    pack_dict["FORCE"] = packet[66]
    pack_dict["GYRO_X"] = packet[67]
    pack_dict["GYRO_Y"] = packet[68]
    pack_dict["GYRO_Z"] = packet[69]
    pack_dict["ACCEL_X"] = packet[70]
    pack_dict["ACCEL_Y"] = packet[71]
    pack_dict["ACCEL_Z"] = packet[72]
    pack_dict["PL_SIZE"] = packet[73]
    pack_dict["TIME"] = str(b''.join(packet[74:]), "utf-8")

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

    with open('json_data.json', 'w') as outfile:

        while not isnum:
            test = receive_data(port)
            print(test.hex(" "))
            BtP = bytes_to_packet(test)
            pack_dict = packet_to_dict(BtP)
            js = dict_to_json(pack_dict)

            json.dump(js, outfile)

            print(js)

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















