import struct
import serial
import numpy as np
import matplotlib.pyplot as plt
import csv
import serial.tools.list_ports
import json

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
    return struct.unpack('IHHB32H32HfffffffB21c', rcv_bytes)

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
    pack_dict["ECG"]      = packet[4:37]
    pack_dict["EMG"]      = packet[37:70]
    pack_dict["GYRO_X"]   = packet[70]
    pack_dict["GYRO_Y"]   = packet[71]
    pack_dict["GYRO_Z"]   = packet[72]
    pack_dict["ACCEL_X"]  = packet[73]
    pack_dict["ACCEL_Y"]  = packet[74]
    pack_dict["ACCEL_Z"]  = packet[75]
    pack_dict["TIME"]     = b''.join(packet[-21:-4])

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