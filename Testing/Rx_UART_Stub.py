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

def connect_to_stm():
	for p in serial.tools.list_ports.comports():
     		if "STM" in p.description:
     			print("STM32WB15CC Connected Successfully!")
     			return p.device
	raise IOError("Could not find STM device. Make sure it's plugged in?")

def connect_to_stm_serial(serial_number):
	for pinfo in serial.tools.list_ports.comports():
		return p.device
	raise IOError("Could not find STM device. Make sure it's plugged in?")

#ser = connect_to_stm_serial(serial_number='85430353531351B09121')

def print_packet(packet):
	print(rcv.hex(' '))
	print("")
	
	print("Start Bit: " + str(packet[0]))
	print("Packet Size: " + str(packet[1]))
	print("Packet Number: " + str(packet[2]))
	print("MCU State: " + str(packet[3]))

	x = 4
	for misc in packet[x:x + 32]:
		print("ECG Value: " + str(misc))
	x = x + 32
	for misc in packet[x:x + 32]:
		print("EMG Value: " + str(misc))
	x = x + 32
	print("Force Value: " + str(packet[x]))
	x = x + 1	
	for misc in packet[x:x + 32]:
		print("Accel X: " + str(misc))	
	x = x + 32
	for misc in packet[x:x + 32]:
		print("Accel Y: " + str(misc))
	x = x + 32	
	for misc in packet[x:x + 32]:
		print("Accel Z: " + str(misc))
	x = x + 32	
	for misc in packet[x:x + 32]:
		print("Gyro X: " + str(misc))
	x = x + 32
	for misc in packet[x:x + 32]:
		print("Gyro Y: " + str(misc))
	x = x + 32	
	for misc in packet[x:x + 32]:
		print("Gyro Z: " + str(misc))
	x = x + 32

	print("PL Size: " + str(packet[x]))
	x = x + 1
	for char in packet[x:x + 24]:
		print("Time: " + str(char))

'''
typedef struct payload_t
{
	uint16_t ecg_s[32];     //heart sensor
	uint16_t emg_s[32];	   //emg sensor
	float    force_s;      //force sensor
	float    accelx_s; //accelerometer sensor
	float    accely_s; //accelerometer sensor
	float    accelz_s; //accelerometer sensor
	float    gyrox_s;  //gyro sensor
	float    gyroy_s;  //gyro sensor
	float    gyroz_s;  //gyro sensor

	uint8_t  payload_size; //size of payload
} __attribute__((packed)) payload_t;
'''

if __name__ == '__main__':
	usb_port = connect_to_stm()
	port = serial.Serial(usb_port, baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=25.0)
	rcv = []
	cnt = 0

	ecg_file = open("ecg_samples.txt", "w", encoding='UTF8')
	print("ECG Sample Collection Running ... ")

	f = open('ECG_SAMPLES.csv', 'w')
	fieldnames = ["Time", "ECG_Val"]
	writer = csv.DictWriter(f, fieldnames=fieldnames, escapechar='\\')
	writer.writeheader()

	while cnt < 1000:
		rcv = port.read(934)
		packet = struct.unpack('IHHB32H32HfffffffB21c', rcv)

		time = b''.join(packet[-21:-4])
		print(time.decode("utf-8"))
		
		for ecg_sample in packet[4:36]:
			writer.writerow({'Time' : time.decode("utf-8"), 'ECG_Val' : ecg_sample})
		
		print_packet(packet)

		cnt = cnt + 1

	f.close()
	ecg_file.close()


