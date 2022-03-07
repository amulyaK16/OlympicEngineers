import struct
import serial
import numpy as np
import matplotlib.pyplot as plt
import csv
import serial.tools.list_ports

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
		packet = struct.unpack('IHHB32H32Hf32f32f32f32f32f32fB21c', rcv)

		time = b''.join(packet[-21:-4])
		print(time.decode("utf-8"))
		
		for ecg_sample in packet[4:36]:
			writer.writerow({'Time' : time.decode("utf-8"), 'ECG_Val' : ecg_sample})
		
		print_packet(packet)

		cnt = cnt + 1

	f.close()
	ecg_file.close()