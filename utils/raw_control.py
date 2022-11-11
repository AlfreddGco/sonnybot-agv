import struct, serial
import time

def xorSum(chain):
	s = int.from_bytes(chain[:2], 'little')
	for i in range(2, len(chain), 2):
		s = s ^ int.from_bytes(chain[i:i+2], 'little')
	return int.to_bytes(s, 2, 'little')


def build_message():
	msg = b'\xcd\xab'
	msg += struct.pack('hh', 13, 23)
	msg += xorSum(msg) 
	print(msg)
	return msg


ser = serial.Serial('/dev/ttyS0')

while True:
	ser.write(build_message())
	time.sleep(0.5)	
