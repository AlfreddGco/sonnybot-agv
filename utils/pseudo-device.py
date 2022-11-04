import os, pty, serial, time

START_FRAME = b'\xcd\xab'
MSG_LEN = 22

master_fd, slave_fd = pty.openpty()
slave_filename = os.ttyname(slave_fd)

print(f"Special file assigned: {slave_filename}")

serial_conn = serial.Serial(slave_filename)

def xorSum(chain):
	s = int.from_bytes(chain[:2], 'little')
	for i in range(2, len(chain), 2):
		s = s ^ int.from_bytes(chain[i:i+2], 'little')
	return int.to_bytes(s, 2, 'little')


def build_message():
	msg = START_FRAME
	# cmd1
	msg += b'\x00\x00'
	# cmd2
	msg += b'\x00\x00'
	# speedR_meas
	msg += b'\x00\x00'
	# speedL_meas
	msg += b'\x00\x00'
	# wheelR_cnt
	msg += b'\x00\x00'
	# wheelL_cnt
	msg += b'\x00\x00'
	# batVoltage
	msg += b'\x18\x5f'
	# boardTemp
	msg += b'\x23\x00'
	# cmdLed
	msg += b'\x00\x00'
	# checkSum
	msg += xorSum(msg)
	fake = b'\xcd\xab' + str.encode("A"*18) + START_FRAME
	assert len(msg) == MSG_LEN
	#print(msg)
	return msg
	#return fake 


while True:
	length = 1024
	os.write(master_fd, build_message())
	time.sleep(1)
	#print(os.read(master_fd, length))

