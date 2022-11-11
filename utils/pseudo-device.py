import sys, os, pty, serial
import struct, time

START_FRAME = b'\xcd\xab'
MSG_LEN = 22
CMD_LEN = 8

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
	assert len(msg) == MSG_LEN
	return msg


def parse_message(msg):
	start_idx = -1
	for i in range(len(msg) - 2):
		if(msg[i:i+2] == START_FRAME):
			start_idx = i
			break
	if(start_idx != -1):
		it = start_idx + 2
		steer = struct.unpack('h', msg[it:it + 2])
		it += 2
		speed = struct.unpack('h', msg[it:it + 2]) 
		it += 2
		checksum = msg[it:it + 2] 
		s = xorSum(msg[start_idx:start_idx + 6])
		return (steer, speed, checksum == s) 
	return (-1, -1, False)


master_fd, slave_fd = pty.openpty()
slave_filename = os.ttyname(slave_fd)

print(f"Special file assigned: {slave_filename}")
input("Start?")

serial_conn = serial.Serial(slave_filename)
while True:
	os.write(master_fd, build_message())
	# TODO: Throttle logging and define parameter formally
	time.sleep(1/int(sys.argv[1]))
	command = os.read(master_fd, CMD_LEN)
	print("CMD:", parse_message(command))

