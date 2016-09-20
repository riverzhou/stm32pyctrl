#!/usr/bin/env python3

from serial import Serial
from struct import calcsize, pack, unpack

fmt_output = '=HHiiiiiiiiiHH'
len_output = calcsize(fmt_output)

serial_dev = '/dev/rfcomm0'
serial_rate= 9600

'''
struct env_t{
        int env_lock;
        int bat_voltage;
        int bal_angle;
        int bal_kp;
        int bal_kd;
        int vel_kp;
        int vel_ki;
        int enc_filte;
        int mpu_count;
        int mpu_bal_angle;
        int mpu_bal_gypo;
        int mpu_turn_gypo;
        int enc_left;
        int enc_right;
        int moto_left;
        int moto_right;
        int cmd_forward;
        int cmd_back;
        int cmd_left;
        int cmd_right;
};

struct env_buff_t{
        unsigned short head;
        unsigned short len;
        struct env_t env;
        unsigned short alen;
        unsigned short sum;
};
'''

class env_type():
	def __init__(self,p):
		self.env_lock = p[0]
		self.bat_voltage = p[1]
		self.bal_angle = p[2]
		self.bal_kp = p[3]
		self.bal_kd = p[4]
		self.vel_kp = p[5]
		self.vel_ki = p[6]
		self.enc_filte = p[7]
		self.mpu_count = p[8]
		self.mpu_bal_angle = p[9]
		self.mpu_bal_gypo = p[10]
		self.mpu_turn_gypo = p[11]
		self.enc_left = p[12]
		self.enc_right = p[13]
		self.moto_left = p[14]
		self.moto_right = p[15]
		self.cmd_forward = p[16]
		self.cmd_back = p[17]
		self.cmd_left = p[18]
		self.cmd_right = p[19]

	def print(self):
		print('bat_v %-4d bal_a %-4d bal_p %-4d bal_d %-4d vel_p %-4d vel_i %-4d enc_f %-4d mpu_c %-6d mbal_a %-6d mbal_g %-8d mbal_t %-8d enc_l %-3d enc_r %-3d mot_l %-6d mot_r %-6d' % 
		(
		self.bat_voltage,
		self.bal_angle,
		self.bal_kp,
		self.bal_kd,
		self.vel_kp,
		self.vel_ki,
		self.enc_filte,
		self.mpu_count,
		self.mpu_bal_angle,
		self.mpu_bal_gypo,
		self.mpu_turn_gypo,
		self.enc_left,
		self.enc_right,
		self.moto_left,
		self.moto_right)
		)
		return


class data_input():
	fmt_input  = '=HHiiiiiiiiiiiiiiiiiiiiHH'
	len_input  = calcsize(fmt_input)

	def __init__(self, data):
		self.checked = False
		if len(data) < self.len_input : return
		raw = data[-self.len_input:]
		p = unpack(self.fmt_input,raw)
		self.head = p[0]
		self.len  = p[1]
		self.alen = p[22]
		self.sum  = p[23]
		if(self.check(raw)!= True) : return
		self.checked = True
		self.env  = env_type(p[2:22])

	def check(self, raw):
		if(self.head != 0xff or self.len != self.len_input or self.len & self.alen):
			return False
		s = 0
		for b in raw[:-2]:
			s += b
		if(self.sum != s):
			print('data_input check sum failed')
			return False
		return True

	def print(self):
		self.env.print()


def ser_init(dev,bound):
	ser = Serial(dev, bound)
	if(ser.isOpen()):
		print(dev,':open succeed')
		return ser
	else:
		print(dev,':open failed')
		return None


def proc_read(ser):
	buff = b''
	while (1):
		data = ser.read()
		#print(data,end='',flush=True)
		buff += data
		package = data_input(buff)
		if(package.checked):
			#print('package parsed , buff reset')
			package.print()
			buff = b''
	

if __name__ == '__main__':
	ser=ser_init(serial_dev,serial_rate)
	if(ser==None): exit()
	proc_read(ser)
	ser.close()


