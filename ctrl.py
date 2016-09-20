#!/usr/bin/env python3

from serial import Serial
from struct import calcsize, pack, unpack
from time   import sleep
from threading import Lock,Thread
from copy   import deepcopy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


#==================================================

len_time = 60
unit_time = 200

list_mpu_anagle = []
list_mpu_count  = []
lock_common_list = Lock()

###############################################

'''
struct cmd_t{
        int bal_angle;
        int bal_kp;
        int bal_kd;
        int vel_kp;
        int vel_ki;
        int enc_filte;
        int turn_kp;
        int turn_ki;
        int turn_cmd;
};

struct cmd_buff_t{
        unsigned short head;
        unsigned short len;
        struct cmd_t cmd;
        unsigned short alen;
        unsigned short sum;
};
{
        cmd_buff_p->cmd.bal_angle = 0;
        cmd_buff_p->cmd.bal_kp    = 500;
        cmd_buff_p->cmd.bal_kd    = 1500;
        cmd_buff_p->cmd.vel_kp    = 80;
        cmd_buff_p->cmd.vel_ki    = 400;
        cmd_buff_p->cmd.enc_filte = 900;
        cmd_buff_p->cmd.turn_kp   = 0;
        cmd_buff_p->cmd.turn_ki   = 0;
        cmd_buff_p->cmd.turn_cmd  = 0;
'''

class cmd_type():
	def __init__(self):
		self.bal_angle = 1
		self.bal_kp = 501
		self.bal_kd = 1500
		self.vel_kp = 80
		self.vel_ki = 400
		self.enc_filte = 900
		self.turn_kp = 0
		self.turn_ki = 0
		self.turn_cmd = 0
		return

	def print(self):
		return

class data_output():
	fmt_output = '=HHiiiiiiiiiHH'
	len_output = calcsize(fmt_output)

	def __init__(self):
		self.cmd = cmd_type()
		self.head = 0xff
		self.len  = self.len_output
		self.alen = (~self.len) & 0xffff
		self.sum  = 0
		return

	def makebuff(self):
		buff = pack(self.fmt_output,\
			self.head,\
			self.len,\
			self.cmd.bal_angle,\
			self.cmd.bal_kp,\
			self.cmd.bal_kd,\
			self.cmd.vel_kp,\
			self.cmd.vel_ki,\
			self.cmd.enc_filte,\
			self.cmd.turn_kp,\
			self.cmd.turn_ki,\
			self.cmd.turn_cmd,\
			self.alen,\
			0)
		s = 0
		for b in buff:
			s += b
		s = s & 0xffff
		checksum = pack('H',s)
		buff = buff[:-2] + checksum
		return buff
		
	def print(self):
		self.cmd.print()

#==================================================

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
		if(self.sum != (s & 0xffff) ):
			print('data_input check sum failed')
			return False
		return True

	def save2list(self):
		global list_mpu_anagle, list_mpu_count, lock_common_list
		lock_common_list.acquire()
		list_mpu_anagle.append(self.env.mpu_bal_angle)
		list_mpu_count.append(self.env.mpu_count)
		lock_common_list.release()

	def print(self):
		self.env.print()

#==================================================

class bluecom(Thread):
	devname = '/dev/rfcomm0'
	rate = 9600

	def __init__(self):
		Thread.__init__(self)
		self.dev = Serial(self.devname, self.rate)
		print('Serial opened')

	def run(self):
		print('bluecom thread started')
		self.proc_read()
	
	def close(self):
		self.dev.close()

	def proc_read(self):
		buff = b''
		while (1):
			data = self.dev.read()
			#print(data,end='',flush=True)
			buff += data
			package = data_input(buff)
			if(package.checked):
				package.save2list()
				#print('package parsed , buff reset')
				#package.print()
				buff = b''
	
	def proc_write(self):
		buff = data_output().makebuff()
		#for b in buff: print(hex(b),' ',end='')
		#print()
		ret = self.dev.write(buff) 
		print('%d bytes writed'%ret)
		sleep(1)
		ret = self.dev.write(buff)
		print('%d bytes writed'%ret)

def update(i):
	global list_mpu_anagle, list_mpu_count, lock_common_list, len_time, unit_time
	lock_common_list.acquire()
	empty = len_time - len(list_mpu_count)
	if empty > 0:
		x = deepcopy(list_mpu_count)
		y = deepcopy(list_mpu_anagle)
	else:
		x = deepcopy(list_mpu_count[-len_time:])
		y = deepcopy(list_mpu_anagle[-len_time:])
	lock_common_list.release()

	if len(x) <= 2: return line,
	if empty > 0:
		a = []
		b = []
		for i in range(empty):
			a.append(x[0] - unit_time*(empty-i))
			b.append(y[0])
		x = a+x
		y = b+y

	maxy=max(y)*1.1
	ax.set_xlim(min(x), max(x))
	ax.set_ylim(-maxy, maxy)
	ax.figure.canvas.draw()

	line.set_data(x, y)
	return line,

#==================================================

if __name__ == '__main__':
	blue=bluecom()
	blue.proc_write()
	blue.start()
	fig = plt.figure()
	ax = plt.axes()
	line, = ax.plot([], [], lw=2)
	#anim = animation.FuncAnimation(fig, update, blit=True)
	anim = animation.FuncAnimation(fig, update)
	plt.show()
	blue.close()

