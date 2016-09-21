#!/usr/bin/env python3

from serial import Serial
from struct import calcsize, pack, unpack
from time   import sleep
from threading import Lock,Thread
from copy   import deepcopy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#import pylab as pl


#==================================================

class record():
	def __init__(self):
		self.mpu_angle_avg = None
		self.mpu_angle_std = None
		self.mpu_gypo_avg   = None
		self.mpu_gypo_std   = None
		self.enc_left_avg   = None
		self.enc_left_std   = None
		self.enc_right_avg  = None
		self.enc_right_std  = None

class plt_data():
	def __init__(self):
		self.lock_common = Lock()
		self.array_time  = np.array([])
		self.array_angle = np.array([])
		self.array_gypo  = np.array([])
		self.array_left  = np.array([])
		self.array_right = np.array([])
		self.time_min    = None
		self.time_max    = None
		self.angle_avg   = None
		self.angle_std   = None
		self.angle_min   = None
		self.angle_max   = None
		self.gypo_avg    = None
		self.gypo_std    = None
		self.gypo_min    = None
		self.gypo_max    = None
		self.left_avg    = None
		self.left_std    = None
		self.left_min    = None
		self.left_max    = None
		self.right_avg   = None
		self.right_std   = None
		self.right_min   = None
		self.right_max   = None

	def calc(self):
		self.time_min   = np.min(self.array_time)
		self.time_max   = np.max(self.array_time)
		self.angle_avg  = np.mean(self.array_angle)
		self.angle_std  = np.std(self.array_angle)
		self.angle_min  = np.min(self.array_angle)
		self.angle_max  = np.max(self.array_angle)
		self.gypo_avg   = np.mean(self.array_gypo)
		self.gypo_std   = np.std(self.array_gypo)
		self.gypo_min   = np.min(self.array_gypo)
		self.gypo_max   = np.max(self.array_gypo)
		self.left_avg   = np.mean(self.array_left)
		self.left_std   = np.std(self.array_left)
		self.left_min   = np.min(self.array_left)
		self.left_max   = np.max(self.array_left)
		self.right_avg  = np.mean(self.array_right)
		self.right_std  = np.std(self.array_right)
		self.right_min  = np.min(self.array_right)
		self.right_max  = np.max(self.array_right)

	def get_angle(self):
		self.lock_common.acquire()
		x = deepcopy(self.array_time)
		y = deepcopy(self.array_angle)
		min_x = self.time_min
		max_x = self.time_max
		min_y = self.angle_min
		max_y = self.angle_max
		avg_y = self.angle_avg
		std_y = self.angle_std
		self.lock_common.release()
		return x, y, min_x, min_y, max_x, max_y, avg_y, std_y

class database():
	len_time = 600
	unit_time = 20

	def __init__(self):
		self.lock_common = Lock()
		self.list_record = []
		self.record 	 = record()
		self.cmd 	 = cmd_type()
		self.plt_data 	 = plt_data()

		self.list_mpu_count  = []
		self.list_mpu_angle  = []
		self.list_mpu_gypo   = []
		self.list_enc_left   = []
		self.list_enc_right  = []

	def reset(self, cmd=None):
		self.cmd = cmd_type() if cmd == None else cmd
		self.list_mpu_count  = []
		self.list_mpu_angle  = []
		self.list_mpu_gypo   = []
		self.list_enc_left   = []
		self.list_enc_right  = []

	def add_data(self, env):
		if len(self.list_mpu_count) == 0:
			for i in range(self.len_time):
				self.list_mpu_count.append(env.mpu_count - self.unit_time*(self.len_time-i))
				self.list_mpu_angle.append(env.mpu_bal_angle)
				self.list_mpu_gypo.append(env.mpu_bal_gypo)
				self.list_enc_left.append(env.enc_left)
				self.list_enc_right.append(env.enc_right)
		self.list_mpu_count.append(env.mpu_count)
		self.list_mpu_angle.append(env.mpu_bal_angle)
		self.list_mpu_gypo.append(env.mpu_bal_gypo)
		self.list_enc_left.append(env.enc_left)
		self.list_enc_right.append(env.enc_right)
		self.make_plt()
		self.make_record()

	def make_plt(self):
		self.plt_data.lock_common.acquire()
		self.plt_data.array_time  = np.array(self.list_mpu_count[-self.len_time:])
		self.plt_data.array_angle = np.array(self.list_mpu_angle[-self.len_time:])
		self.plt_data.array_gypo  = np.array(self.list_mpu_gypo[-self.len_time:])
		self.plt_data.array_left  = np.array(self.list_enc_left[-self.len_time:])
		self.plt_data.array_right = np.array(self.list_enc_right[-self.len_time:])
		self.plt_data.calc()
		self.plt_data.lock_common.release()

	def make_record(self):
		self.record.mpu_angle_avg  = self.plt_data.angle_avg
		self.record.mpu_angle_std  = self.plt_data.angle_std
		self.record.mpu_gypo_avg   = self.plt_data.gypo_avg
		self.record.mpu_gypo_std   = self.plt_data.gypo_std
		self.record.enc_left_avg   = self.plt_data.left_avg
		self.record.enc_left_std   = self.plt_data.left_std
		self.record.enc_right_avg  = self.plt_data.right_avg
		self.record.enc_right_std  = self.plt_data.right_std

	def save_record(self):
		self.list_record.append((self.cmd, self.record))

	def print_record_angle(self):
		if len(self.list_record) >= 1:
			print('bal_angle %d --- std_angle %d' % (self.list_record[-1][0].bal_angle, self.list_record[-1][1].mpu_angle_std))

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
		self.bal_angle = 0
		self.bal_kp = 20000
		self.bal_kd = 1000
		self.vel_kp = 80
		self.vel_ki = 400
		self.enc_filte = 800
		self.turn_kp = 0
		self.turn_ki = 0
		self.turn_cmd = 0
		return

	def print(self):
		return

class data_output():
	fmt_output = '=HHiiiiiiiiiHH'
	len_output = calcsize(fmt_output)

	def __init__(self, cmd=None):
		self.cmd  = cmd_type() if cmd == None else cmd
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

	def save2db(self):
		global db_env
		db_env.add_data(self.env)

	def print(self):
		self.env.print()

#==================================================

class control(Thread):
	cycle_time = 120

	def __init__(self,blue):
		Thread.__init__(self)
		self.blue = blue
		self.cmd  = cmd_type()

	def run(self):
		self.blue.proc_write(self.cmd)
		while (1):
			sleep(self.cycle_time)
			self.adjust_cmd()
			self.blue.proc_write(self.cmd)

	def adjust_cmd(self):
		global db_env
		old_angle = self.cmd.bal_angle
		self.cmd.bal_angle = (int)(db_env.plt_data.angle_avg)
		db_env.save_record()
		db_env.reset(self.cmd)
		db_env.print_record_angle()
		print('adjust params: angle from %d to %d' % (old_angle, self.cmd.bal_angle))


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
				package.save2db()
				#print('package parsed , buff reset')
				#package.print()
				buff = b''
	
	def proc_write(self, cmd=None):
		buff = data_output(cmd).makebuff()
		#for b in buff: print(hex(b),' ',end='')
		#print()
		ret = self.dev.write(buff) 
		#print('%d bytes writed'%ret)
		#sleep(1)
		ret = self.dev.write(buff)
		#print('%d bytes writed'%ret)

def update(i):
	global db_env

	x, y, min_x, min_y, max_x, max_y, avg_y, std_y = db_env.plt_data.get_angle()

	if len(x) == 0 or min_x == None :
		return mpu_angle,mpu_angle_avg

	min_y = min_y * 0.9 if min_y > 0 else min_y * 1.1
	max_y = max_y * 1.1 if max_y > 0 else max_y * 0.9

	ax.set_xlim(min_x, max_x)
	ax.set_ylim(min_y, max_y)
	ax.figure.canvas.draw()

	mpu_angle.set_data(x, y)
	mpu_angle_avg.set_data([min_x,max_x],[avg_y, avg_y])
	plt.annotate(format('%.2f / %.2f' % (avg_y, std_y)), xy=(x[0], avg_y), xytext=(x[0], avg_y*0.95))

	return mpu_angle,mpu_angle_avg

#==================================================

if __name__ == '__main__':
	db_env = database()

	blue = bluecom()
	blue.start()

	ctrl = control(blue)
	ctrl.start()
	
	fig = plt.figure()
	ax = plt.axes()
	mpu_angle,mpu_angle_avg = ax.plot([],[],'-',[],[],'--')
	anim = animation.FuncAnimation(fig, update,interval=40)
	plt.grid()
	plt.show()

	blue.close()

