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
		self.mot_left_avg   = None
		self.mot_left_std   = None
		self.mot_right_avg  = None
		self.mot_right_std  = None

class plt_data():
	def __init__(self):
		self.lock_common = Lock()
		self.array_time  = np.array([])
		self.array_angle = np.array([])
		self.array_gypo  = np.array([])
		self.array_lenc  = np.array([])
		self.array_renc  = np.array([])
		self.array_lmot  = np.array([])
		self.array_rmot  = np.array([])
		self.array_ldrv  = np.array([]) 
		self.array_rdrv  = np.array([])
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
		self.lenc_avg    = None
		self.lenc_std    = None
		self.lenc_min    = None
		self.lenc_max    = None
		self.renc_avg    = None
		self.renc_std    = None
		self.renc_min    = None
		self.renc_max    = None
		self.lmot_avg    = None
		self.lmot_std    = None
		self.lmot_min    = None
		self.lmot_max    = None
		self.rmot_avg    = None
		self.rmot_std    = None
		self.rmot_min    = None
		self.rmot_max    = None

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
		self.lenc_avg   = np.mean(self.array_lenc)
		self.lenc_std   = np.std(self.array_lenc)
		self.lenc_min   = np.min(self.array_lenc)
		self.lenc_max   = np.max(self.array_lenc)
		self.renc_avg   = np.mean(self.array_renc)
		self.renc_std   = np.std(self.array_renc)
		self.renc_min   = np.min(self.array_renc)
		self.renc_max   = np.max(self.array_renc)
		self.lmot_avg   = np.mean(self.array_lmot)
		self.lmot_std   = np.std(self.array_lmot)
		self.lmot_min   = np.min(self.array_lmot)
		self.lmot_max   = np.max(self.array_lmot)
		self.rmot_avg   = np.mean(self.array_rmot)
		self.rmot_std   = np.std(self.array_rmot)
		self.rmot_min   = np.min(self.array_rmot)
		self.rmot_max   = np.max(self.array_rmot)

	def get_data(self):
		data = {}
		self.lock_common.acquire()
		data['time']  = deepcopy(self.array_time)
		data['angle'] = deepcopy(self.array_angle)
		data['gypo']  = deepcopy(self.array_gypo)
		data['lenc']  = deepcopy(self.array_lenc)
		data['renc']  = deepcopy(self.array_renc)
		data['lmot']  = deepcopy(self.array_lmot)
		data['rmot']  = deepcopy(self.array_rmot)
		data['time_min']  = self.time_min 
		data['time_max']  = self.time_max 
		data['angle_avg'] = self.angle_avg
		data['angle_std'] = self.angle_std
		data['angle_min'] = self.angle_min
		data['angle_max'] = self.angle_max
		data['gypo_avg']  = self.gypo_avg 
		data['gypo_std']  = self.gypo_std 
		data['gypo_min']  = self.gypo_min 
		data['gypo_max']  = self.gypo_max 
		data['lenc_avg']  = self.lenc_avg 
		data['lenc_std']  = self.lenc_std 
		data['lenc_min']  = self.lenc_min 
		data['lenc_max']  = self.lenc_max 
		data['renc_avg']  = self.renc_avg 
		data['renc_std']  = self.renc_std 
		data['renc_min']  = self.renc_min 
		data['renc_max']  = self.renc_max 
		data['lmot_avg']  = self.lmot_avg 
		data['lmot_std']  = self.lmot_std 
		data['lmot_min']  = self.lmot_min 
		data['lmot_max']  = self.lmot_max 
		data['rmot_avg']  = self.rmot_avg 
		data['rmot_std']  = self.rmot_std 
		data['rmot_min']  = self.rmot_min 
		data['rmot_max']  = self.rmot_max 
		self.lock_common.release()
		return data

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
		self.list_mot_left   = []
		self.list_mot_right  = []

	def reset(self, cmd=None):
		self.cmd = cmd_type() if cmd == None else cmd
		self.list_mpu_count  = []
		self.list_mpu_angle  = []
		self.list_mpu_gypo   = []
		self.list_enc_left   = []
		self.list_enc_right  = []
		self.list_mot_left   = []
		self.list_mot_right  = []

	def add_data(self, env):
		if len(self.list_mpu_count) == 0:
			for i in range(self.len_time):
				self.list_mpu_count.append(env.mpu_count - self.unit_time*(self.len_time-i))
				self.list_mpu_angle.append(env.mpu_bal_angle)
				self.list_mpu_gypo.append(env.mpu_bal_gypo)
				self.list_enc_left.append(env.enc_left)
				self.list_enc_right.append(env.enc_right)
				self.list_mot_left.append(env.mot_left)
				self.list_mot_right.append(env.mot_right)
		self.list_mpu_count.append(env.mpu_count)
		self.list_mpu_angle.append(env.mpu_bal_angle)
		self.list_mpu_gypo.append(env.mpu_bal_gypo)
		self.list_enc_left.append(env.enc_left)
		self.list_enc_right.append(env.enc_right)
		self.list_mot_left.append(env.mot_left)
		self.list_mot_right.append(env.mot_right)
		self.make_plt()
		self.make_record()

	def make_plt(self):
		self.plt_data.lock_common.acquire()
		self.plt_data.array_time  = np.array(self.list_mpu_count[-self.len_time:])/200
		self.plt_data.array_angle = np.array(self.list_mpu_angle[-self.len_time:])/1000
		self.plt_data.array_gypo  = np.array(self.list_mpu_gypo[-self.len_time:])/1000
		self.plt_data.array_lenc  = np.array(self.list_enc_left[-self.len_time:])
		self.plt_data.array_renc  = np.array(self.list_enc_right[-self.len_time:])
		self.plt_data.array_lmot  = np.array(self.list_mot_left[-self.len_time:])
		self.plt_data.array_rmot  = np.array(self.list_mot_right[-self.len_time:])
		self.plt_data.calc()
		self.plt_data.lock_common.release()

	def make_record(self):
		self.record.mpu_angle_avg  = self.plt_data.angle_avg
		self.record.mpu_angle_std  = self.plt_data.angle_std
		self.record.mpu_gypo_avg   = self.plt_data.gypo_avg
		self.record.mpu_gypo_std   = self.plt_data.gypo_std
		self.record.enc_left_avg   = self.plt_data.lenc_avg
		self.record.enc_left_std   = self.plt_data.lenc_std
		self.record.enc_right_avg  = self.plt_data.renc_avg
		self.record.enc_right_std  = self.plt_data.renc_std
		self.record.mot_left_avg   = self.plt_data.lmot_avg
		self.record.mot_left_std   = self.plt_data.lmot_std
		self.record.mot_right_avg  = self.plt_data.rmot_avg
		self.record.mot_right_std  = self.plt_data.rmot_std

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
		self.bal_kp = 25000
		self.bal_kd = 20000
		self.vel_kp = 90
		self.vel_ki = 0
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
		self.mot_left = p[14]
		self.mot_right = p[15]
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
		self.mot_left,
		self.mot_right)
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
		self.cmd.bal_angle = (int)(db_env.plt_data.angle_avg * 1000)
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

	data 	= db_env.plt_data.get_data()

	t     	= data['time']
	min_t 	= data['time_min']
	max_t 	= data['time_max']

	a	= data['angle']
	min_a	= data['angle_min']
	max_a	= data['angle_max']
	avg_a	= data['angle_avg']
	std_a	= data['angle_std']

	g	= data['gypo']
	min_g	= data['gypo_min']
	max_g	= data['gypo_max']
	avg_g	= data['gypo_avg']
	std_g	= data['gypo_std']

	le	= data['lenc']
	min_le	= data['lenc_min']
	max_le	= data['lenc_max']
	avg_le	= data['lenc_avg']
	std_le	= data['lenc_std']


	re	= data['renc']
	min_re	= data['renc_min']
	max_re	= data['renc_max']
	avg_re	= data['renc_avg']
	std_re	= data['renc_std']

	lm	= data['lmot']
	min_lm	= data['lmot_min']
	max_lm	= data['lmot_max']
	avg_lm	= data['lmot_avg']
	std_lm	= data['lmot_std']

	rm	= data['rmot']
	min_rm	= data['rmot_min']
	max_rm	= data['rmot_max']
	avg_rm	= data['rmot_avg']
	std_rm	= data['rmot_std']

	if len(t) == 0 or min_t == None :
		return

	### ax1 =========================================
	min_a = min_a * 0.9 if min_a > 0 else min_a * 1.1
	max_a = max_a * 1.1 if max_a > 0 else max_a * 0.9

	ax1.set_xlim(min_t, max_t)
	ax1.set_ylim(min_a, max_a)

	mpu_angle.set_data(t, a)
	mpu_angle_avg.set_data((min_t,max_t),avg_a)

	ax1.set_title(format('angle: %.2f(avg) / %.2f(std)' % (avg_a, std_a)), fontsize=16)

	### ax2 =========================================
	min_g = min_g * 0.9 if min_g > 0 else min_g * 1.1
	max_g = max_g * 1.1 if max_g > 0 else max_g * 0.9

	ax2.set_xlim(min_t, max_t)
	ax2.set_ylim(min_g, max_g)

	mpu_gypo.set_data(t, g)
	mpu_gypo_avg.set_data((min_t,max_t),avg_g)

	ax2.set_title(format('gypo: %.2f(avg) / %.2f(std)' % (avg_g, std_g)), fontsize=16)

	### ax3 =========================================
	min_le = min_le * 0.9 if min_le > 0 else min_le * 1.1
	max_le = max_le * 1.1 if max_le > 0 else max_le * 0.9

	ax3.set_xlim(min_t, max_t)
	ax3.set_ylim(min_le, max_le)

	left_enc.set_data(t, le)
	left_enc_avg.set_data((min_t,max_t),avg_le)

	ax3.set_title(format('enc: %.2f(avg) / %.2f(std)' % (avg_le, std_le)), fontsize=16)

	### ax4 =========================================
	min_lm = min_lm * 0.9 if min_lm > 0 else min_lm * 1.1
	max_lm = max_lm * 1.1 if max_lm > 0 else max_lm * 0.9

	ax4.set_xlim(min_t, max_t)
	ax4.set_ylim(min_lm, max_lm)

	left_mot.set_data(t, lm)
	left_mot_avg.set_data((min_t,max_t),avg_lm)

	ax4.set_title(format('moto: %.2f(avg) / %.2f(std)' % (avg_lm, std_lm)), fontsize=16)

	### axes finished. canvas draw ==================

	fig.canvas.draw()

	return

#==================================================

if __name__ == '__main__':
	db_env = database()

	blue = bluecom()
	blue.start()

	ctrl = control(blue)
	ctrl.start()
	
	fig = plt.figure()

	ax1 = plt.subplot(221)
	plt.grid()
	ax2 = plt.subplot(223)
	plt.grid()
	ax3 = plt.subplot(222)
	plt.grid()
	ax4 = plt.subplot(224)
	plt.grid()

	mpu_angle, mpu_angle_avg = ax1.plot([],[],'-',[],[],'--')
	mpu_gypo,  mpu_gypo_avg	 = ax2.plot([],[],'-',[],[],'--')
	left_enc,  left_enc_avg  = ax3.plot([],[],'-',[],[],'--')
	left_mot,  left_mot_avg  = ax4.plot([],[],'-',[],[],'--')

	anim = animation.FuncAnimation(fig, update,interval=40)
	plt.show()

	blue.close()

