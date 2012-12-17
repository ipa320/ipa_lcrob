#!/usr/bin/python
from spi import SPI
import Ox.GPIO as GPIO
import threading


class AVRInterface:
	SET_OUTPUT	=0x10
	SET_MOTOR	=0x20
	GET_ANALOG	=0x30
	GET_INPUT	=0x40
	SETUP		=0x50

	def __init__(self):
		#setup device
		self.spi = SPI(miso=22, mosi=23, clk=20, reset=38)
		self.lock= threading.Lock()

	def write(self,cmd, data=[]):
		self.spi.sendByte(cmd)
		for d in data:
			self.spi.sendByte(d)

	def setup(self):
		self.lock.acquire()
		self.write(self.SETUP)
		assert self.spi.sendByte(0x00)==ord('O')	#no pullups actiavted (0-0x0f)
		self.lock.release()

	def set_output(self, ch, v):
		assert (ch>=0 and ch<6)
		if ch==2 or ch==5: return
		self.lock.acquire()
		self.write( (self.SET_OUTPUT|ch),[v])
		self.lock.release()

	def set_motor(self, motor, back, speed):
		self.lock.acquire()
		if motor==0: self.write( (self.SET_MOTOR|motor|(back<<1)), [speed])
		self.lock.release()

	def get_analog(self, ch):
		assert(ch<4 and ch>=0)
		self.lock.acquire()
		self.write(self.GET_ANALOG|ch)
		self.spi.wait(200)
		data1 = self.spi.readByte()
		data2 = self.spi.readByte()
		self.lock.release()
		return data1 + data2*255

	def get_input(self):
		self.lock.acquire()
		self.write(self.GET_INPUT)
		data = self.spi.readByte()
		self.lock.release()
		r=[]
		for i in range(0,4):
			v = False
			if data&(1<<i)>0: v=True
			r.append( v )
		return r
