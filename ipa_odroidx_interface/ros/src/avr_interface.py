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
	SET_PULSE	=0x60
	SET_MOTORAIM	=0x70

	def __init__(self):
		#setup device
		self.pullup = 0x00
		self.spi = SPI(miso=22, mosi=23, clk=20, reset=38)
		self.lock= threading.Lock()

	def write(self,cmd, data=[]):
		self.spi.sendByte(cmd)
		for d in data:
			self.spi.sendByte(d)

	def setup(self):
		self.lock.acquire()
		self.write(self.SETUP)
		assert self.spi.sendByte(self.pullup)==ord('O')	#no pullups actiavted (0-0x0f)
		self.lock.release()

	def set_pullup(self, ch):
		assert (ch>=0 and ch<4)
		self.lock.acquire()
		self.pullup |= (1<<ch)
		self.write(self.SETUP)
		assert self.spi.sendByte(self.pullup)==ord('O')	#no pullups actiavted (0-0x0f)
		self.lock.release()

	def set_output(self, ch, v):
		assert (ch>=0 and ch<6)
		#if ch==2 or ch==5: return
		self.lock.acquire()
		self.write( (self.SET_OUTPUT|ch),[v])
		self.lock.release()

        def set_pulse(self, ch, v):
                assert (ch>=0 and ch<6)
                #if ch==2 or ch==5: return
                self.lock.acquire()
                self.write( (self.SET_PULSE|ch),[(v<<1)])
                self.lock.release()

	def set_motor(self, motor, back, speed):
		self.lock.acquire()
		self.write( (self.SET_MOTOR|motor|(back<<1)), [speed])
		self.lock.release()

        def set_motoraim0(self, aim):
                self.lock.acquire()
                self.write( (self.SET_MOTORAIM), [(aim>>8),(aim&0xff)])
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
