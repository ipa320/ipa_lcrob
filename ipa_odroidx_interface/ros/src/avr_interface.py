#!/usr/bin/python
from spi import SPI


class AVRInterface:
	SET_OUTPUT	=0x10
	SET_MOTOR	=0x20
	GET_ANALOG	=0x30
	GET_INPUT	=0x40
	SETUP		=0x50

	def __init__(self):
		#setup device
		self.spi = SPI(miso=22, mosi=23, clk=20, reset=38)

	def write(self,cmd, data=[]):
		self.spi.sendByte(cmd)
		for d in data:
			self.spi.sendByte(d)

	def setup(self):
		self.write(self.SETUP)
		assert self.spi.sendByte(0x00)==ord('O')	#no pullups actiavted (0-0x0f)

	def set_output(self, ch, v):
		assert (ch>=0 and ch<6)
		if ch==0 or ch==5: return
		self.write( (self.SET_OUTPUT|ch),[v])

	def set_motor(self, motor, back, speed):
		self.write( (self.SET_MOTOR|motor|(back<<1)), [speed])
		self.spi.GPIO.usleep(500)

	def get_analog(self, ch):
		assert(ch<4 and ch>=0)
		self.write(self.GET_ANALOG|ch)
		self.spi.wait(200)
		data1 = self.spi.readByte()
		data2 = self.spi.readByte()
		return data1 + data2*255

	def get_input(self):
		self.write(self.GET_INPUT)
		data = self.spi.readByte()
		r=[]
		for i in range(0,4):
			v = False
			if data&(1<<i)>0: v=True
			r.append( v )
		return r
