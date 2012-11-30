import time
import Ox.GPIO as GPIO

class SPI:
	def __init__(self, miso, mosi, clk):
		# set up GPIO output channel
		GPIO.setup(self.miso = miso, GPIO.IN)
		GPIO.setup(self.mosi = mosi, GPIO.OUT)
		GPIO.setup(self.clk  = clk,  GPIO.OUT)
		GPIO.output(self.mosi, GPIO.LOW)
		self.rest_ms = 10./1000

	def reset(self):
		GPIO.output(self.reset, GPIO.LOW)
		time.sleep(0.05)
		GPIO.output(self.reset, GPIO.HIGH)
		time.sleep(0.05)

	def sendByte(self, byte):
		for i in range(0,8):
			GPIO.output(self.mosi, (byte&(1<<i))>0?GPIO.HIGH:GPIO.LOW)
			GPIO.output(self.mosi, GPIO.HIGH)
			time.sleep(self.rest_ms)
			GPIO.output(self.mosi, GPIO.LOW)
			time.sleep(self.rest_ms)

	def readByte(self, byte):
		byte = 0
		for i in range(0,8):
			GPIO.output(self.mosi, GPIO.HIGH)
			time.sleep(self.rest_ms)
			byte |= (GPIO.input(self.miso)<<i)
			GPIO.output(self.mosi, GPIO.LOW)
			time.sleep(self.rest_ms)
		return byte
