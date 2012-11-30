#!/usr/bin/python

import time
import Ox.GPIO as GPIO

class SPI:
	def __init__(self, miso, mosi, clk, reset):
		# set up GPIO output channel
		GPIO.setup(miso, GPIO.IN)
		GPIO.setup(mosi, GPIO.OUT)
		GPIO.setup(clk,  GPIO.OUT)
		GPIO.setup(reset,GPIO.OUT)

		self.miso  = miso
		self.mosi  = mosi
		self.clk  = clk
		self.reset  = reset

		GPIO.output(self.reset, GPIO.HIGH)
		GPIO.output(self.mosi, GPIO.LOW)

		self.rest_ms = 20./10000
		time.sleep(0.1)

	def reset(self):
		GPIO.output(self.reset, GPIO.LOW)
		time.sleep(0.05)
		GPIO.output(self.reset, GPIO.HIGH)
		time.sleep(0.05)

	def sendByte(self, byte):
		r = 0
		for i in range(0,8):
			v = GPIO.LOW
			if (byte&(1<<(7-i)))>0: v = GPIO.HIGH
			GPIO.output(self.mosi, v)
			GPIO.output(self.clk, GPIO.HIGH)
			time.sleep(self.rest_ms)
			r |= (GPIO.input(self.miso)<<i)
			GPIO.output(self.clk, GPIO.LOW)
			time.sleep(self.rest_ms)
		return r

	def readByte(self):
		return self.sendByte(0)
