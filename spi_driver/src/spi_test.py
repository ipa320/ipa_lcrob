#!/usr/bin/python

from spi import SPI


dev = SPI(miso=22, mosi=23, clk=20, reset=38)
#dev.reset()

while True:
	print dev.sendByte(0x50)
