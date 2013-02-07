#!/usr/bin/env python
import os, time
from subprocess import Popen, PIPE

if __name__ == "__main__":
        print "waiting for joystick..."
	p1 = Popen(["dmesg"], stdout=PIPE)
	p2 = Popen(["grep", "hid_logitech"], stdin=p1.stdout, stdout=PIPE)
	while len(p2.communicate())<1:
		time.sleep(1)
	os.system("rosrun mobina_interface evjoy2.py")
