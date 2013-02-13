#!/usr/bin/python
import os, time
from subprocess import Popen, PIPE

if __name__ == "__main__":
        print "waiting for joystick..."
	while True:
		p1 = Popen(["dmesg"], stdout=PIPE)
		p2 = Popen(["grep", "-i", "hid"], stdin=p1.stdout, stdout=PIPE)
		p3 = Popen(["grep", "-i", "logitech"], stdin=p2.stdout, stdout=PIPE)
		c = p3.communicate()
		if len(c[0])<1:
			time.sleep(1)
			continue
		time.sleep(1)
		os.system("rosrun mobina_interface evjoy.py")
		break
