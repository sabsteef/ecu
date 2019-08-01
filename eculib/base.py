import os, sys, platform
from pyftdi.ftdi import Ftdi
import usb.core
from ctypes import *
import time

class KlineAdapter(Ftdi):

	def __init__(self, device, baudrate=10400):
		super(KlineAdapter, self).__init__()
		self.open_from_device(device)
		self.set_baudrate(baudrate)
		self.set_line_property(8, 1, "N")

	def kline(self):
		ret = False
		self.purge_buffers()
		starttime = time.time()
		while True:
			self.write_data(b"\xff")
			time.sleep(.002)
			tmp = self._read()
			if len(tmp) == 3:
				ret = (tmp[2] == 0xff)
				break
			if time.time() - starttime > 1:
				break
		self.purge_buffers()
		return ret

class ECU(object):

	def __init__(self, klineadapter):
		self.dev = klineadapter
