import os, sys, platform
from pyftdi.ftdi import Ftdi
import usb.core
from ctypes import *
import time

class KlineAdapter(Ftdi):

	def __init__(self, device, baudrate=10400):
		super(KlineAdapter, self).__init__()
		self.open(device)
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

	def open(self, dev, interface=1):
		self.usb_dev = dev
		try:
			self.usb_dev.set_configuration()
		except usb.core.USBError:
			pass
		# detect invalid interface as early as possible
		config = self.usb_dev.get_active_configuration()
		if interface > config.bNumInterfaces:
			raise FtdiError('No such FTDI port: %d' % interface)
		self._set_interface(config, interface)
		self.max_packet_size = self._get_max_packet_size()
		# Drain input buffer
		self.purge_buffers()
		self._reset_device()
		self.set_latency_timer(Ftdi.LATENCY_MIN)

class ECU(object):

	def __init__(self, klineadapter):
		self.dev = klineadapter
