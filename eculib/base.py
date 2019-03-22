import os, sys, platform
from pylibftdi import Device
from ctypes import *
import time

class KlineAdapter(Device):

	def __init__(self, device_id, baudrate=10400):
		super(KlineAdapter, self).__init__(device_id, auto_detach=(platform.system()!="Windows"))
		self.baudrate = baudrate
		self.ftdi_fn.ftdi_usb_reset()
		self.ftdi_fn.ftdi_set_line_property(8, 1, 0)
		self.ftdi_fn.ftdi_usb_purge_buffers()

	def kline(self):
		self.ftdi_fn.ftdi_set_bitmode(1, 0x00)
		self._write(b'\x00')
		time.sleep(.002)
		ret = (self._read(1) == b'\x00')
		self.ftdi_fn.ftdi_set_bitmode(0, 0x00)
		return ret

	def KWP_SlowInit(self):
		self.ftdi_fn.ftdi_set_bitmode(1, 0x00)
		self._write(b'\x00')
		time.sleep(.2)
		self._write(b'\x01')
		time.sleep(.4)
		self._write(b'\x00')
		time.sleep(.4)
		self._write(b'\x01')
		time.sleep(.4)
		self._write(b'\x00')
		time.sleep(.4)
		self._write(b'\x01')
		time.sleep(.2)
		self.ftdi_fn.ftdi_set_bitmode(0, 0x00)
		self.ftdi_fn.ftdi_usb_purge_buffers()

	def KWP_FastInit(self):
		self.ftdi_fn.ftdi_set_bitmode(1, 0x00)
		self._write(b'\x00')
		time.sleep(.025)
		self._write(b'\x01')
		time.sleep(.025)
		self.ftdi_fn.ftdi_set_bitmode(0, 0x00)
		self.ftdi_fn.ftdi_usb_purge_buffers()
		self._write(b"\xc1\x33\xf1\x81\x66")
		self._read(5)
		self._read(7)
		self.ftdi_fn.ftdi_usb_purge_buffers()

class ECU(object):

	def __init__(self, klineadapter):
		self.dev = klineadapter
