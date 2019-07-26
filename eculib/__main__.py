import sys
import argparse
import pyftdi.ftdi
import usb.core
import usb.util
import pyftdi
from eculib import KlineAdapter

def GetFtdiDevices():
	return [d for d in usb.core.find(find_all=True, idVendor=pyftdi.ftdi.Ftdi.FTDI_VENDOR)]

def Main():

	devices = GetFtdiDevices()
	default_device = None
	if len(devices) > 0:
		default_device = devices[0]

	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument('-b','--bus', help="usb bus of ftdi device")
	parser.add_argument('-a','--address', help="usb bus address of ftdi device")
	subparsers = parser.add_subparsers(metavar='mode',dest='mode')

	parser_kline = subparsers.add_parser('kline', help='test kline')

	db_grp = parser.add_argument_group('debugging options')
	db_grp.add_argument('--list-devices', action='store_true', help="list ftdi devices")
	args = parser.parse_args()

	if args.list_devices:
		print("FTDI Devices:")
		for cfg in devices:
			print("Bus %03d Device %03d: %s %s %s" % (cfg.bus, cfg.address, usb.util.get_string(cfg,cfg.iManufacturer), usb.util.get_string(cfg,cfg.iProduct), usb.util.get_string(cfg,cfg.iSerialNumber)))
		return
	elif args.device is None:
		print("No FTDI device connected")
		return

	dev = KlineAdapter(args.device)

	if args.mode is None:
		parser.print_help()

	elif args.mode == "kline":
		try:
			oldstate = None
			while True:
				newstate = dev.kline()
				if oldstate != newstate:
					sys.stdout.write("\rK-line state: %d" % newstate)
					sys.stdout.flush()
					oldstate = newstate
		except KeyboardInterrupt:
			sys.stdout.write("\n")
			sys.stdout.flush()

if __name__ == '__main__':
	Main()
