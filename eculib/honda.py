import time
import struct
from enum import Enum, auto
from .base import ECU
from pydispatch import dispatcher
from array import array
import pyftdi
import usb

class ECUSTATE(Enum):
	OFF = auto()
	UNKNOWN = auto()
	READ = auto()
	READING = auto()
	OK = auto()
	RECOVER_OLD = auto()
	RECOVER_NEW = auto()
	WRITEx00 = auto()
	WRITEx10 = auto()
	WRITEx20 = auto()
	WRITEx30 = auto()
	WRITEx40 = auto()
	WRITEx50 = auto()
	WRITEx0D = auto()
	WRITEx0F = auto()
	WRITExFA = auto()
	WRITING = auto()
	ERASING = auto()
	INIT_WRITE = auto()
	INIT_RECOVER = auto()
	POSTWRITEx00 = auto()
	POSTWRITEx0F = auto()
	POSTWRITEx12 = auto()

DTC = {
	"01-01": "MAP sensor circuit low voltage",
	"01-02": "MAP sensor circuit high voltage",
	"02-01": "MAP sensor performance problem",
	"07-01": "ECT sensor circuit low voltage",
	"07-02": "ECT sensor circuit high voltage",
	"08-01": "TP sensor circuit low voltage",
	"08-02": "TP sensor circuit high voltage",
	"09-01": "IAT sensor circuit low voltage",
	"09-02": "IAT sensor circuit high voltage",
	"11-01": "VS sensor no signal",
	"12-01": "No.1 primary injector circuit malfunction",
	"13-01": "No.2 primary injector circuit malfunction",
	"14-01": "No.3 primary injector circuit malfunction",
	"15-01": "No.4 primary injector circuit malfunction",
	"16-01": "No.1 secondary injector circuit malfunction",
	"17-01": "No.2 secondary injector circuit malfunction",
	"18-01": "CMP sensor no signal",
	"19-01": "CKP sensor no signal",
	"21-01": "0₂ sensor low voltage",
	"21-02": "0₂ sensor high voltage",
	"23-01": "0₂ sensor heater malfunction",
	"25-02": "Knock sensor circuit malfunction",
	"25-03": "Knock sensor circuit malfunction",
	"29-01": "IACV circuit malfunction",
	"33-02": "ECM EEPROM malfunction",
	"34-01": "ECV POT low voltage malfunction",
	"34-02": "ECV POT high voltage malfunction",
	"35-01": "EGCA malfunction",
	"36-01": "A/F sensor malfunction",
	"38-01": "A/F sensor heater malfunction",
	"48-01": "No.3 secondary injector circuit malfunction",
	"49-01": "No.4 secondary injector circuit malfunction",
	"51-01": "HESD linear solenoid malfunction",
	"54-01": "Bank angle sensor circuit low voltage",
	"54-02": "Bank angle sensor circuit high voltage",
	"56-01": "Knock sensor IC malfunction",
	"82-01": "Fast idle solenoid valve malfunction",
	"86-01": "Serial communication malfunction",
	"88-01": "EVAP purge control solenoid valve malfunction",
	"91-01": "Ignition coil primary circuit malfunction"
}

def format_read(location):
	tmp = struct.unpack(">4B",struct.pack(">I",location))
	return [tmp[1], tmp[3], tmp[2]]

def checksum8bitHonda(data):
	return ((sum(bytearray(data)) ^ 0xFF) + 1) & 0xFF

def checksum8bit(data):
	return 0xff - ((sum(bytearray(data))-1) >> 8)

def validate_checksums(byts, nbyts, cksum):
	ret = False
	fixed = False
	if cksum >= 0 and cksum < nbyts:
		byts[cksum] = checksum8bitHonda(byts[:cksum]+byts[(cksum+1):])
		fixed = True
	ret = checksum8bitHonda(byts)==0
	return ret, fixed, byts

def do_validation(byts, nbyts, cksum=-1):
	status = "good"
	ret, fixed, byts = validate_checksums(byts, nbyts, cksum)
	if not ret:
		status = "bad"
	elif fixed:
		status = "fixed"
	return ret, status, byts

def format_message(mtype, data):
	ml = len(mtype)
	dl = len(data)
	msgsize = 0x02 + ml + dl
	msg = mtype + [msgsize] + data
	msg += [checksum8bitHonda(msg)]
	assert(msg[ml] == len(msg))
	return msg, ml, dl

class HondaECU(ECU):

	def init(self):
		self.dev.set_bitmode(1, 0x01)
		self.dev.write_data(b'\x00')
		time.sleep(.070)
		self.dev.write_data(b'\x01')
		self.dev.set_bitmode(0, 0x00)
		time.sleep(.200)
		self.dev._read()

	def send(self, buf, ml, timeout=.1):
		try:
			msg = "".join([chr(b) for b in buf]).encode("latin1")
			mlen = len(msg)
			self.dev.write_data(msg)
			readbuffer = array('B')
			r = mlen + ml + 1
			starttime = time.time()
			while len(readbuffer) < r:
				tempbuf = self.dev._read()
				length = len(tempbuf)
				i = 0
				if length > 2:
					while i < length:
						readbuffer += tempbuf[(i+2):(i+64)]
						i += 64
				if time.time() - starttime > timeout:
					return None
			r = mlen + readbuffer[r-1]
			while len(readbuffer) < r:
				tempbuf = self.dev._read()
				length = len(tempbuf)
				i = 0
				if length > 2:
					while i < length:
						readbuffer += tempbuf[(i+2):(i+64)]
						i += 64
				if time.time() - starttime > timeout:
					return None
			return readbuffer[mlen:].tostring()
		except:
			pass
		return None

	def send_command(self, mtype, data=[], retries=1):
		msg, ml, dl = format_message(mtype, data)
		r = 0
		ret = None
		while r <= retries:
			dispatcher.send(signal="ecu.debug", sender=self, msg="%d > [%s]" % (r, ", ".join(["%02x" % m for m in msg])))
			resp = self.send(msg, ml)
			if resp:
				if checksum8bitHonda(resp[:-1]) == resp[-1]:
					dispatcher.send(signal="ecu.debug", sender=self, msg="%d < [%s]" % (r, ", ".join(["%02x" % r for r in resp])))
					rmtype = resp[:ml]
					valid = False
					if ml == 3:
						valid = (rmtype[:2] == bytearray(map(lambda x: x | 0x10, mtype[:2])))
					elif ml == 2:
						valid = ([b for b in rmtype]==mtype)
					elif ml == 1:
						valid = (rmtype == bytearray(map(lambda x: x & 0xf, mtype)))
					if valid:
						rml = resp[ml:(ml+1)]
						rdl = ord(rml) - 2 - len(rmtype)
						rdata = resp[(ml+1):-1]
						ret = (rmtype, rml, rdata, rdl)
						break
			r += 1
		return ret

	def ping(self, mode=0x72, retries=0):
		return self.send_command([0xfe],[mode], retries=retries) != None

	def diag(self, mode=0xf0, retries=0):
		return self.send_command([0x72],[0x00, mode], retries=retries) != None

	def detect_ecu_state(self):
		self.init()
		self.ping()
		if self.diag():
			t0 = self.send_command([0x72], [0x71, 0x00], retries=0)
			if t0 is not None:
				if bytes(t0[2][5:7]) != b"\x00\x00":
					return ECUSTATE.OK
		if self.send_command([0x7d], [0x01, 0x01, 0x03], retries=0):
			return ECUSTATE.RECOVER_OLD
		if self.send_command([0x7b], [0x00, 0x01, 0x04], retries=0):
			return ECUSTATE.RECOVER_NEW
		if self.send_command([0x7d], [0x01, 0x01, 0x00], retries=0):
			return ECUSTATE.RECOVER_OLD
		if self.send_command([0x7b], [0x00, 0x01, 0x00], retries=0):
			return ECUSTATE.RECOVER_NEW
		writestatus = self.get_write_status(retries=0)
		if writestatus is not None:
			return ECUSTATE["WRITEx%02X" % writestatus]
		postwrite = self.send_command([0x7e], [0x01, 0x0d], retries=0)
		if postwrite is not None:
			return ECUSTATE["POSTWRITEx%02X" % postwrite[2][1]]
		for i in [0x0,0x4000,0x8000]:
			readinfo = self.send_command([0x82, 0x82, 0x00], format_read(i) + [1], retries=0)
			if not readinfo is None:
				return ECUSTATE.READ
		if self.dev.kline():
			return ECUSTATE.UNKNOWN
		else:
			return ECUSTATE.OFF

	def probe_tables(self, tables=None):
		if not tables:
			tables = [0x10, 0x11, 0x13, 0x17, 0x20, 0x21, 0x60, 0x61, 0x63, 0x67, 0x70, 0x71, 0xd0, 0xd1]
		ret = {}
		for t in tables:
			info = self.send_command([0x72], [0x71, t])
			if info:
				if info[3] > 2:
					ret[t] = [info[3],info[2]]
			else:
				return {}
		return ret

	def do_init_recover(self, retries=10):
		# self.send_command([0x7b], [0x00, 0x01, 0x01], retries=retries)
		# self.send_command([0x7b], [0x00, 0x01, 0x02], retries=retries)
		# self.send_command([0x7b], [0x00, 0x01, 0x03], retries=retries)
		self.send_command([0x7b], [0x00, 0x02, 0x76, 0x03, 0x17], retries=retries)
		self.send_command([0x7b], [0x00, 0x03, 0x75, 0x05, 0x13], retries=retries)

	def do_init_write(self, retries=10):
		# self.send_command([0x7d], [0x01, 0x01, 0x00], retries=retries)
		# self.send_command([0x7d], [0x01, 0x01, 0x01], retries=retries)
		# self.send_command([0x7d], [0x01, 0x01, 0x02], retries=retries)
		# self.send_command([0x7d], [0x01, 0x01, 0x03], retries=retries)
		self.send_command([0x7d], [0x01, 0x02, 0x50, 0x47, 0x4d], retries=retries)
		self.send_command([0x7d], [0x01, 0x03, 0x2d, 0x46, 0x49], retries=retries)

	def get_write_status(self, retries=10):
		status = None
		info = self.send_command([0x7e], [0x01, 0x01, 0x00], retries=retries)
		if info:
			status = info[2][1]
		return status

	def do_erase(self, retries=10):
		ret = False
		self.send_command([0x7e], [0x01, 0x02], retries=retries)
		self.send_command([0x7e], [0x01, 0x03, 0x00, 0x00], retries=retries)
		self.get_write_status(retries=retries)
		self.send_command([0x7e], [0x01, 0x0b, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff], retries=retries)
		self.get_write_status(retries=retries)
		self.send_command([0x7e], [0x01, 0x0e, 0x01, 0x90], retries=retries)
		time.sleep(.040)
		info = self.send_command([0x7e], [0x01, 0x04, 0xff], retries=retries)
		if info:
			if info[2][1] == 0x00:
				ret = True
		return ret

	def do_erase_wait(self, retries=10):
		cont = 1
		while cont:
			time.sleep(.1)
			info = self.send_command([0x7e], [0x01, 0x05], retries=retries)
			if info:
				if info[2][1] == 0x00:
					cont = 0
			else:
				cont = -1
		if cont == 0:
			self.get_write_status(retries=retries)

	def do_post_write(self, retries=10):
		ret = False
		self.send_command([0x7e], [0x01, 0x08], retries=retries)
		time.sleep(.5)
		self.get_write_status(retries=retries)
		self.send_command([0x7e], [0x01, 0x09], retries=retries)
		time.sleep(.5)
		self.get_write_status(retries=retries)
		self.send_command([0x7e], [0x01, 0x0a], retries=retries)
		time.sleep(.5)
		self.get_write_status(retries=retries)
		self.send_command([0x7e], [0x01, 0x0c], retries=retries)
		time.sleep(.5)
		if self.get_write_status(retries=retries) == 0x0f:
			info = self.send_command([0x7e], [0x01, 0x0d], retries=retries)
			if info:
				ret = (info[2][1] == 0x0f)
		return ret

	def get_faults(self, retries=10):
		faults = {'past':[], 'current':[]}
		for i in range(1,0x0c):
			info_current = self.send_command([0x72],[0x74, i], retries=retries)[2]
			for j in [3,5,7]:
				if info_current[j] != 0:
					faults['current'].append("%02d-%02d" % (info_current[j],info_current[j+1]))
			if info_current[2] == 0:
				break
		for i in range(1,0x0c):
			info_past = self.send_command([0x72],[0x73, i], retries=retries)[2]
			for j in [3,5,7]:
				if info_past[j] != 0:
					faults['past'].append("%02d-%02d" % (info_past[j],info_past[j+1]))
			if info_past[2] == 0:
				break
		return faults

	############################################################################

	def _read_flash_bytes(self, location, size=12):
		if size <= 12:
			info = self.send_command([0x82, 0x82, 0x00], format_read(location) + [size])
			if info and struct.unpack("<B",info[1])[0] == size + 5:
				return True, info[2]
		return False, None

	def _read_ram_bytes(self, location, size=12):
		if size <= 12:
			info = self.send_command([0x82, 0x82, 0x04], list(struct.unpack("<2B",struct.pack("<H", location))) + [size])
			if info and struct.unpack("<B",info[1])[0] == size + 5:
				return True, struct.unpack("<%sB" % size, info[2])
		return False, None

	def _read_ram_words(self, location, size=6):
		size2 = size * 2
		if size % 2 == 0 and size <= 6:
			info = self.send_command([0x82, 0x82, 0x05], list(struct.unpack("<2B",struct.pack("<H", location))) + [size])
			if info and struct.unpack("<B",info[1])[0] == size2 + 5:
				return True, struct.unpack("<%sB" % size2, struct.pack("<%sH" % size, *struct.unpack(">%sH" % size, info[2])))
		return False, None

	def _write_ram_bytes(self, location, data):
		size = len(data)
		if size <= 12:
			info = self.send_command([0x82, 0x82, 0x08], list(struct.unpack("<2B",struct.pack("<H", location))) + list(struct.unpack("<%sB" % size, data)) + [size])
			if info and struct.unpack("<B",info[1])[0] == 5:
				return True, info[2]
		return False, None

	def _write_ram_words(self, location, data):
		size = len(data)
		if size % 2 == 0 and size / 2 <= 6:
			info = self.send_command([0x82, 0x82, 0x08], list(struct.unpack("<2B",struct.pack("<H", location))) + list(struct.unpack("<%sB" % size, data)) + [size])
			if info and struct.unpack("<B",info[1])[0] == 5:
				return True, info[2]
		return False, None

	def _read_eeprom_word(self, location):
		info = self.send_command([0x82, 0x82, 0x10], list(struct.unpack("<2B",struct.pack("<H", location))))
		if info and struct.unpack("<B",info[1])[0] == 7:
			return True, struct.unpack("<2B", info[2])
		return False, None

	def _format_eeprom(self):
		self.send_command([0x82, 0x82, 0x19])
