import struct
import serial


fst = struct.Struct('<3f2h2f')
numBytes = 24
arduinoPort = 'COM6'


def crc8(x):
    poly = 0x84
    crc = 0x0
    for byte in x:
        crc = crc ^ ord(byte)
        for i in range(8):
            last = (0xFF & crc) & 1
            crc = (0xff & crc) >> 1
            if last == 1:
                crc = crc ^ poly
    return crc & 0xFF


def dataRead(s):
    # print 111
    tmp = s.read(24)
    print tmp
    gpsDatas = fst.unpack(tmp)
    print gpsDatas


def main():
    arduinoSer = serial.Serial(arduinoPort, baudrate=115200, timeout = 2)
    print arduinoSer.isOpen()
    while 1:
        dataRead(arduinoSer)


if __name__ == '__main__':
    main()
