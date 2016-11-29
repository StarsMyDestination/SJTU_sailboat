import struct
import serial
# from msgdev import MsgDevice, PeriodTimer


fst = struct.Struct('<H3f2h2fH')
numBytes = 28
arduinoPort = 'COM6'
header = chr(0x4f) + chr(0x5e)


def crc16(x):
    poly = 0x8408
    crc = 0x0
    for byte in x:
        crc = crc ^ ord(byte)
        for i in range(8):
            last = (0xFFFF & crc) & 1
            crc = (0xffff & crc) >> 1
            if last == 1:
                crc = crc ^ poly
    return crc & 0xFFFF


def dataRead(s):
    # print 111
    tmp = s.read(numBytes)
    # print type(tmp)
    if (tmp.find(header)) == 0 and len(tmp) == numBytes:
        crc16Num = crc16(tmp[:-2])
        gpsDatas = fst.unpack(tmp)
        if gpsDatas[-1] == crc16Num:
            EGpsData = gpsDatas[1:-1]
            print EGpsData


def main():
    arduinoSer = serial.Serial(arduinoPort, baudrate=115200, timeout = 2)
    arduinoSer.flush()
    print arduinoSer.isOpen()
    while 1:
        dataRead(arduinoSer)


if __name__ == '__main__':
    main()
