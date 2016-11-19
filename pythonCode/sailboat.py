import struct
import binascii
from msgdev import MsgDevice, PeriodTimer
import socket
import math
import serial


fst = struct.Struct('!3B')
arduinoUrl = "socket://192.168.188.200:9000"

msgSubConnect = 'tcp://127.0.0.1:5555'
msgPubBind = 'tcp://0.0.0.0:6666'

dataNum = 17


def constraint(num, lowerLimit, upperLimit):
    if num < lowerLimit:
        num = lowerLimit
    if num > upperLimit:
        num = upperLimit
    return num


def myMap(num, l1, u1, l2, u2):
    k = (u2 - l2) / (u1 - l1)
    tmp = l2 + k * (num - l1)
    return tmp


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


def dataSend(data):
    header = '\xff'
    tmp = fst.pack(data[0], data[1], data[2])
    crc_code = struct.pack('!B', crc8(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    print binascii.hexlify(tmp)
    return tmp


def dataRead(s):
    try:
        tmp = s.recv(1024)
    except(AttributeError):
        tmp = s.readline()
    if tmp.startswith('#') and tmp.endswith('\n'):
        tmp = tmp.rstrip('\n')
        ps = tmp.split(',')
        tmp = ""
        if len(ps) != dataNum:
            print 'invalid length'
            print tmp
            return None
        else:
            try:
                motorMicroSec = int(ps[1])
                rudderAng = int(ps[2])
                sailAng = int(ps[3])
                arduinoReadMark = int(ps[4])
                autoFlag = int(ps[5])
                windAngRead = float(ps[6])
                sailAngRead = float(ps[7])
                roll = float(ps[8])
                pitch = float(ps[9])
                yaw = float(ps[10])
                north = float(ps[11])
                east = float(ps[12])
                FS = int(ps[13])
                SVs = int(ps[14])
                HDOP = float(ps[15])
                SOG = float(ps[16])
                return([motorMicroSec, rudderAng, sailAng, arduinoReadMark,
                        autoFlag, windAngRead, sailAngRead, roll, pitch, yaw,
                        north, east, FS, SVs, HDOP, SOG])
            except Exception, e:
                print "data read from Arduino error", e
                return None
    else:
        print 'invalid data'
        return None


def pubToVeristand(dev, data):
    dev.pub_set1('motorMicroSec', data[0])
    dev.pub_set1('rudderAng', data[1])
    dev.pub_set1('sailAng', data[2])
    dev.pub_set1('arduinoReadMark', data[3])
    dev.pub_set1('autoFlag', data[4])
    dev.pub_set1('windAngRead', data[5])
    dev.pub_set1('sailAngRead', data[6])
    dev.pub_set1('roll', data[7])
    dev.pub_set1('pitch', data[8])
    dev.pub_set1('yaw', data[9])
    dev.pub_set1('north', data[10])
    dev.pub_set1('east', data[11])
    dev.pub_set1('FS', data[12])
    dev.pub_set1('SVs', data[13])
    dev.pub_set1('HDOP', data[14])
    dev.pub_set1('SOG', data[15])


def subFromVeristand(dev):
    motorSpeed = dev.sub_get1('motorSpeed')
    rudderAng = dev.sub_get1('rudderAng')
    sailAng = dev.sub_get1('sailAng')
    if math.isnan(motorSpeed) or math.isnan(rudderAng):
        return ([100, 90, 90])
    motorSpeed = myMap(motorSpeed, -100, 100, 0, 200)
    motorSpeed = constraint(motorSpeed, 0, 200)
    motorSpeed = int(motorSpeed)
    rudderAng = myMap(rudderAng, -40, 40, 50, 130)
    rudderAng = constraint(rudderAng, 50, 130)
    rudderAng = int(rudderAng)
    sailAng = myMap(sailAng, -80, 80, 20, 160)
    sailAng = constraint(sailAng, 20, 160)
    sailAng = int(sailAng)
    return([motorSpeed, rudderAng, sailAng])


def main():
    # arduinoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, )
    arduinoSer = serial.serial_for_url(arduinoUrl, baudrate=115200, do_not_open = True, timeout = 1)
    dev = MsgDevice()
    dev.open()
    dev.pub_bind(msgPubBind)
    dev.sub_connect(msgSubConnect)
    dev.sub_add_url('motorSpeed')
    dev.sub_add_url('rudderAng')
    dev.sub_add_url('sailAng')

    t = PeriodTimer(0.1)
    t.start()
    try:
        try:
            arduinoSer.open()
        except serial.serialutil.SerialException, e:
            arduinoSer.close()
            print e, ", trying reconnet..."
        while True:
            with t:
                # data = subFromVeristand(dev)
                data = [100, 90, 0]
                print data
                if not arduinoSer._isOpen:
                    try:
                        arduinoSer.open()
                    except serial.serialutil.SerialException, e:
                        print e, ", trying reconnect..."
                try:
                    if arduinoSer._isOpen:
                        arduinoSer.write(dataSend(data))
                        dataFromArduino = dataRead(arduinoSer)
                        print dataFromArduino
                except serial.serialutil.SerialException, e:
                    arduinoSer.close()
                try:
                    if dataFromArduino:
                        pubToVeristand(dev, dataFromArduino)
                except UnboundLocalError:
                    pass
    finally:
        dev.close()
        arduinoSer.close()
        print "cleaning up..., dev and serial closed!"


if __name__ == '__main__':
    main()
