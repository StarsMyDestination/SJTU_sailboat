import struct
import binascii
from msgdev import MsgDevice, PeriodTimer
# import socket
import math
import serial
from math import *

sendDataFst = struct.Struct('!3B')
recvDataBytes = 92
# unsigned int--H;int--h;char--c;float--f;
recvDataFst = struct.Struct('<H5h14fh2fhfhLH')

arduinoUrl = "socket://192.168.188.200:9000" # exp use
# arduinoUrl = "COM9"  # serial test use
# arduinoUrl = "socket://192.168.199.101:9000"  # lab test use

msgSubConnect = 'tcp://127.0.0.1:5555'
msgPubBind = 'tcp://0.0.0.0:6666'

# encoder offset
sailOffset = -43
windOffset = 90

pubUrls = ['motorMicroSec', 'rudderAng', 'sailAng', 'arduinoReadMark', 'autoFlag',  # control related
           'windAngRead', 'sailAngRead',  # encoder
           'roll', 'pitch', 'yaw', 'Gx', 'Gy', 'Gz', 'Ax', 'Ay', 'Az',  # ahrs
           'UTC', 'north', 'east', 'FS', 'Hacc', 'SOG', 'ageC', 'HDOP', 'SVs',  # gps
           'time']  # arduino time since program starting

# gps origin
latOrigin, lonOrigin = 31.0231885623383, 121.425143928016


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


def d2r(d):
    return d / 180.0 * pi


def w84_calc_ne(lat2, lon2):
    lat1, lon1 = d2r(latOrigin), d2r(lonOrigin)
    lat2, lon2 = d2r(lat2), d2r(lon2)
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    a = 6378137.0
    e_2 = 6.69437999014e-3
    r1 = a * (1 - e_2) / (1 - e_2 * (sin(lat1))**2)**1.5
    r2 = a / sqrt(1 - e_2 * (sin(lat1))**2)

    north = r1 * d_lat
    east = r2 * cos(lat1) * d_lon
    return north, east


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


def dataSend(data):
    header = '\xff'
    tmp = sendDataFst.pack(data[0], data[1], data[2])
    crc_code = struct.pack('!B', crc8(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    # print binascii.hexlify(tmp)
    return tmp


def dataRead(s):
    header = chr(0x4f) + chr(0x5e)
    try:
        buff = s.recv(recvDataBytes)  # for socket
    except(AttributeError):
        buff = s.read(recvDataBytes)  # for serial
    # print binascii.hexlify(buff)
    # print (buff.find(header))
    if (buff.find(header)) == 0 and len(buff) == recvDataBytes:
        crc16Num = crc16(buff[2:-2])
        # print crc16Num
        sensorDatas = recvDataFst.unpack(buff)
        # print sensorData[-1]
        if sensorDatas[-1] == crc16Num:
            EsensorData = sensorDatas[1:-1]
            # print EsensorData
            # print type(EsensorData)
            return list(EsensorData)
        else:
            print "recv error: crc check error"
            return None
    else:
        print "recv error: invalid data"
        return None


def to180(data):
    if data < -180:
        data = data + 360
    elif data > 180:
        data = data - 360
    return data


def pubToVeristand(dev, data):
    data[5] = to180(data[5] - windOffset)  # wind data
    data[6] = to180(data[6] - sailOffset)  # sail data
    north, east = w84_calc_ne(data[17], data[18])  # lat lon
    # print north, east
    data[17] = north
    data[18] = east
    for i in xrange(len(pubUrls)):
        # print pubUrls[i], data[i]
        dev.pub_set1(pubUrls[i], data[i])


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
    sailAng = myMap(sailAng, 0, 80, 50, 130)
    sailAng = constraint(sailAng, 50, 130)
    sailAng = int(sailAng)
    return([motorSpeed, rudderAng, sailAng])


def main():
    # arduinoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    arduinoSer = serial.serial_for_url(
        arduinoUrl, baudrate=115200, do_not_open=True, timeout=1)
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
                data = subFromVeristand(dev)
                # data = [100, 90, 0]
                print "SEND: ", data
                if not arduinoSer._isOpen:
                    try:
                        arduinoSer.open()
                    except serial.serialutil.SerialException, e:
                        print e, ", trying reconnect..."
                try:
                    if arduinoSer._isOpen:
                        arduinoSer.write(dataSend(data))
                        dataFromArduino = dataRead(arduinoSer)
                        print "RECV: ", dataFromArduino
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
