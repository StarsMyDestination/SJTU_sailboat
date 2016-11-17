#include <Servo.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>
//servo configure
Servo motor, rudder, sail;
int motorMicroSec = 1500, motorMicroSecOld = 1500;
int rudderAng = 140, rudderAngOld = 140;
int sailAng = 140, sailAngOld = 140;
//data read
const int dataNum = 3; //motorData rudderData sailData
int serial_out_count = 0; //used for counting the times that serial communication error
boolean readMark = 0; //used for serialread flag
//read data servoData,motorData
int motorData = 0, rudderData = 0, sailData = 0;
int motorDataOld = 100, rudderDataOld = 90, sailDataOld = 90;
//remote control pins
const int elevPin = 3, throPin = 4, ruddPin = 5, gearPin = 6;
//auto control flag
boolean autoFlag = 1;
//loop count
int count = 0;
// remote controller duration
int durGear; // used for control type switch (control type: auto or manual)
// velocity limit
int motorVelLimit = 3;
int rudderVelLimit = 4;
int rudderLimit = 35;
int sailVelLimit = 4;
int sailLimit = 40;
// encoder
const byte CS = 22;
const byte CL = 23;
const byte DA1 = 24;
const byte DA2 = 25;
float sailAngRead = 0; //deg
float windAngRead = 0; //deg
// gps
double north, east;
float HDOP, SOG;
int SVs, FS;

void encoderRead() {
  long int Angle_1 = 0;
  long int Angle_2 = 0;
  int Angle_1_data = 0;
  int Angle_2_data = 0;
  digitalWrite(CS, LOW);
  for (int i = 0; i < 16; i++) {
    digitalWrite(CL, LOW);
    delay(1);
    digitalWrite(CL, HIGH);
    if (digitalRead(DA1))  Angle_1 ++;
    if (digitalRead(DA2))  Angle_2 ++;
    Angle_1 = Angle_1 << 1;
    Angle_2 = Angle_2 << 1;
  } //end of for loop
  Angle_1 = Angle_1 >> 1;
  Angle_2 = Angle_2 >> 1;
  digitalWrite(CS, HIGH);
  int Angle_1_Status = 0x003F & Angle_1;
  int Angle_2_Status = 0x003F & Angle_2;
  if ((Angle_1_Status >> 1) == 16 | (Angle_1_Status >> 1) == 19) { // 16=10000B,19=10011B
    Angle_1_data = Angle_1 >> 6;
  }
  else  Angle_1_data = 0;
  if ((Angle_2_Status >> 1) == 16 | (Angle_2_Status >> 1) == 19) { // 16=10000B,19=10011B
    Angle_2_data = Angle_2 >> 6;
  }
  else  Angle_2_data = 0;
  sailAngRead = Angle_Cal(Angle_1_data, 0);
  windAngRead = Angle_Cal(Angle_2_data, 0);
} //end of ReadEncoder

float Angle_Cal(int data, float Offset) {
  float Ang_data = data * 360.0 / 1024.0;

  if (data != 0) {
    if (Ang_data > 180) {
      Ang_data = Ang_data - 360;
    }
    Ang_data = Ang_data - Offset;
  }
  return Ang_data;
}


void setup() {
  motor.attach(9, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10, 1100, 1900);
  sail.attach(11, 1100, 1900);
  pinMode(elevPin, INPUT);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);
  motor.writeMicroseconds(motorMicroSec);
  rudder.write(rudderAng);
  sail.write(sailAng);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  while (Serial2.available()) Serial2.read(); //clear buff
  while (Serial3.available()) Serial3.read(); //clear buff
  durGear = pulseIn(gearPin, HIGH);
  delay(1000);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}


unsigned int calcCRC(int data[3])
{
  unsigned int poly = 0x84;
  unsigned int crc = 0;
  int carry;
  int i_bits;
  for (int j = 0; j < dataNum; j++)
  {
    crc = crc ^ data[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc >> 1;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
}


void serial1Read() {
  int serialdata[dataNum + 1];
  boolean header_find_flag = false;
  while (Serial1.available() > (2 * (dataNum + 2) - 1)) Serial1.read(); //clear the buffer(transfer 4bytes data, 7bytes enough for read)
  while (Serial1.available()) {
    if ((int)Serial1.read() == 255) {
      header_find_flag = true;
      break;
    }
  }
  if (header_find_flag == true && Serial1.available() > dataNum) {
    for (int i = 0; i < (dataNum + 1); i++) {
      serialdata[i] = (int)Serial1.read();
    }
    int Data[dataNum];
    for (int i = 0; i < dataNum; i++) {
      Data[i] = serialdata[i];
    }
    unsigned int crcnum = serialdata[dataNum];
    unsigned int crchecknum = calcCRC(Data);
    if (crcnum == crchecknum) {
      readMark = 1;
      motorData = motorDataOld = Data[0];
      rudderData = rudderDataOld = Data[1];
      sailData = sailDataOld = Data[2];
      serial_out_count = 0;
    }
  }
  else {
    serial_out_count ++;
    readMark = 0;
    if (serial_out_count > 10) {  //timeout=10*0.2=2s
      motorData = 100;
      rudderData = 90;
      sailData = 90;
    }
    else {
      motorData = motorDataOld;
      rudderData = rudderDataOld;
      sailData = sailDataOld;
    }
  }
}


void serial2Read() {

}


void serial3Read() { //for GPS
  char inchar;
  String gpsString = "";
  boolean headIsFound = 0;
  boolean endRead = 0;
  while (Serial3.available()) {
    inchar = Serial3.read();
    if (inchar == '#') {
      headIsFound = 1;
      break;
    }
  }
  while (headIsFound && Serial3.available()) {
    inchar = (char)Serial3.read();
    gpsString += inchar;
    if (inchar == '\n') {
      endRead = 1;
      break;
    }
  }
  if (endRead) {
    String dataStr[6] = {""};
    int commaCount = 0;
    for (int i = 0; i < gpsString.length(); i++) {
      if (gpsString[i] == ',') commaCount ++;
      else {
        dataStr[commaCount] += gpsString[i];
      }
    }
    north = dataStr[0].toFloat();
    east = dataStr[1].toFloat();
    FS = dataStr[2].toInt();
    SVs = dataStr[3].toInt();
    HDOP = dataStr[4].toFloat();
    SOG = dataStr[5].toFloat();
  }
}


void signalSelection() {
  if (autoFlag == 1) {
    motorMicroSec = map(motorData, 0, 200, 1400, 1600);
    rudderAng = map(rudderData, 50, 130, 100, 180);
    sailAng = map(sailData, 50, 130, 100, 180);
  }
  else {
    motorMicroSec = pulseIn(elevPin, HIGH);
    motorMicroSec = map(motorMicroSec, 1100, 1900, 1100, 1900);
    int durRudd = pulseIn(ruddPin, HIGH);
    rudderAng = map(durRudd, 1100, 1900, 100, 180);
    int durThro = pulseIn(throPin, HIGH);
    sailAng = map(durThro, 1100, 1900, 100, 180);
  }
}


void veloLimit() {
  motorMicroSec = constrain(motorMicroSec, motorMicroSecOld - motorVelLimit, motorMicroSecOld + motorVelLimit);

  rudderAng = constrain(rudderAng, rudderAngOld - rudderVelLimit, rudderAngOld + rudderVelLimit); //limit up to 50 deg/s

  sailAng = constrain(sailAng, sailAngOld - sailVelLimit, sailAngOld + sailVelLimit);
}


void servoCtrl() {
  motorMicroSecOld = motorMicroSec = constrain(motorMicroSec, 1400, 1600);
  rudderAngOld = rudderAng = constrain(rudderAng, 140 - rudderLimit, 140 + rudderLimit);
  sailAngOld = sailAng = constrain(sailAng, 140 - sailLimit, 140 + sailLimit);
  motor.writeMicroseconds(motorMicroSec);
  // int rudderAngReverse = map(rudderAng, 100, 180, 180, 100); //reverse the rudder signal
  rudder.write(rudderAng);
  sail.write(sailAng);
}


void dataSend() {
  Serial1.print("#");
  Serial1.print(",");
  Serial1.print(motorMicroSec);
  Serial1.print(",");
  Serial1.print(rudderAng - 140);
  Serial1.print(",");
  Serial1.print(sailAng - 100);
  Serial1.print(",");
  Serial1.print(readMark);
  Serial1.print(",");
  Serial1.print(autoFlag);
  Serial1.print(",");
  Serial1.print(11); //keep for dogvane
  Serial1.print(",");
  Serial1.print(22); //keep for sail encoder
  Serial1.print(",");
  Serial1.print(33); //keep for ahrs roll
  Serial1.print(",");
  Serial1.print(44); //keep for ahrs pitch
  Serial1.print(",");
  Serial1.print(55); //keep for ahrs yaw
  Serial1.print(",");
  Serial1.print(66); //keep for gps pos.x
  Serial1.print(",");
  Serial1.println(77); //keep for gps pos.y
}


void flash() {
  encoderRead();
  serial1Read();
  serial2Read(); //for AHRS
  serial3Read(); //for GPS
  count ++;
  if (count == 10) { //read the gearPin every 10 intervals
    durGear = pulseIn(gearPin, HIGH, 20000);
    count = 0;
  }
  if (durGear < 1950 && durGear > 1050) {
    if (durGear < 1500)
      autoFlag = 0;
    else
      autoFlag = 1;
  }
  signalSelection();
  veloLimit();
  servoCtrl();
  dataSend();
}

void loop() {
}
