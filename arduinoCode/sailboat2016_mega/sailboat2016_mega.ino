#include <Servo.h>
#include <FlexiTimer2.h>
#define pi 3.141592654;
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
int motorVelLimit = 20;
int rudderVelLimit = 6;
int rudderLimit = 35;
int sailVelLimit = 6;
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
float HDOP, SOG, UTC;
int SVs, FS;
// AHRS
float roll, pitch, yaw, rollOld, pitchOld, yawOld;


void setup() {
  motor.attach(9, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10, 1100, 1900);
  sail.attach(11, 1100, 1900);
  pinMode(elevPin, INPUT);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);
  pinMode(CS, OUTPUT);
  pinMode(CL, OUTPUT);
  pinMode(DA1, INPUT);
  pinMode(DA2, INPUT);
  motor.writeMicroseconds(motorMicroSec);
  rudder.write(rudderAng);
  sail.write(sailAng);
  delay(1000);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial1.flush(); //clear buff
  Serial2.flush(); //clear buff
  durGear = pulseIn(gearPin, HIGH);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}


void encoderRead() {
  long Angle_1 = 0;
  long Angle_2 = 0;
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
  if ((Angle_1_Status >> 1) == 16 || (Angle_1_Status >> 1) == 19) { // 16=10000B,19=10011B
    Angle_1_data = Angle_1 >> 6;
  }
  else  Angle_1_data = 0;
  if ((Angle_2_Status >> 1) == 16 || (Angle_2_Status >> 1) == 19) { // 16=10000B,19=10011B
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
  while (Serial.available() > (2 * (dataNum + 2) - 1)) Serial.read(); //clear the buffer(transfer 4bytes data, 7bytes enough for read)
  while (Serial.available()) {
    if ((int)Serial.read() == 255) {
      header_find_flag = true;
      break;
    }
  }
  if (header_find_flag == true && Serial.available() > dataNum) {
    for (int i = 0; i < (dataNum + 1); i++) {
      serialdata[i] = (int)Serial.read();
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


void serial2Read() { //used for AHRS
  unsigned char n[3] = {0}, x[4] = {0}, y[4] = {0}, z[4] = {0}, crc[2] = {0}, crcfield[15];
  boolean headIsFound = 0;
  while (Serial1.available() > (2 * 17 - 1)) Serial1.read();
  while (Serial1.available()) {
    if ((int)Serial1.read() == 0xFF && (int)Serial1.read() == 0X02) {
      headIsFound = 1;
      break;
    }
  }
  while (headIsFound && Serial1.available() > 15) {
    for (int i = 0; i < 3; i++)
    {
      n[i] = Serial1.read();
      crcfield[i] = n[i];
    }//three bytes 09 0C 00 after FF 02
    for (int i = 0; i < 4; i++) {
      x[i] = Serial1.read();
      crcfield[i + 3] = x[i];
    }//read 4-byte angle for x
    for (int i = 0; i < 4; i++) {
      y[i] = Serial1.read();
      crcfield[i + 7] = y[i];
    }//read 4-byte angle for y
    for (int i = 0; i < 4; i++) {
      z[i] = Serial1.read();
      crcfield[i + 11] = z[i];
    }//read 4-byte angle for z
    for (int i = 0; i < 2; i++) {
      crc[i] = Serial1.read();
    }
    if (ahrsCRC(crcfield, 15) == (int)crc[0] << 8 | (int)crc[1])
    {
      roll = *((float *)x); //tranfer x 4-byte array to a float num
      pitch = *((float *)y); //tranfer y 4-byte array to a float num
      yaw = *((float *)z); //tranfer z 4-byte array to a float num
      roll = roll * 180 / pi;
      pitch = pitch * 180 / pi;
      yaw = yaw * 180 / pi;
      rollOld = roll;
      pitchOld = pitch;
      yawOld = yaw;
    }
    else {
      roll = rollOld;
      pitch = pitchOld;
      yaw = yawOld;
    }
  }
}

unsigned int ahrsCRC(const byte *pBuffer, unsigned int bufferSize)
{
  unsigned int poly = 0x8408;
  unsigned int crc = 0;
  byte carry;
  byte i_bits;
  unsigned int j;
  for (j = 0; j < bufferSize; j++)
  {
    crc = crc ^ pBuffer[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
}


void serial3Read() {
  String gpsString = "";
  while (Serial2.available()) {
    char inchar = Serial2.read();
    gpsString += inchar;
  }
  if (gpsString.startsWith("#@") && gpsString.endsWith("$*\n")) {
    String dataStr[9] = {""};
    int commaCount = 0;
    for (int i = 0; i < gpsString.length(); i++) {
      if (gpsString[i] == ',') commaCount ++;
      else {
        dataStr[commaCount] += gpsString[i];
      }
    }
//    Serial.println(gpsString);
    if (commaCount == 8) {
      UTC = dataStr[1].toFloat();
      north = dataStr[2].toFloat();
      east = dataStr[3].toFloat();
      FS = dataStr[4].toInt();
      SVs = dataStr[5].toInt();
      HDOP = dataStr[6].toFloat();
      SOG = dataStr[7].toFloat();
    }
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
  int rudderAngReverse = map(rudderAng, 100, 180, 180, 100); //reverse the rudder signal
  rudder.write(rudderAngReverse);
  sail.write(sailAng);
}


void dataSend() {
  Serial.print("#");
  Serial.print(",");
  Serial.print(motorMicroSec);
  Serial.print(",");
  Serial.print(rudderAng - 140);
  Serial.print(",");
  Serial.print(sailAng - 100);
  Serial.print(",");
  Serial.print(readMark);
  Serial.print(",");
  Serial.print(autoFlag);
  Serial.print(",");
  Serial.print(windAngRead); //dogvane angle
  Serial.print(",");
  Serial.print(sailAngRead); //sail encoder
  Serial.print(",");
  Serial.print(roll); //keep for ahrs roll
  Serial.print(",");
  Serial.print(pitch); //keep for ahrs pitch
  Serial.print(",");
  Serial.print(yaw); //keep for ahrs yaw
  Serial.print(",");
  Serial.print(UTC); // gps pos.x
  Serial.print(",");
  Serial.print(north); // gps pos.x
  Serial.print(",");
  Serial.print(east); // gps pos.y
  Serial.print(",");
  Serial.print(FS); // FS(Fix Status, 0 no fix; 1 Standard(2D/3D); 2 DGPS)
  Serial.print(",");
  Serial.print(SVs); // SVs(satellites used, range 0-12)
  Serial.print(",");
  Serial.print(HDOP); // HDOP(Horizontal Dilution of Precision， range 0.5-99)
  Serial.print(",");
  Serial.println(SOG); // SOG(speed over ground m/s)
}


void flash() {
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
  encoderRead();
  serial1Read();
  serial2Read(); //for AHRS
  serial3Read(); //for GPS
}
