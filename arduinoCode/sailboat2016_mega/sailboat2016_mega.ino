#include <Servo.h>
#include <FlexiTimer2.h>
#define pi 3.141592654;
#define GPSBUFFER_MAX_LENGTH 28;
//servo configure
Servo motor, rudder, sail;
int motorMicroSec = 1500, motorMicroSecOld = 1500;
int motorDeadZone = 45;
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
int sailVelLimit = 10;
int sailLimit = 40;
// encoder
const byte CS = 22;
const byte CL = 23;
const byte DA1 = 24;
const byte DA2 = 25;


struct //total 2+10+8+12+30+2=60 bytes
{
  //header (total 2 bytes)
  byte header1;  // 1 bytes
  byte header2;  // 1 bytes

  //control part (total 10 bytes)
  int motorMicroSec; // 2 bytes
  int rudderAng;  // 2
  int sailAng;    // 2
  int readMark;   // 2
  int autoFlag;   // 2

  //encoder (total 8 bytes)
  float windAngRead;  //4
  float sailAngRead;  //4


  //AHRS data (total 12 bytes)
  float roll;   //4
  float pitch;  //4
  float yaw;    //4

  //GPS data (total 30 bytes)
  float UTC;   //4
  float lat;   //4
  float lon;   //4
  int FS;      //2
  float Hacc;  //4
  float SOG;   //4
  int ageC;    //2
  float HDOP;  //4
  int SVs;     //2

  //crc (total 2 bytes)
  unsigned int crcnum;  //2

} sensorData = {0};


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
  sensorData.header1 = 0x4f;
  sensorData.header2 = 0x5e;
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
  sensorData.sailAngRead = Angle_Cal(Angle_1_data, 0);
  sensorData.windAngRead = Angle_Cal(Angle_2_data, 0);
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


void serial2ReadStruct(int ahrsBufferSize) { //used for AHRS
  boolean headIsFound = 0;
  while (Serial1.available() > (2 * ahrsBufferSize - 1)) Serial1.read();
  while (Serial1.available()) {
    if ((int)Serial1.read() == 0xFF && (int)Serial1.read() == 0X02) {
      headIsFound = 1;
      break;
    }
  }
  while (headIsFound && Serial1.available() >= (ahrsBufferSize - 2)) {
    byte ahrsBuffer[ahrsBufferSize - 2];
    Serial1.readBytes(ahrsBuffer, (ahrsBufferSize - 2));
    byte crcField[ahrsBufferSize - 4]; //i.e. valid data length
    for (int i = 0; i < (ahrsBufferSize - 4); i++) crcField[i] = ahrsBuffer[i];
    unsigned int crcnum = CRC16(crcField, sizeof(crcField));
    byte *pAhrsBuffer = ahrsBuffer;
    unsigned int ahrsCRC = *((unsigned int*)(pAhrsBuffer + 15));
    if (crcnum == ahrsCRC) {
      sensorData.roll = *((float*)pAhrsBuffer + 3);
      sensorData.pitch = *((float*)(pAhrsBuffer + 7));
      sensorData.yaw = *((float*)(pAhrsBuffer + 11));
    }
  }
}

unsigned int CRC16(const byte *pBuffer, unsigned int bufferSize)
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

void serial3ReadStruct(int gpsBufferSize) {
  boolean headIsFound = 0;
  while (Serial2.available() > (2 * gpsBufferSize - 1)) Serial2.read();
  while (Serial2.available()) {
    //    Serial.println((int)Serial2.read());
    if ((int)Serial2.read() == 0x4f && (int)Serial2.read() == 0x5e) {
      headIsFound = 1;
      break;
    }
  }
  //  Serial.println(headIsFound);
  if (headIsFound && Serial2.available() >= (gpsBufferSize - 2)) {
    // Serial.println(Serial2.available());
    byte gpsBuffer[gpsBufferSize - 2]; //minus header 2 bytes
    Serial2.readBytes(gpsBuffer, gpsBufferSize - 2);
    byte crcField[gpsBufferSize - 4]; //minus header 2 bytes and crc 2 bytes,i.e valid data length
    for (int i = 0; i < (gpsBufferSize - 4); i++) crcField[i] = gpsBuffer[i];
    unsigned int crcnum = CRC16(crcField, sizeof(crcField));
    byte *pGpsBuffer = gpsBuffer;
    unsigned int gpsCRC = *((unsigned int*)(pGpsBuffer + 30));
    if (crcnum == gpsCRC) {
      sensorData.UTC = *((float*)pGpsBuffer);
      sensorData.lat = *((float*)(pGpsBuffer + 4));
      sensorData.lon = *((float*)(pGpsBuffer + 8));
      sensorData.FS = *((int*)(pGpsBuffer + 12));
      sensorData.Hacc = *((float*)(pGpsBuffer + 14));
      sensorData.SOG = *((float*)(pGpsBuffer + 18));
      sensorData.ageC = *((int*)(pGpsBuffer + 22));
      sensorData.HDOP = *((float*)(pGpsBuffer + 24));
      sensorData.SVs = *((int*)(pGpsBuffer + 28));
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
    motorMicroSec = map(motorMicroSec, 1100, 1900, 1300, 1700);
    int durRudd = pulseIn(ruddPin, HIGH);
    rudderAng = map(durRudd, 1100, 1900, 100, 180);
    int durThro = pulseIn(throPin, HIGH);
    sailAng = map(durThro, 1100, 1900, 100, 180);
  }
}


void veloLimit() {
  // motorMicroSec = constrain(motorMicroSec, motorMicroSecOld - motorVelLimit, motorMicroSecOld + motorVelLimit);

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


void structDataSend() {
  sensorData.motorMicroSec = motorMicroSec;
  sensorData.rudderAng = rudderAng - 140;
  sensorData.sailAng = sailAng - 100;
  sensorData.readMark = readMark;
  sensorData.autoFlag = autoFlag;

  byte *tobyte = (byte*)&sensorData;
  sensorData.crcnum = CRC16(tobyte + 2, sizeof(sensorData) - 4); //the valid data part as used to generate crc
  //  Serial.println(sizeof(sensorData));
  Serial.write(tobyte, sizeof(sensorData));
}


void flash() {
  serial1Read();
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
  structDataSend();
}

void loop() {
  encoderRead();
  serial2ReadStruct(19); //for AHRS
  serial3ReadStruct(34); //for GPS
  // delay(10);
}
