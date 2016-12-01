#include <Wire.h>
#include <FlexiTimer2.h>

const float pi = 3.141592653589793;
int GPSAddress = 0x42;//GPS I2C Address
String gpsString = "";
double originLat = 31.0326 / 180.0 * pi, originLon = 121.44162 / 180.0 * pi;
double lat, lon;
boolean readFlag;


struct
{ //total 2+(4*6+2*3)+2= 2+ 30 +2 = 34 bytes
  byte header1; // 1
  byte header2; // 1

  float UTC;   //4
  float lat;   //4
  float lon;   //4
  int FS;      //2
  float Hacc;  //4
  float SOG;   //4
  int ageC;    //2
  float HDOP;  //4
  int SVs;     //2

  unsigned int crcnum;  //2
} gpsData = {0};


void recvInit() {
  Wire.beginTransmission(GPSAddress);
  Wire.write(0xff);// sends instruction byte
  Wire.endTransmission();
  Wire.beginTransmission(GPSAddress);
  Wire.requestFrom(GPSAddress, BUFFER_LENGTH);
}


void recvFromGps() {
  gpsString = "";
  boolean headerIsFound = 0;
  while (1) {
    recvInit();
    while (Wire.available() && !headerIsFound) { //this while() used for finding header
      gpsString = String((char)Wire.read());
      if (gpsString == "$") {
        headerIsFound = 1;
        break;
      }
    }
    if (headerIsFound) {
      while (Wire.available()) {
        gpsString += (char)Wire.read();
        if (gpsString.endsWith("\r\n")) {
          Wire.endTransmission();
          return;
        }
      }
    }
    Wire.endTransmission();
  }
}


void dataParser() {
  if (gpsString.startsWith("$PUBX")) {
    int commaIndexList[30] = {0};
    int commaCount = 1;
    while (1) {
      int tmpIndex = gpsString.indexOf(',', commaIndexList[commaCount - 1] + 1);
      if (tmpIndex == -1) break;
      commaIndexList[commaCount] = tmpIndex;
      //      Serial.println(commaIndexList[commaCount]);
      commaCount ++;
    }
    //UTC
    String UTCStr = "";
    for (int i = commaIndexList[2] + 1; i < commaIndexList[3]; i++) {
      UTCStr += gpsString[i];
    }
    gpsData.UTC = UTCStr.toFloat();
    // latitude
    String latStr = "";
    for (int i = commaIndexList[3] + 1; i < commaIndexList[4]; i++) {
      latStr += gpsString[i];
    }
    //    Serial.println("latStr: " + latStr);
    gpsData.lat = dm2dd(latStr.toFloat());
    // longtitude
    String lonStr = "";
    for (int i = commaIndexList[5] + 1; i < commaIndexList[6]; i++) {
      lonStr += gpsString[i];
    }
    //    Serial.println("latStr: " + lonStr);
    gpsData.lon = dm2dd(lonStr.toFloat());
    // FS(Fix Status, 0 no fix; 1 Standard(2D/3D); 2 DGPS)
    String FSStr = "";
    for (int i = commaIndexList[8] + 1; i < commaIndexList[9]; i++) {
      FSStr += gpsString[i];
    }
    if (FSStr == "G2" || FSStr == "G3")
      gpsData.FS = 1;
    else if (FSStr == "D2" || FSStr == "D3")
      gpsData.FS = 2;
    else gpsData.FS = 0 ;
    // Hacc(Horizontal accuracy estimate)
    String HaccStr = "";
    for (int i = commaIndexList[9] + 1; i < commaIndexList[10]; i++) {
      HaccStr += gpsString[i];
    }
    gpsData.Hacc = HaccStr.toFloat();
    // SOG(speed over ground m/s)
    String SOGStr = "";
    for (int i = commaIndexList[11] + 1; i < commaIndexList[12]; i++) {
      SOGStr += gpsString[i];
    }
    gpsData.SOG = SOGStr.toFloat() / 3.6;
    // ageC(Age of most recent DGPS corrections, empty = noneavailable)
    String ageCStr = "";
    for (int i = commaIndexList[14] + 1; i < commaIndexList[15]; i++) {
      ageCStr += gpsString[i];
    }
    gpsData.ageC = ageCStr.toInt();
    // HDOP(Horizontal Dilution of Precision， range 0.5-99)
    String HDOPStr = "";
    for (int i = commaIndexList[15] + 1; i < commaIndexList[16]; i++) {
      HDOPStr += gpsString[i];
    }
    gpsData.HDOP = HDOPStr.toFloat();
    // SVs(satellites used, range 0-12)
    String SVsStr = "";
    for (int i = commaIndexList[18] + 1; i < commaIndexList[19]; i++) {
      SVsStr += gpsString[i];
    }
    gpsData.SVs = SVsStr.toInt();

    readFlag = 1;
  }
  else readFlag = 0;
}


// double d2r(double d) {
//   return d / 180.0 * pi;
// }

// void w84ToNE(double lat, double lon) {
//   double d_lat = d2r(lat) - originLat;
//   double d_lon = d2r(lon) - originLon;
//   double a = 6378137.0;
//   double e_2 = 6.69437999014e-3;
//   double r1 = a * (1 - e_2) / pow((1 - e_2 * pow(sin(originLat), 2)), 1.5);
//   double r2 = a / sqrt(1 - e_2 * pow(sin(originLat), 2));

//   gpsData.north = r1 * d_lat;
//   gpsData.east = r2 * cos(originLat) * d_lon;
// }


double dm2dd(double data) {
  double tmp = data / 100.0;
  double d = floor(tmp);
  double m = 100 * (tmp - d);
  return d + m / 60.0;
}

//void dataAssignTest() {
//  gpsData.UTC = 123.324;
//  gpsData.northlat = 3243.5464;
//  gpsData.east = 57849.4231;
//  gpsData.FS = 2;
//  gpsData.SVs = 10;
//  gpsData.HDOP = 1.32;
//  gpsData.SOG = 0.123;
//}

unsigned int crc16(const byte *pBuffer, unsigned int bufferSize) {
  unsigned int poly = 0x8408;
  unsigned int crc = 0x0;
  byte carry;
  byte i_bits;
  unsigned int j;
  for (j = 0; j < bufferSize; j++) {
    crc = crc ^ pBuffer[j];
    for (i_bits = 0; i_bits < 8; i_bits++) {
      carry = crc & 1;
      crc = crc / 2;
      if (carry)
        crc = crc ^ poly;
    }
  }
  return crc;
}

void structDataSend() {
  //  Serial.println(sizeof(gpsData));
  byte *tobyte = (byte*)&gpsData;
  gpsData.crcnum = crc16(tobyte + 2, sizeof(gpsData) - 4); //the valid data part as used to generate crc
  Serial.write(tobyte, sizeof(gpsData));
}


void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  gpsData.header1 = 0x4f;
  gpsData.header2 = 0x5e;
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}

void flash() {
  // dataAssignTest();
  if (readFlag)
    structDataSend();
}

void loop()
{
  recvFromGps();
  //  Serial.println(gpsString);
  dataParser();
  //  Serial.println(gpsData.lat);
  // if (readFlag) {
  //   w84ToNE(lat, lon);
  // }
  //  Serial.println(111);
}
