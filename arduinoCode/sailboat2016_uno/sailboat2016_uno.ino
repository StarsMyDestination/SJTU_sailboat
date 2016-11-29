#include <Wire.h>
#include <FlexiTimer2.h>

const float pi = 3.141592653589793;
int GPSAddress = 0x42;//GPS I2C Address
String gpsString = "";
double originLat = 31.0326 / 180.0 * pi, originLon = 121.44162 / 180.0 * pi;
double listNE[2];
double lat, lon;
int SVs, FS;
float HDOP, SOG, UTC;
boolean readFlag;


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
  Serial.println(headerIsFound);
}


void dataParser() {
  if (gpsString.startsWith("$GPGGA")) {
    int commaIndexList[20] = {0};
    int commaCount = 1;
    while (1) {
      int tmpIndex = gpsString.indexOf(',', commaIndexList[commaCount - 1] + 1);
      if (tmpIndex == -1) break;
      commaIndexList[commaCount] = tmpIndex;
      // Serial.println(commaIndexList[commaCount]);
      commaCount ++;
    }
    //UTC
    String UTCStr = "";
    for (int i = commaIndexList[1] + 1; i < commaIndexList[2]; i++) {
      UTCStr += gpsString[i];
    }
    UTC = UTCStr.toFloat();
    // latitude
    String latStr = "";
    for (int i = commaIndexList[2] + 1; i < commaIndexList[3]; i++) {
      latStr += gpsString[i];
    }
    //    Serial.println("latStr: " + latStr);
    lat = latStr.toFloat();
    lat = dm2dd(lat);
    // longtitude
    String lonStr = "";
    for (int i = commaIndexList[4] + 1; i < commaIndexList[5]; i++) {
      lonStr += gpsString[i];
    }
    //    Serial.println("latStr: " + lonStr);
    lon = lonStr.toFloat();
    lon = dm2dd(lon);
    // FS(Fix Status, 0 no fix; 1 Standard(2D/3D); 2 DGPS)
    String FSStr = "";
    for (int i = commaIndexList[6] + 1; i < commaIndexList[7]; i++) {
      FSStr += gpsString[i];
    }
    FS = FSStr.toInt();
    // SVs(satellites used, range 0-12)
    String SVsStr = "";
    for (int i = commaIndexList[7] + 1; i < commaIndexList[8]; i++) {
      SVsStr += gpsString[i];
    }
    SVs = SVsStr.toInt();
    // HDOP(Horizontal Dilution of Precision， range 0.5-99)
    String HDOPStr = "";
    for (int i = commaIndexList[8] + 1; i < commaIndexList[9]; i++) {
      HDOPStr += gpsString[i];
    }
    HDOP = HDOPStr.toFloat();
    readFlag = 1;
  }
  else if (gpsString.startsWith("$GPVTG")) {
    int commaIndexList[20] = {0};
    int commaCount = 1;
    while (1) {
      int tmpIndex = gpsString.indexOf(',', commaIndexList[commaCount - 1] + 1);
      if (tmpIndex == -1) break;
      commaIndexList[commaCount] = tmpIndex;
      commaCount ++;
    }
    // SOG(speed over ground m/s)
    String SOGStr = "";
    for (int i = commaIndexList[7] + 1; i < commaIndexList[8]; i++) {
      SOGStr += gpsString[i];
    }
    SOG = SOGStr.toFloat();
    SOG = SOG / 3.6;
    readFlag = 1;
  }
  else readFlag = 0;
}


double d2r(double d) {
  return d / 180.0 * pi;
}

void w84ToNE(double lat, double lon, double NE[]) {
  double d_lat = d2r(lat) - originLat;
  double d_lon = d2r(lon) - originLon;
  double a = 6378137.0;
  double e_2 = 6.69437999014e-3;
  double r1 = a * (1 - e_2) / pow((1 - e_2 * pow(sin(originLat), 2)), 1.5);
  double r2 = a / sqrt(1 - e_2 * pow(sin(originLat), 2));

  NE[0] = r1 * d_lat;
  NE[1] = r2 * cos(originLat) * d_lon;
}


double dm2dd(double data) {
  double tmp = data / 100.0;
  double d = floor(tmp);
  double m = 100 * (tmp - d);
  return d + m / 60.0;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}


struct
{
  float UTC;
  float north;
  float east;
  int FS;
  int SVs;
  float HDOP;
  float SOG;
} gpsData;


void dataAssignTest() {
  gpsData.UTC = 123.324;
  gpsData.north = 3243.5464;
  gpsData.east = 57849.4231;
  gpsData.FS = 2;
  gpsData.SVs = 10;
  gpsData.HDOP = 1.32;
  gpsData.SOG = 0.123;
}

void structDataSend() {
  byte *tobyte = (byte*)&gpsData;
  Serial.write(tobyte, sizeof(gpsData));
};



void dataSend() {
  Serial.print("#@");
  Serial.print(',');
  Serial.print(UTC);
  Serial.print(',');
  Serial.print(listNE[0], 2);
  Serial.print(',');
  Serial.print(listNE[1], 2);
  Serial.print(',');
  Serial.print(FS);
  Serial.print(',');
  Serial.print(SVs);
  Serial.print(',');
  Serial.print(HDOP);
  Serial.print(',');
  Serial.print(SOG);
  Serial.print(',');
  Serial.print("$*\n");
}

void flash() {
  //  dataSend();
  dataAssignTest();
  structDataSend();
//  Serial.println(sizeof(gpsData));
}

void loop()
{
  // recvFromGps();
  // // Serial.println(gpsString);
  // dataParser();
  // if (readFlag) {
  //   w84ToNE(lat, lon, listNE);
  // }

//  delay(500);
}
