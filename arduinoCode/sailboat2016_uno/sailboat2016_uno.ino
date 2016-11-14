#include <Wire.h>

const float pi = 3.141592653589793;
int GPSAddress = 0x42;//GPS I2C Address
String gpsString = "";
double originLat = 31.0326 / 180.0 * pi, originLon = 121.44162 / 180.0 * pi;
double listNE[2];
double lat, lon;


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


int dataParser() {
  if (gpsString.startsWith("$GPGGA")) {
    int commaIndexList[20] = {0};
    int commaCount = 1;
    while (1) {
      int tmpIndex = gpsString.indexOf(',', commaIndexList[commaCount - 1] + 1);
      if (tmpIndex == -1) break;
      commaIndexList[commaCount] = tmpIndex;
      //      Serial.println(commaIndexList[commaCount]);
      commaCount ++;
    }
    // latitude
    String latStr = "";
    for (int i = commaIndexList[2] + 1; i < commaIndexList[3]; i++) {
      latStr += gpsString[i];
    }
    Serial.println("latStr: " + latStr);
    lat = latStr.toFloat();
    lat = dm2dd(lat);
    // longtitude
    String lonStr = "";
    for (int i = commaIndexList[4] + 1; i < commaIndexList[5]; i++) {
      lonStr += gpsString[i];
    }
    Serial.println("latStr: " + lonStr);
    lon = lonStr.toFloat();
    lon = dm2dd(lon);
    return 1;
  }
  else return 0;
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
  double m = 100*(tmp - d);
  return d + m / 60.0;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
}

// void test(int a[]) {
//   a[0] = 100;
// }

void loop()
{
  recvFromGps();
  Serial.println(gpsString);
  if (dataParser()) {
    w84ToNE(lat, lon, listNE);
    Serial.println(lat, 5);
    Serial.println(lon, 5);
    Serial.println(listNE[0], 4);
    Serial.println(listNE[1], 4);
  }
  //  char *buff[] = {"A","B"};
  //  *buff[0] = 'A';
  //  Serial.println(buff[0]);
  //  int b[2];
  //  test(b);
  //  Serial.println(b[0]);
  // int *p;
  // p[0]=1; p[1]=3;
  // Serial.println(*(p+1));
  //  String a = "-12.3";
  //  Serial.println(a.toFloat());
  //  delay(500);

}
