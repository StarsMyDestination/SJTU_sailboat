#include <Wire.h>

const float pi = 3.141592653589793;
int GPSAddress = 0x42;//GPS I2C Address
String gpsString = "";
double originLat = 31.0326 / 180.0 * pi, originLon = 121.44162 / 180.0 * pi;
double listNE[2];


double Datatransfer(char *data_buf, char num) //Data type converter：convert char type to float
{ //*data_buf:char data array ;num:float length
  double temp = 0.0;
  unsigned char i, j;

  if (data_buf[0] == '-') //negative number
  {
    i = 1;
    //char to int and add
    while (data_buf[i] != '.')
      temp = temp * 10 + (data_buf[i++] - 0x30);
    for (j = 0; j < num; j++)
      temp = temp * 10 + (data_buf[++i] - 0x30);
    //int to float
    for (j = 0; j < num; j++)
      temp = temp / 10;
    //to negative number
    temp = 0 - temp;
  }
  else//positive number
  {
    i = 0;
    while (data_buf[i] != '.')
      temp = temp * 10 + (data_buf[i++] - 0x30);
    for (j = 0; j < num; j++)
      temp = temp * 10 + (data_buf[++i] - 0x30);
    for (j = 0; j < num; j++)
      temp = temp / 10 ;
  }
  return temp;
}


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

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("DFRobot DFRduino GPS Shield v1.0");
  Serial.println("$GPGGA statement information: ");
}

void loop()
{
  //  recvFromGps();
  //  Serial.println(gpsString);
  w84ToNE(31.0326, 121.44169, listNE);
  Serial.println(listNE[0], 8);
  Serial.println(listNE[1], 8);
  delay(1000);

  //String aa = '$';
  //Serial.println(aa);
  //delay(500);
}
