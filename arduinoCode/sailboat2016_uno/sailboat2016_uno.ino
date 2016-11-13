#include <Wire.h>

int GPSAddress = 0x42;//GPS I2C Address
String gpsString = "";


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


// char headerIsFound(char header[7]) {
//   char i = 0;
//   char buff[7];
//   while (1)
//   {
//     recvInit();
//     while (Wire.available())
//     {
//       buff[i] = WireRead();
//       if (buff[i] == header[i]) //compare with header
//       {
//         i++;
//         if (i == 7)
//         {
//           Wire.endTransmission();//end transmission
//           return 1;
//         }
//       }
//       else
//         i = 0;
//     }
//     Wire.endTransmission();
//   }
// }


void recvFromGps() {
  gpsString = "";
  boolean headerIsFound = 0;
  while (1) {
    recvInit();
    while (Wire.available() && !headerIsFound) { 
      gpsString = String((char)Wire.read());
//      Serial.println(gpsString);
      if (gpsString == "$") {
        headerIsFound = 1;
//        Serial.println(111);
        break;
      }
    }
    if (headerIsFound) {
      Serial.println(2222);
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



void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("DFRobot DFRduino GPS Shield v1.0");
  Serial.println("$GPGGA statement information: ");
}
void loop()
{
  recvFromGps();
  Serial.println(gpsString);
  delay(500);
//String aa = '$';
//Serial.println(aa);
//delay(500);
}
