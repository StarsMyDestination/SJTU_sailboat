
#include <Wire.h>
 
#define BUFFER_LENGTH 10//Define the buffer length
 
int GPSAddress = 0x42;//GPS I2C Address
 
double Datatransfer(char *data_buf,char num)//Data type converter：convert char type to float
{                                           //*data_buf:char data array ;num:float length
  double temp=0.0;
  unsigned char i,j;
 
  if(data_buf[0]=='-')//负数的情况
  {
    i=1;
    //数组中的字符型数据转换成整数并累加
    while(data_buf[i]!='.')
      temp=temp*10+(data_buf[i++]-0x30);
    for(j=0;j<num;j++)
      temp=temp*10+(data_buf[++i]-0x30);
    //将转换后的整数转换成浮点数
    for(j=0;j<num;j++)
      temp=temp/10;
    //转换成负数
    temp=0-temp;
  }
  else//正数情况
  {
    i=0;
    while(data_buf[i]!='.')
      temp=temp*10+(data_buf[i++]-0x30);
    for(j=0;j<num;j++)
      temp=temp*10+(data_buf[++i]-0x30);
    for(j=0;j<num;j++)
      temp=temp/10 ;
  }
  return temp;
}
void rec_init()//initial GPS
{
  Wire.beginTransmission(GPSAddress);
  WireSend(0xff);//发送数据所在的地址      
  Wire.endTransmission(); 
 
  Wire.beginTransmission(GPSAddress);
  Wire.requestFrom(GPSAddress,10);//要求从GPS器件读取10个字节
}
char ID()//接收语句的ID
{
  char i = 0;
  char value[7]={
    '$','G','P','G','G','A',','      };//要接收的GPS语句的ID内容
  char buff[7]={
    '0','0','0','0','0','0','0'      };
 
  while(1)
  {
    rec_init();//接收数据初始化    
    while(Wire.available())   
    { 
      buff[i] = WireRead();//接收串口的数据  
      if(buff[i]==value[i])//对比是否是正确的ID
      {
        i++;
        if(i==7)
        {
          Wire.endTransmission();//结束接收
          return 1;//接收完毕返回1
        }
      }
      else
        i=0;
    }
    Wire.endTransmission();//结束接收
  }
}
void UTC()//获取时间信息
{
  char i = 0,flag=0;
  char value[7]={
    '$','G','P','G','G','A',','   };
  char buff[7]={
    '0','0','0','0','0','0','0'       };
  char time[9]={
    '0','0','0','0','0','0','0','0','0'    };//存放时间数据
  double t=0.0;
 
  while(1)
  {
    rec_init();    
    while(Wire.available())   
    { 
      if(!flag)
      { 
        buff[i] = WireRead();
        if(buff[i]==value[i])
        {
          i++;
          if(i==7)
          {
            i=0;
            flag=1;
          }
        }
        else
          i=0;
      }
      else
      {
        time[i] = WireRead();
        i++;
        if(i==9)
        {
          t=Datatransfer(time,2);//转换成浮点型数据
          t=t+80000.00;//将时间转换成北京时间
          Serial.println(t);//输出时间数据 
          Wire.endTransmission();
          return;
        }
      }
    }
    Wire.endTransmission(); 
  }
}
void rec_data(char *buff,char num1,char num2)//接收数据子函数
{                                            //*buff：存放接收数据的数组；num1：逗号数目；num2：数组长度。
  char i=0,count=0;
 
  if(ID())
  {
    while(1)
    {
      rec_init();    
      while(Wire.available())   
      { 
        buff[i] = WireRead();
        if(count!=num1)
        {  
          if(buff[i]==',')
            count++;
        }
        else
        {
          i++;
          if(i==num2)
          {
            Wire.endTransmission();
            return;
          }
        }
      }
      Wire.endTransmission();
    }
  }
}
void latitude()//获取纬度信息
{
  char lat[10]={
    '0','0','0','0','0','0','0','0','0','0' };//存放纬度数据
  rec_data(lat,1 ,10);//接收纬度数据
  Serial.println(Datatransfer(lat,5),5);//将纬度数据转换成浮点型数据并输出
}
void lat_dir()//获取纬度方向信息
{
  char dir[1]={'0'};//存放纬度方向数据
  rec_data(dir,2,1);//接收纬度方向数据
  printlnByte(dir[0]);//将纬度方向信息输出
}
void  longitude()//获取经度信息
{
  char lon[11]={
    '0','0','0','0','0','0','0','0','0','0','0' };//存放经度数据
  rec_data(lon,3,11);//接收经度数据
  Serial.println(Datatransfer(lon,5),5);//将经度数据转换成浮点型数据并输出
}
void lon_dir()//获取经度方向信息
{
  char dir[1]={'0'};
  rec_data(dir,4,1);
  printlnByte(dir[0]);//将纬度方向信息输出
}
void altitude()//获取海拔信息
{
  char i=0,count=0;
  char alt[8]={
    '0','0','0','0','0','0','0','0' };
 
  if(ID())
  {
    while(1)
    {
      rec_init();    
      while(Wire.available())   
      { 
        alt[i] = WireRead();
        if(count!=8)
        {  
          if(alt[i]==',')
            count++;
        }
        else
        {
          if(alt[i]==',')
          {
            Serial.println(Datatransfer(alt,1),1);
            Wire.endTransmission();
            return;
          }
          else
            i++;
        }
      }
      Wire.endTransmission();
    }
  }
}
void setup()
{
  Wire.begin();//IIC初始化
  Serial.begin(9600);//设置波特率
  Serial.println("DFRobot DFRduino GPS Shield v1.0");
  Serial.println("$GPGGA statement information: ");
}
void loop()
{
  while(1)
  {
    Serial.print("UTC:");
    UTC();
    Serial.print("Lat:");
    latitude();
    Serial.print("Dir:");
    lat_dir();
    Serial.print("Lon:");
    longitude();
    Serial.print("Dir:");
    lon_dir();
    Serial.print("Alt:");
    altitude();
    Serial.println(' ');
    Serial.println(' ');
  }
}
