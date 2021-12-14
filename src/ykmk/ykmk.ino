/*
使用SBUS协议控制
该版本完成：读取SBUS协议数据，完成对车辆底盘的前后左右移动控制。
*/
#include <Servo.h>
# define DATA_LEN 35
# define CHANNEL 16
# define DEADZONE 20
int channelData[CHANNEL] = {0};
float k = 0.5;

Servo motorLeft;  // 左轮电机
Servo motorRight;  // 右轮电机

int motorLeft_output_pin = 9;        //左轮电机输出
int motorRight_output_pin = 7;        //右轮电机输出
int Vl=1500;   //左轮
int Vr=1500;   //右轮

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    motorLeft.attach(motorLeft_output_pin); //设置pwm输出口
    motorRight.attach(motorRight_output_pin);
}

void loop()
{
  ReadSBUS();
//  channelData[1] = (data[3]<<8) + data[4];   //前进   1700~1024~352
//  channelData[3] = (data[7]<<8) + data[8];   //转弯   1700~1024~352
//  channelData[2]                             //油门   1696~1024~356
  
  float V_norm = DataMap(channelData[1],352,1700,-1,1);
  float w_norm = DataMap(channelData[3],352,1700,-1,1);
  float SpeedGain_norm = DataMap(channelData[2],356,1696,0,1);
  V_norm *= SpeedGain_norm;
  w_norm *= SpeedGain_norm;

  int v = DataMap(V_norm,-1,1,-500,500);
  int w = DataMap(w_norm,-1,1,-500,500);
  
  MotionModel(v,w,k);
  
  Vl = ValLimit(DeadZone(Vl,DEADZONE,0)+1500,1000,2000);
  Vr = ValLimit(DeadZone(Vr,DEADZONE,0)+1500,1000,2000);
  
  Serial.println(Vl);
  motorLeft.writeMicroseconds(Vl);
  motorRight.writeMicroseconds(Vr);
}


/*
 * 函数名：ReadSBUS
 * 输入值：空
 * 作用：读取SBUS协议中的数据，并将16路的通道数据存储在数组channelData中 
 * 返回值：空
*/
void ReadSBUS()
{
  int flag,i= 0;
  unsigned char XOR=0;
  unsigned char temp;
  unsigned char data[DATA_LEN];
//  unsigned char data_[DATA_LEN];
  while(1)
  {
    if(Serial1.available())   // 一共读入35个字节,存到rec之中
    {
      temp = Serial1.read();
      if ((temp == 0x0F)||(flag == 1))
      {
        data[i] = temp;
        if(i == 1) XOR = data[1];
        else if(i != (DATA_LEN-1)) XOR = XOR ^ temp; 
        i++;
        flag = 1;
      }
      if(i == DATA_LEN)
      {
        i=0;
        flag=0;
        if(XOR==(data[DATA_LEN-1]))
        {
          for(int j = 0; j < 16 ; ++j)
            {channelData[j] = ( data[2*j+1] <<8 ) + data[2*j+2];}
          return;
        }
      }
    }
  }
}

/*
 * 函数名：DataMap
 * 输入：映射前的数据FrontData，映射前的下限FrontLow，映射前的上限FrontHigh，映射后的下限AfterLow，映射后的上限AfterHigh
 * 函数作用：按照上下限将待映射的数据线性映射
 * 返回值：映射后的数据AfterData
*/
float DataMap(int FrontData,float FrontLow,float FrontHigh,float AfterLow,float AfterHigh)
{
  return (float)((AfterHigh-AfterLow)*(FrontData-FrontLow)/(FrontHigh-FrontLow)+AfterLow);
}
int DataMap(float FrontData,float FrontLow,float FrontHigh,float AfterLow,float AfterHigh)
{
  return (AfterHigh-AfterLow)*(FrontData-FrontLow)/(FrontHigh-FrontLow)+AfterLow;
}

/*
 * 函数名：DeadZone
 * 输入：通道脉宽channelData，死区阈值deadzone,零位zero
 * 功能：设置死区
 * 输出：设置有死区的通道脉宽
*/
int DeadZone(int channelData,int deadzone,int zero)
{
  if (abs(channelData-zero)<deadzone) channelData = zero;
  else
  {
    if (channelData > zero) channelData -= DEADZONE;
    if (channelData < zero) channelData += DEADZONE;
  }
  return channelData;
}

/*
 *函数名：MotionModel 
 *输入：平移速度v，角速度w，转动参数k
 *作用：将控制器的前进转向映射到底盘的左右电机上
 *输出：空
*/
void MotionModel(int v_,int w_,float k)
{
  Vl = (v_+k*w_);
  Vr = (v_-k*w_);
}
/*
 * 函数名：ValLimit
 * 输入：待限幅的数据data，下限Low，上限High
 * 作用：限幅
 * 输出：data
*/
int ValLimit(int data_, int Low, int High)
{
  if(data_<Low) data_=Low;
  if(data_>High) data_=High;
  return data_;
  }
