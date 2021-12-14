/*
使用SBUS协议控制
该版本完成：读取SBUS协议数据，完成对车辆底盘的前后左右移动控制。
*/
#include <Servo.h>
# define DATA_LEN 35
# define CHANNEL 16
# define DEADZONE 20
int channelData[CHANNEL] = {0};


Servo motorLeft;  // 左轮电机
Servo motorRight;  // 右轮电机

int motorLeft_output_pin = 9;        //左轮电机输出
int motorRight_output_pin = 7;        //右轮电机输出

int armMotor_enable_output_pin = 8;  //摆臂电机使能信号输出引脚（低电平使能）


void setup()
{
  pinMode(armMotor_enable_output_pin,OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  motorLeft.attach(motorLeft_output_pin); //设置pwm输出口
  motorRight.attach(motorRight_output_pin);

    /*定时器产生不同占空比的PWM波*/
  OCR1B=128;
  OCR1C=128;

  DDRB=0xFF;
  PORTB=0xFF;
  ICR1=256;                                        //16位
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |=_BV(WGM11)|_BV(COM1A0)|_BV(COM1A1)|_BV(COM1B0)|_BV(COM1B1)|_BV(COM1C0)|_BV(COM1C1);
  TCCR1B |=_BV(WGM13)|_BV(WGM12)|_BV(CS10);           //FastPWM模式4，不分频
}

void loop()
{
  ReadSBUS();
  if(channelData[9] > 1600)  //遥控器使能
  {
     track_drive(channelData[1],channelData[3],channelData[2]);//底盘控制
     
     if(channelData[8] > 1600) //前摆臂使能
       arm_drive(channelData[4],channelData[7]);//摆臂控制   channelData[4]为前摆臂 channelData[7]为后摆臂
  }
  else if(Serial.available() > 0)    //上位机使能
  {}
  else    //无控制源
  {
    motorLeft.writeMicroseconds(1500);
    motorRight.writeMicroseconds(1500);
    digitalWrite(armMotor_enable_output_pin,HIGH);//摆臂电机不使能
    OCR1C=128;
    OCR1B=128;
   }
  
}

/*
函数名：arm_drive
功能：摆臂的驱动
输入：前摆臂的方向量（1000~2000），后摆臂的方向量（1000~2000）
*/
void arm_drive(int front_arm, int back_arm)     //前后摆臂控制
{
  digitalWrite(armMotor_enable_output_pin,LOW); //摆臂电机使能
  
  if (front_arm<450) OCR1C=90;        // 前摆臂前动 输入65%占空比PWM波
  else if (front_arm>1600) OCR1C=166;  //前摆臂后动 输入35%占空比PWM波
  else OCR1C=128;                      //前摆臂不动

  if (back_arm<450) OCR1B=90;       //后摆臂前动 输入65%占空比PWM波
  else if (back_arm>1600) OCR1B=166; //后摆臂后动  输入35%占空比PWM波
  else OCR1B=128;                    //后摆臂不动      
}

/*
*函数名：track_drive
*功能：履带的驱动
*输入：channelData[1]  //前进   1700~1024~352
*     channelData[3]  //转弯   1700~1024~352
*     channelData[2]  //油门   1696~1024~356
*/
void track_drive(int linear_vel, int angular_vel, int SpeedGain) 
{
  int Vl=1500;   //左轮
  int Vr=1500;   //右轮
  float k = 0.5; //转动参数k
  
  float V_norm = DataMap(linear_vel,352,1700,-1,1);
  float w_norm = DataMap(angular_vel,352,1700,-1,1);
  float SpeedGain_norm = DataMap(SpeedGain,356,1696,0,1);
  V_norm *= SpeedGain_norm;
  w_norm *= SpeedGain_norm;

  int v = DataMap(V_norm,-1,1,-500,500);
  int w = DataMap(w_norm,-1,1,-500,500);
  
//  MotionModel(v,w,k);
  Vl = (v+k*w);
  Vr = (v-k*w);
  
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

///*
// *函数名：MotionModel 
// *输入：平移速度v，角速度w，转动参数k
// *作用：将控制器的前进转向映射到底盘的左右电机上
// *输出：空
//*/
//void MotionModel(int v_,int w_,float k)
//{
//  Vl = (v_+k*w_);
//  Vr = (v_-k*w_);
//}
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
