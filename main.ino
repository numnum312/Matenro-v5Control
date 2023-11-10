//mainプログラム
//タクトスイッチ入力でduty比変更
//加速度センサの値に応じてデバッグLEDの点灯数を変える

//Accelerometer related
#include <Wire.h>
#define MMA8452_ADRS 0x1D
 
#define MMA8452_OUT_X_MSB 0x01
#define MMA8452_XYZ_DATA_CFG 0x0E
#define MMA8452_CTRL_REG1 0x2A
#define MMA8452_CTRL_REG1_ACTV_BIT 0x01
#define MMA8452_G_SCALE 8 //change max range

//Timer Interrupt Processing
#ifdef ARDUINO_ARCH_MEGAAVR
#include "EveryTimerB.h"
#define Timer1 TimerB2 // use TimerB2 as a drop in replacement for Timer1
#else // assume architecture supported by TimerOne ....
#include "TimerOne.h"
#endif

//buffer for Accelermeter
byte buf[6];
float g[3];

//control motor
#define IN1 10
#define IN2 11
#define MAX_SPEED 153
#define LOW_SPEED 80 //min motor speed
int speed_5V[6] = {89,110,115,128,140,150};
int speed = LOW_SPEED;

//Switch
#define SW 6
#define MAX_COUNT 5 //6step gear sift
int count = 0; //switch counter
bool buttonState= LOW; //current state
bool old_buttonState = LOW; //old state

//LED
#define LED0 13  //on arduino LED
#define LED1 7
#define LED2 8
#define LED3 9


void setup() {
  //debug
  Serial.begin(115200);
  //main setup
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(SW,INPUT);
  buttonState = digitalRead(SW);

  //setting Accelerometer
  byte tmp;
  Wire.begin();
  tmp = MMA8452_ReadByte(MMA8452_CTRL_REG1);
  MMA8452_WriteByte(MMA8452_CTRL_REG1, tmp & ~(MMA8452_CTRL_REG1_ACTV_BIT));
  MMA8452_WriteByte(MMA8452_XYZ_DATA_CFG, (MMA8452_G_SCALE >> 2));
  tmp = MMA8452_ReadByte(MMA8452_CTRL_REG1);
  MMA8452_WriteByte(MMA8452_CTRL_REG1, tmp | MMA8452_CTRL_REG1_ACTV_BIT);
}

void loop() {
  //SW control
  buttonState = digitalRead(SW);
  if(buttonState == HIGH && old_buttonState == LOW){
    count++;
    speed = speed_5V[count];
    if(count > MAX_COUNT){
      count = 0;
      speed = speed_5V[count];
    }
  }

  //Motor control
  if(speed > MAX_SPEED){speed = MAX_SPEED;}
  analogWrite(IN2,0);
  analogWrite(IN1,speed);

  //Acceleration Sensor Control
  //この下タイマー割込み処理したい
  MMA8452_ReadByteArray(MMA8452_OUT_X_MSB, 6, buf);
  g[0] = -(float((int((buf[0] << 8) | buf[1]) >> 4)) / ((1 << 11) / MMA8452_G_SCALE));
  g[1] = -(float((int((buf[2] << 8) | buf[3]) >> 4)) / ((1 << 11) / MMA8452_G_SCALE));
  g[2] = -(float((int((buf[4] << 8) | buf[5]) >> 4)) / ((1 << 11) / MMA8452_G_SCALE));

  float sum_data = 0;
  sum_data += pow(g[0],2);
  sum_data += pow(g[1],2);
  sum_data += pow(g[2],2);
  sum_data -= 0.981;

  //加速度によってLED点灯
  if(sum_data < 2.0){
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
  }
  else if(sum_data >= 2.0 && sum_data < 4.0){
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,HIGH);
    digitalWrite(LED3,LOW);
  }
  else if(sum_data >= 4.0){
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,HIGH);
    digitalWrite(LED3,HIGH);
  }
  else{
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
  }

  //Update SW Status
  old_buttonState = buttonState;

  //debug
  Serial.print("state->");
  Serial.println(count);
  //speed
  Serial.print("speed->");
  Serial.println(speed);
  //加速度
  Serial.print(sum_data, 4);
  Serial.println("\t");
  delay(30);
}

//############################################################
//############################################################
//control Accelerometer Function
void MMA8452_ReadByteArray(byte adrs, int datlen, byte *dest){
  Wire.beginTransmission(MMA8452_ADRS);
  Wire.write(adrs);
  Wire.endTransmission(false);
 
  Wire.requestFrom(MMA8452_ADRS, datlen);
 
  while(Wire.available() < datlen);
 
  for(int x = 0; x < datlen; x++){
    dest[x] = Wire.read();
  }
}
 
byte MMA8452_ReadByte(byte adrs){
  Wire.beginTransmission(MMA8452_ADRS);
  Wire.write(adrs);
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8452_ADRS, 1);
  
  while(!Wire.available());
  Serial.println("read byte test");
  return(Wire.read());
}
 
void MMA8452_WriteByte(byte adrs, byte dat){
  Wire.beginTransmission(MMA8452_ADRS);
  Wire.write(adrs);
  Wire.write(dat);
  Wire.endTransmission();
}