#include "my_flash.h"
#include <Preferences.h>
#include "SCServo.h"
#include "HardwareSerial.h"

int8_t servoDev[21];  // 在源文件中定义变量
uint16_t servoPOS[21];
extern SCSCL arm;
extern HardwareSerial Serial;
void setDev(int8_t *buf)
{
  uint8_t err=0;
  Preferences prefs;
  prefs.begin("devSpace");
  err=prefs.putBytes("servodev", buf, 21);
  prefs.end();
  if(err==0){
    Serial.println("set dev error!!!");
  }
}
void getDev(int8_t *buf)
{
  uint8_t err=0;
  Preferences prefs;
  prefs.begin("devSpace");
  err=prefs.getBytes("servodev", buf, 21);
  prefs.end();
  if(err==0){
    Serial.println("get dev error!!!");
  }
}
void getServoPos(uint16_t *buf)
{
  for (int i = 0; i < 21; i++)
  {
    buf[i] = map(arm.ReadPos(i + 1), 0, 1023, 0, 300);
    delay(10);
  }
}
void bootTask()
{
  Serial.begin(115200);
  Serial.println("===================================================");
  Serial.println("Into Boot,now you can set the servo dev");
  Serial.println("input \"GET POS\" to get the servo current position");
  Serial.println("input \"SET DEV\" to set the servo dev");
  Serial.println("input \"GET DEV\" to get the servo dev");
  Serial.println("input \"GIVE POWER\" to give power to all servo");
  Serial.println("===================================================");

  getServoPos(servoPOS);
  getDev(servoDev);
  while (1)
  {
    if (Serial.available())
    {
      String cmd = Serial.readStringUntil('\n');
      Serial.println("get cmd:"+cmd);
      cmd.trim();  // 移除字符串前后的空白字符
      if (cmd == "GET POS")
      {
        getServoPos(servoPOS);
        for (int i = 0; i < 21; i++)
        {
          Serial.printf("servo%dpos:%d ", i + 1, servoPOS[i]);
          if((i + 1) % 3 == 0) {
            Serial.println();
          }
        }
      }
      else if (cmd == "SET DEV")
      {
        getServoPos(servoPOS);
        for (int i = 0; i < 21; i++)
        {
          int16_t dev = (int16_t)servoPOS[i] - 150;
          if(dev == -150){
            servoDev[i] = 0;  // 某舵机检测错误，则设置为0
            Serial.printf("servo%d read back error\r", i+1);
          } else if(dev < -128 || dev > 127) {
            servoDev[i] = 0;  // 如果结果超出int8_t范围，设为0
            Serial.printf("servo%d deviation too large\r", i+1);
          } else {
            servoDev[i] = (int8_t)dev;
          }
        }
        setDev(servoDev);
      }
      else if (cmd == "GET DEV")
      {
        getDev(servoDev);
        for (int i = 0; i < 21; i++)
        {
          Serial.printf("servo%ddev:%d ", i + 1, servoDev[i]);
          if((i + 1) % 3 == 0) {
            Serial.println();
          }
        }
      }
      else if(cmd=="GIVE POWER"){
        for(int i=0;i<21;i++){
          arm.servoMove(i+1,150,1000);
        }
      }
      else 
      {
        Serial.println("command error");
        Serial.println("===================================================");
        Serial.println("input \"GET POS\" to get the servo current position");
        Serial.println("input \"SET DEV\" to set the servo dev");
        Serial.println("input \"GET DEV\" to get the servo dev");
        Serial.println("input \"GIVE POWER\" to give power to all servo");
        Serial.println("===================================================");
      }
    }
  }
}
