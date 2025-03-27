#include <Arduino.h>
#include <Sentry.h>
#include <Wire.h>
#include "LegControl_task.h"
#include "SCServo.h"
#include "ESP32Servo.h"
#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#include "my_flash.h"
typedef Sentry2 Sentry;
// #define VISION_TYPE Sentry::kVisionBlob
uint8_t VISION_TYPE = Sentry::kVisionBlob;
Sentry sentry;
sentry_object_t param = {0};
uint8_t w, h, x, y, z, l;
TaskHandle_t myTaskHandle;
bool taskFlag = 0;
Servo servo1;
Servo servo2;
UserMpu Mpu;
// 全局变量
Gait_prg gait_prg; // 步态规划
SCSCL arm;
Hexapod hexapod(arm, gait_prg);                            // 机器人结构体
uint32_t round_time;                                       // 回合时间
static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
void myTask(void *parameter)
{
  while (1)
  {
    if (sentry.GetValue(VISION_TYPE, kStatus))
    {
      w = sentry.GetValue(VISION_TYPE, kWidthValue, 1);
      h = sentry.GetValue(VISION_TYPE, kHeightValue, 1);
      x = sentry.GetValue(VISION_TYPE, kXValue, 1);
      y = sentry.GetValue(VISION_TYPE, kYValue, 1);
      l = sentry.GetValue(VISION_TYPE, kLabel, 1);
      z = 1;
    }
    else
    {
      w = h = x = y = l = z = 0;
    }
    if (taskFlag == 1)
    {
      vTaskDelay(1000);
      w = h = x = y = l = z = 0;
      taskFlag = 0;
    }
  }
}
void hexapodMove()
{
  code_time_start = millis(); // 获取当前时间
  if (hexapod.velocity.omega >= 0)
    LegControl_round = (++LegControl_round) % N_POINTS; // 控制回合自增长
  else
  {
    if (LegControl_round == 0)
      LegControl_round = N_POINTS - 1;
    else
      LegControl_round--;
  }
  /*步态控制*/
  gait_prg.CEN_and_pace_cal();
  gait_prg.gait_proggraming();
  /*开始移动*/
  round_time = gait_prg.get_pace_time() / N_POINTS;
  hexapod.move(round_time);
  // delay(round_time);
  // 计算程序运行时间
  code_time_end = millis();                    // 获取当前时间
  code_time = code_time_end - code_time_start; // 做差获取程序运行时间（8ms）
  if (code_time < round_time)
  {
    delay(round_time - code_time); // 保证程序执行周期等于回合时间
  }
  else
    delay(10); // 至少延时1ms
}

void hexapodStop()
{
  for (int i = 0; i < 10; i++)
  {
    hexapod.body_angle_and_pos_zero(); // 停止
    hexapod.velocity_cal(0, 0, 0);
    hexapod.body_angle_cal(0, 0, 0);
    hexapod.body_position_cal(0, 0, 0);
    hexapodMove();
  }
}

void findBlock(uint8_t xMin, uint8_t xMax, uint8_t yMin, uint8_t yMax)
{
  while (!(x < xMax && x > xMin && y < yMax && y > yMin))
  {
    int xVelocity = (x < xMin) ? (x - xMin) / 1.5 : (x > xMax) ? (x - xMax) / 1.5
                                                               : 0;
    int yVelocity = (y < yMin) ? (yMin - y) / 1.5 : (y > yMax) ? (yMax - y) / 1.5
                                                               : 0;
    if (xVelocity == 0 && yVelocity == 0)
      break;
    // 限制xVelocity和yVelocity的绝对值不小于10
    if (abs(xVelocity) < 15 && xVelocity != 0)
      xVelocity = (xVelocity > 0) ? 15 : -15;
    if (abs(yVelocity) < 15 && yVelocity != 0)
      yVelocity = (yVelocity > 0) ? 15 : -15;
    hexapod.velocity_cal(xVelocity, yVelocity, 0);
    hexapodMove();
  }
}
void seekBlock(int x_min, int x_max, int y_min, int y_max)
{
  float posX = 1, posY = 1;
  int posZ = 1;
  uint32_t hour = millis();
  // 当接收到的数据不在特定范围内时，执行循环
  while (!(x < x_max && x > x_min && y < y_max && y > y_min))
  {
    // 限制posX和posY的值在-256到256之间
    value_limit(posX, -256, 256);
    value_limit(posY, -256, 256);
    // 根据接收到的xValue和yValue的值，更新posX和posY
    posX += (x < x_min) ? -4 : (x > x_max) ? 4
                                           : 0;
    posY += (y < y_min) ? 4 : (y > y_max) ? -4
                                          : 0;
    // 计算机器人的身体位置
    hexapod.body_position_cal(posX, posY, posZ);
    // 移动机器人
    hexapodMove();
    if (millis() - hour > 6000)
    {
      break;
    }
  }
}
void turnForward()
{
  int angleTemp = Mpu.mpu_Get_angle('x');
  while (!(angleTemp > -2 && 2 > angleTemp))
  {
    if (-2 > angleTemp)
    {
      hexapod.velocity_cal(0, 0, (-60));
    }
    if (angleTemp > 2)
    {
      hexapod.velocity_cal(0, 0, 60);
    }
    hexapodMove();
    angleTemp = Mpu.mpu_Get_angle('x');
    if (angleTemp == -2 || angleTemp == 2)
    {
      break;
    }
    // Serial.println(angleTemp);
  }
}

void setup()
{

  // PS4.begin("ba:ba:ca:ea:02:01");
  // uint8_t pairedDeviceBtAddr[20][6];
  // int count = esp_bt_gap_get_bond_device_num();
  // esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  // for (int i = 0; i < count; i++)
  // {
  //   esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  // }

  // servo1.attach(14);
  // servo2.attach(15);
  // sentry_err_t err = SENTRY_OK;
  // Wire.begin();
  // while (SENTRY_OK != sentry.begin(&Wire))
  // {
  // }
  // xTaskCreatePinnedToCore(myTask, "My Task", 4096, NULL, 1, &myTaskHandle, 1);
  // sentry.SetParamNum(VISION_TYPE, 1);
  // taskFlag = 1;
  // param.width = 3;
  // param.height = 4;
  // param.label = 3;
  // sentry.SetParam(VISION_TYPE, &param, 1);
  // sentry.VisionBegin(VISION_TYPE);
  // vTaskResume(myTaskHandle);
  Serial1.begin(500000, SERIAL_8N1, 18, 19);
  arm.pSerial = &Serial1;
  Serial.begin(115200);
  gait_prg.Init();
  bootTask();
  // Mpu.mpu_Init();
  hexapod.Init(0);
  arm.servoMove(20, 20, 1000);
  arm.servoMove(21, 50, 1000);
  // arm.SetID(16);
  delay(1000);
}
void loop()
{
  hexapod.velocity_cal(0, 60, 0);
  hexapodMove();
}
