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
uint32_t preTime;
uint32_t curTime;
uint16_t pos19 = 150;
uint16_t pos20 = 150;
uint16_t pos21 = 150;
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
void takeBall()
{
  arm.servoMove(21, 160, 500); // 张开夹爪
  arm.servoMove(20, 100, 1000);
  delay(1000);
  arm.servoMove(19, 240, 1000);

  delay(1000);

  arm.servoMove(21, 60, 500);
  delay(500);
  arm.servoMove(19, 150, 1000);
  delay(1000);
  arm.servoMove(20, 15, 1000);
  delay(1000);
}
void putBall()
{
  arm.servoMove(20, 130, 1000);
  delay(1000);
  arm.servoMove(19, 210, 1000);
  delay(1000);
  arm.servoMove(21, 160, 500); // 张开夹爪
  delay(500);
  arm.servoMove(19, 150, 1000);
  delay(1000);
  arm.servoMove(20, 15, 1000);
  arm.servoMove(21, 60, 500); // 收起夹爪
  delay(1000);
}

void setup()
{
  // arm.setDEV(3, -5);
  // arm.setDEV(6, 5);
  // arm.setDEV(5, 5);
  // arm.setDEV(9, 7);
  // arm.setDEV(12, 5);
  // arm.setDEV(15, 5);
  // arm.setDEV(17, -7);
  // arm.setDEV(18, -10);
  // arm.setDEV(20, -45);

  //   arm.setDEV(4, -5);
  // arm.setDEV(5, 5);
  // arm.setDEV(8, -10);
  // arm.setDEV(9, -10);
  // arm.setDEV(11, -10);
  // arm.setDEV(13, 5);
  // arm.setDEV(14, 5);
  // arm.setDEV(15, -5);
  // arm.setDEV(18, 5);//
  // arm.setDEV(20, -45);

  arm.setDEV(2, 5);
  arm.setDEV(4, 5);
  arm.setDEV(5, -5);
  arm.setDEV(6, -5);
  arm.setDEV(9, 10);
  arm.setDEV(13, 10);

  PS4.begin("ba:ba:ca:ea:02:01");
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++)
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }

  servo1.attach(14);
  servo2.attach(15);
  sentry_err_t err = SENTRY_OK;
  Wire.begin();
  while (SENTRY_OK != sentry.begin(&Wire))
  {
    yield();
  }
  xTaskCreatePinnedToCore(myTask, "My Task", 4096, NULL, 1, &myTaskHandle, 1);
  sentry.SetParamNum(VISION_TYPE, 1);
  taskFlag = 1;
  param.width = 3;
  param.height = 4;
  param.label = 3;
  sentry.SetParam(VISION_TYPE, &param, 1);
  sentry.VisionBegin(VISION_TYPE);
  vTaskResume(myTaskHandle);
  Serial2.begin(500000, SERIAL_8N1, 18, 4);
  Serial1.begin(500000, SERIAL_8N1, 16, 5);
  //  Serial.begin(115200);
  arm.pSerial = &Serial1;
  arm.pSerial2 = &Serial2;
  gait_prg.Init();
  delay(100);
  Mpu.mpu_Init();
  hexapod.Init(0);
  arm.servoMove(20, 20, 1000);
  arm.servoMove(21, 50, 1000);
  delay(1000);
  hexapod.mode_select(1);
  // while (1)
  // {
  //   for (int i = 0; i < 19; i++)
  //   {
  //     arm.servoMove(i, 150, 500);
  //     delay(500);
  //   }
  // //   // waveGait();
  //  }

  // hexapod.velocity_cal(0, 60, 0);
  // while (y < 40)
  // {
  //   hexapodMove();
  //   Mpu.mpu_cab();
  // }
  // findBlock(50, 52, 40, 45);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 52, 40, 42);
  // takeBall();
  // for (int i = 0; i < 5; i++)
  // {
  //   sentry.SetParamNum(Sentry::kVisionBlob, 1);
  //   /* Set minimum blob size(pixel) */
  //   param.width = 9;
  //   param.height = 12;
  //   /* Set blob1 color */
  //   param.label = Sentry::kColorRed;
  //   sentry.SetParam(Sentry::kVisionBlob, &param, 1);
  //   delay(10);
  // }
  // hexapod.mode_select(1);
  // hexapod.velocity_cal(0, -60, 0);
  // preTime = millis();
  // while (millis() - preTime < 1000) // 后退些防止右移撞到纸杯
  // {
  //   hexapodMove();
  // }
  // turnForward();
  // hexapod.velocity_cal(60, 0, 0);
  // while (x < 10)
  // {
  //   hexapodMove();
  // }
  // hexapodStop();
  // hexapod.mode_select(1);
  // findBlock(50, 60, 20, 30);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 60, 20, 30);
  // putBall();
  // hexapod.mode_select(1);
  // hexapod.velocity_cal(0, -50, 0);
  // preTime = millis();
  // while (millis() - preTime < 1700) // 后退些防止左移撞到纸杯
  // {
  //   hexapodMove();
  // }
  // turnForward();
  // hexapod.velocity_cal(-60, 0, 0); // 向左直到对准大门
  // preTime = millis();
  // while (millis() - preTime < 3500) //
  // {
  //   hexapodMove();
  // }
  // hexapodStop();
  // turnForward();
  // for (int i = 0; i < 5; i++)
  // {
  //   sentry.SetParamNum(Sentry::kVisionBlob, 1);
  //   /* Set minimum blob size(pixel) */
  //   param.width = 9;
  //   param.height = 12;
  //   /* Set blob1 color */
  //   param.label = Sentry::kColorRed;
  //   sentry.SetParam(Sentry::kVisionBlob, &param, 1);
  //   delay(10);
  // }
  // while (y < 40)
  // {
  //   hexapodMove();
  //   Mpu.mpu_cab();
  // }
  // findBlock(50, 52, 40, 45);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 52, 40, 42);
  // takeBall();
  // for (int i = 0; i < 5; i++)
  // {
  //   sentry.SetParamNum(Sentry::kVisionBlob, 1);
  //   /* Set minimum blob size(pixel) */
  //   param.width = 9;
  //   param.height = 12;
  //   /* Set blob1 color */
  //   param.label = Sentry::kColorRed;
  //   sentry.SetParam(Sentry::kVisionBlob, &param, 1);
  //   delay(10);
  // }
  // hexapod.mode_select(1);
  // hexapod.velocity_cal(0, 60, 0);
  // while (y < 10)
  // {
  //   Mpu.mpu_cab();
  //   hexapodMove();
  // }
  // hexapodStop();
  // hexapod.mode_select(1);
  // findBlock(50, 60, 20, 30);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 60, 20, 30);
  // putBall();

  // hexapod.velocity_cal(0, -50, 0);
  // preTime = millis();
  // while (millis() - preTime < 1000) // 后退些防止右移撞到纸杯
  // {
  //   hexapodMove();
  // }
  // turnForward();
  // hexapod.velocity_cal(60, 0, 0);
  // while (millis() - preTime < 1500) // 右移些防止右移看到纸杯
  // {
  //   hexapodMove();
  // }
  // while (x < 50)
  // {
  //   hexapodMove();
  // }
  // hexapodStop();
  // findBlock(50, 52, 84, 86);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 52, 84, 86);
  // takeBall();
  // hexapod.mode_select(1);
  // turnForward();

  // hexapod.velocity_cal(-60, 0, 0);
  // while (x < 10)
  // {
  //   hexapodMove();
  // }
  // hexapodStop();
  // findBlock(50, 60, 20, 30);
  // hexapodStop();
  // hexapod.mode_select(3);
  // seekBlock(50, 60, 20, 30);
  // putBall();
}
void loop()
{
  if (PS4.Up())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(0, 60, 0);
    // Mpu.mpu_cab();
    hexapodMove();
  }
  else if (PS4.Down())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(0, -60, 0);
    // Mpu.mpu_cab();
    hexapodMove();
  }
  else if (PS4.Left())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(-60, 0, 0);
    hexapodMove();
  }
  else if (PS4.Right())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(60, 0, 0);
    hexapodMove();
  }
  else if (PS4.Circle())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(0, 0, -60);
    hexapodMove();
  }
  else if (PS4.Square())
  {
    hexapod.mode_select(1);
    hexapod.velocity_cal(0, 0, 60);
    hexapodMove();
  }
  else if (abs(PS4.LStickX()) > 2 || abs(PS4.LStickY()) > 2)
  {
    hexapod.mode_select(2);
    hexapod.body_angle_cal(-PS4.LStickY() * 0.001f, -PS4.LStickX() * 0.002f, 0);
    hexapodMove();
    delay(20);
  }
  else if (abs(PS4.RStickX()) > 2 || abs(PS4.RStickY()) > 2)
  {
    hexapod.mode_select(3);
    hexapod.body_position_cal(5 * PS4.RStickX(), 5 * PS4.RStickY(), 0);
    hexapodMove();
    delay(100);
  }
  else if (PS4.L2())
  {
    gait_prg.set_theta_stand_2(35);
    gait_prg.set_theta_stand_3(-80);
    gait_prg.Init();
  }
  else if (PS4.L1())
  {
    pos19 -= 2;
    if (pos19 < 150)
      pos19 = 150;
    arm.servoMove(19, pos19, 20);
    delay(20);
  }
  else if (PS4.R1())
  {
    pos20 += 2;
    if (pos20 > 150)
      pos20 = 150;
    arm.servoMove(20, pos20, 20);
    delay(20);
  }
  else if (PS4.R2())
  {
    gait_prg.set_theta_stand_2(65);
    gait_prg.set_theta_stand_3(-120);
    gait_prg.Init();
  }
  else if (PS4.Share())
  {
  }
  else if (PS4.Options())
  {
    hexapod.waveGait();
  }
  else if (PS4.Cross())
  {
    for (int i = 0; i < 10; i++)
    {
      sentry.SetParamNum(Sentry::kVisionBlob, 1);
      /* Set minimum blob size(pixel) */
      param.width = 9;
      param.height = 12;
      /* Set blob1 color */
      param.label = Sentry::kColorRed;
      sentry.SetParam(Sentry::kVisionBlob, &param, 1);
      delay(10);
    }
    hexapod.mode_select(1);
    findBlock(50, 60, 50, 60);
    hexapodStop();
    hexapod.mode_select(3);
    seekBlock(50, 60, 50, 60);
    putBall();
  }
  else if (PS4.Triangle())
  {
    for (int i = 0; i < 5; i++)
    {
      sentry.SetParamNum(Sentry::kVisionBlob, 1);

      /* Set minimum blob size(pixel) */
      param.width = 9;
      param.height = 12;
      /* Set blob1 color */
      param.label = Sentry::kColorRed;
      sentry.SetParam(Sentry::kVisionBlob, &param, 1);
      delay(10);
    }
    hexapod.mode_select(1);
    findBlock(50, 52, 80, 85);
    hexapodStop();
    hexapod.mode_select(3);
    seekBlock(50, 52, 80, 82);
    takeBall();
  }
  else
  {
    hexapodStop();
  }
}