#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Keyboard.h>
#include <Joystick.h>

MPU6050 mpu0(0x68);
MPU6050 mpu1(0x69);

bool dmpReady0 = false;
bool dmpReady1 = false;
uint8_t devStatus0;
uint8_t devStatus1;
uint8_t fifoBuffer0[64];
uint8_t fifoBuffer1[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

long startRot0 = 0;
long endRot0 = 90;
long startRot1 = 0;
long endRot1 = 90;

int toggleActivePin = A0;
int startRotPin = A1;
int endRotPin = A2;

bool gasOn = false;

int leftXData = 0;
int rightXData = 0;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                   0, 0,                 // Button Count, Hat Switch Count
                   true, true, false,    // X and Y, but no Z Axis
                   false, false, false,  // No Rx, Ry, or Rz
                   false, false,         // No rudder or throttle
                   false, false, false);

void setupMPU(MPU6050 &mpu, bool &dmpReady, uint8_t address) {
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus0 = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  if (devStatus0 == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus0);
    Serial.println(F(")"));
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  while (!Serial)
    ;

  setupMPU(mpu0, dmpReady0, 0x68);
  setupMPU(mpu1, dmpReady1, 0x69);

  Keyboard.begin();

  Joystick.begin();
  Joystick.setXAxisRange(-1000, 1000);
  Joystick.setYAxisRange(-1, 1);
  Joystick.setYAxis(0);
}

void loop() {
  if (!dmpReady0 || !dmpReady1) return;

  if (mpu0.dmpGetCurrentFIFOPacket(fifoBuffer0)) {
    leftXData = readMpuData(mpu0, fifoBuffer0, startRot0, endRot0);
  }
  if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer1)) {
    rightXData = readMpuData(mpu1, fifoBuffer1, startRot1, endRot1);
  }

  Serial.print(leftXData);
  Serial.print("\t");
  Serial.println(rightXData);

  if (analogRead(toggleActivePin) > 100) {
    if (!gasOn && (leftXData < 500 || rightXData < 500)) {
      Keyboard.press('W');
      gasOn = true;
    } else if (gasOn && (leftXData > 500 && rightXData > 500)) {
      Keyboard.release('W');
      gasOn = false;
    }
    Joystick.setXAxis(constrain((leftXData - rightXData) * 2, -1000, 1000));
  }
  else
  {
    gasOn = false;
    Keyboard.release('W');
    Joystick.setXAxis(0);
  }
}

int readMpuData(MPU6050 &mpu, uint8_t *fifoBuffer, long &startRot, long &endRot) {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  long xRot = ypr[2] * 180 / M_PI;
  long rotationAmount = map(xRot, startRot, endRot, 0, 1000);

  if (analogRead(startRotPin) > 100) {
    Serial.println("startRot");
    startRot = xRot;
  } else if (analogRead(endRotPin) > 100) {
    Serial.println("endRot");
    endRot = xRot;
  }
  return rotationAmount;
}