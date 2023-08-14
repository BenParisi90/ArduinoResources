#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

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
    while (!Serial);

    setupMPU(mpu0, dmpReady0, 0x68);
    setupMPU(mpu1, dmpReady1, 0x69);
}

void loop() {
    if (!dmpReady0 || !dmpReady1) return;
    Serial.print("Device1: ");
    readMpuData(mpu0, fifoBuffer0);
    Serial.print("\t");
    Serial.print("Device2: ");
    readMpuData(mpu1, fifoBuffer1);
    Serial.println();
}

void readMpuData(MPU6050 &mpu, uint8_t *fifoBuffer) {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.print(ypr[2]);
    Serial.print("\t");
  }
}