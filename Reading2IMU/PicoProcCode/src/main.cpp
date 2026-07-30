#include <Arduino.h>
#include "gyro_sensor.h"
#include <pico/multicore.h>

// ================= IMUs =================
LSM9DS1_IMU imu1;
LSM9DS1_IMU imu2;

BiasesStr biases1, biases2;
volatile bool lock1, lock2;

uint32_t startTime = 0;
uint32_t currTime = 0;

// ================= Shared Data =================
volatile float q1_shared[4] = {1, 0, 0, 0};
volatile float q2_shared[4] = {1, 0, 0, 0};

volatile uint32_t timestamp1_shared = 0;
volatile uint32_t timestamp2_shared = 0;

// ========================================================
// =================== SEND PACKET ========================
// ========================================================
void sendPacket(uint32_t ts, volatile float *q)
{
  Serial.write((uint8_t *)&ts, 4);
  Serial.write((uint8_t *)&q[0], 4);
  Serial.write((uint8_t *)&q[1], 4);
  Serial.write((uint8_t *)&q[2], 4);
  Serial.write((uint8_t *)&q[3], 4);
}

// ========================================================
// =================== STORE BIASES =======================
// ========================================================

// void biasesToJson(const BiasesStr& Biases, JsonDocument& doc) {
//   JsonArray gyro  = doc["gyroB"].to<JsonArray>();
//   JsonArray accel = doc["accelB"].to<JsonArray>();
//   JsonArray mag   = doc["magB"].to<JsonArray>();

//   for (int i = 0; i < 3; i++) {
//     gyro.add(Biases.gyroB[i]);
//     accel.add(Biases.accelB[i]);
//     mag.add(Biases.magB[i]);
//   }
// }

// ========================================================
// ====================== CORE 1 ==========================
// ========================================================
void core1_main()
{
  uint32_t lastIMU = micros();

  while (true)
  {
    uint32_t now = micros();
    float dt = (now - lastIMU) * 1e-6f;
    lastIMU = now;

    imu2.GyroLoop(dt);

    if (lock1 == true)
    {
      lock1 = false;

      float q[4];
      imu2.GyroGetQuaternion(q);

      q2_shared[0] = q[0];
      q2_shared[1] = q[1];
      q2_shared[2] = q[2];
      q2_shared[3] = q[3];
      
      lock2 = true;
    }
  }
}

// ========================================================
// ====================== CORE 0 ==========================
// ========================================================
void setup()
{
  Serial.begin(460800);
  delay(2000);

  while (!Serial)
  {
  } // wait for USB serial monitor

  // ---- I2C0 for IMU1 ----
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  // ---- I2C1 for IMU2 ----
  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.begin();

  // String line = Serial.readStringUntil('\n');
  // line.trim();
  
  // if (line == "y")
  // {
    biases1 = imu1.GyroSetup(&Wire, 0x6B, 0x1E, 1, false, biases1);
    biases2 = imu2.GyroSetup(&Wire1, 0x6B, 0x1E, 1, false, biases2);


    // biasesToJson(Biases, BiasesDoc);
    // serializeJson(BiasesDoc, Serial);
    // Serial.print("\n");
  // }
  // !!! setup second core precal
  // Serial.println("2");

  // line = Serial.readStringUntil('\n');
  // line.trim();

  // Start second core
  multicore_launch_core1(core1_main);

  startTime = millis();
}

void loop()
{
  static uint32_t lastIMU = micros();
  currTime = millis() - startTime;

  uint32_t now = micros();
  float dt = (now - lastIMU) * 1e-6f;
  lastIMU = now;

  // -------- IMU1 --------
  if ((currTime % 20) < 17)
  {
    imu1.GyroLoop(dt);
  }

  if ((currTime % 20) == 0)
  {
    lock1 = true;

    float q1[4];
    imu1.GyroGetQuaternion(q1);

    timestamp1_shared = currTime;
    timestamp2_shared = timestamp1_shared;
    q1_shared[0] = q1[0];
    q1_shared[1] = q1[1];
    q1_shared[2] = q1[2];
    q1_shared[3] = q1[3];

    // =====================================================
    // SEND BOTH PACKETS IN ORDER (CRITICAL)
    // =====================================================

    while (lock2 != true) {}
    lock2 = false;

    sendPacket(timestamp1_shared, q1_shared); // gyro1
    sendPacket(timestamp2_shared, q2_shared); // gyro2
  }
}