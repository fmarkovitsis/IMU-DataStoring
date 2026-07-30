#include <Arduino.h>
#include "gyro_sensor.h"
#include <pico/multicore.h>
#include <ArduinoJson.h>

// ================= IMU =================
LSM9DS1_IMU imu;

// ================= Shared Data (LOCK-FREE) =================
volatile uint32_t seq = 0;
volatile uint32_t timestamp_shared = 0;
volatile float q_shared[4] = {1, 0, 0, 0};

bool started = false;
String input = "";
uint32_t startTime = 0;
uint32_t currTime = 0;

uint32_t currTimeMic = 0;
uint32_t lastTimeMic = 0;

volatile bool proc1work = false;
volatile bool data1ready = false;

// ================= Binary Packet =================
struct __attribute__((packed)) Packet
{
  uint32_t timestamp;
  float q[4];
};

Packet data;

void biasesToJson(const BiasesStr& Biases, JsonDocument& doc) {
  JsonArray gyro  = doc["gyroB"].to<JsonArray>();
  JsonArray accel = doc["accelB"].to<JsonArray>();
  JsonArray mag   = doc["magB"].to<JsonArray>();

  for (int i = 0; i < 3; i++) {
    gyro.add(Biases.gyroB[i]);
    accel.add(Biases.accelB[i]);
    mag.add(Biases.magB[i]);
  }
}

// ========================================================
// ====================== CORE 0 ==========================
// ========================================================

void setup()
{
  Serial.begin(460800);
  delay(2000);

  JsonDocument BiasesDoc, dummy;
  BiasesStr Biases;

  while (!Serial)
  {
  } // wait for USB serial monitor

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  String line = Serial.readStringUntil('\n');
  line.trim();

  Serial.println("1");

  line = Serial.readStringUntil('\n');
  line.trim();

  if (line == "y")
  {
    Biases = imu.GyroSetup(&Wire, 0x6B, 0x1E, 1, false, Biases);

    biasesToJson(Biases, BiasesDoc); //disable this for now
    serializeJson(BiasesDoc, Serial);
    Serial.print("\n");
  }
  else
  {
    String json = Serial.readStringUntil('\n');

    deserializeJson(BiasesDoc, json);

    for (int i=0; i<3; i++)
    {
      Biases.gyroB[i] = BiasesDoc["gyroB"][i];
      Biases.accelB[i] = BiasesDoc["accelB"][i];
      Biases.magB[i] = BiasesDoc["magB"][i];
    }

    // debugging
    biasesToJson(Biases, dummy);
    serializeJson(dummy, Serial);
    Serial.print("\n");

    Biases = imu.GyroSetup(&Wire, 0x6B, 0x1E, 1, true, Biases);
  } 

  Serial.println("1");
  
  line = Serial.readStringUntil('\n');
  line.trim();

  startTime = millis();
}

void loop()
{
  currTimeMic = micros();
  currTime = millis() - startTime;
  
  if ((currTime % 20) < 18)
  {
    imu.GyroLoop((currTimeMic - lastTimeMic) * 1e-6f);
    lastTimeMic = currTimeMic;
  }

  if (currTime % 20 == 0)
  {
    float q[4];
    imu.GyroGetQuaternion(q);

    data.timestamp = currTime;
    for (int i=0; i<4; i++)
    {
      data.q[i] = q[i];
    }

    Serial.write((uint8_t *)&data, sizeof(data));
  }
}