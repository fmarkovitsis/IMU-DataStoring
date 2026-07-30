#ifndef GYRO_MODULE_H
#define GYRO_MODULE_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "KalmanFilter.h"

typedef struct 
{
    float gyroB[3], accelB[3], magB[3];
} BiasesStr;

// ======================
// ===   Defines   ===
// ======================

// --- Low-level I2C helpers for self-test (Adafruit wrapper doesn’t expose them)
#define LSM9DS1_ADDRESS_AG (0x6B)
#define LSM9DS1_ADDRESS_M (0x1E)
#define LSM9DS1M_CTRL_REG1_M 0x20
#define LSM9DS1M_CTRL_REG2_M 0x21
#define LSM9DS1M_CTRL_REG3_M 0x22
#define LSM9DS1AG_CTRL_REG10 0x24
#define OUT_X_L_G 0x18
#define OUT_Y_L_G 0x1A
#define OUT_Z_L_G 0x1C
#define OUT_X_L_XL 0x28
#define OUT_Y_L_XL 0x2A
#define OUT_Z_L_XL 0x2C
#define REG_ST_G 0x24
#define REG_ST_XL 0x1F
// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS 0x04
#define LSM9DS1XG_ACT_DUR 0x05
#define LSM9DS1XG_INT_GEN_CFG_XL 0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL 0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL 0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL 0x09
#define LSM9DS1XG_INT_GEN_DUR_XL 0x0A
#define LSM9DS1XG_REFERENCE_G 0x0B
#define LSM9DS1XG_INT1_CTRL 0x0C
#define LSM9DS1XG_INT2_CTRL 0x0D
#define LSM9DS1XG_WHO_AM_I 0x0F // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G 0x10
#define LSM9DS1XG_CTRL_REG2_G 0x11
#define LSM9DS1XG_CTRL_REG3_G 0x12
#define LSM9DS1XG_ORIENT_CFG_G 0x13
#define LSM9DS1XG_INT_GEN_SRC_G 0x14
#define LSM9DS1XG_OUT_TEMP_L 0x15
#define LSM9DS1XG_OUT_TEMP_H 0x16
#define LSM9DS1XG_STATUS_REG 0x17
#define LSM9DS1XG_OUT_X_L_G 0x18
#define LSM9DS1XG_OUT_X_H_G 0x19
#define LSM9DS1XG_OUT_Y_L_G 0x1A
#define LSM9DS1XG_OUT_Y_H_G 0x1B
#define LSM9DS1XG_OUT_Z_L_G 0x1C
#define LSM9DS1XG_OUT_Z_H_G 0x1D
#define LSM9DS1XG_CTRL_REG4 0x1E
#define LSM9DS1XG_CTRL_REG5_XL 0x1F
#define LSM9DS1XG_CTRL_REG6_XL 0x20
#define LSM9DS1XG_CTRL_REG7_XL 0x21
#define LSM9DS1XG_CTRL_REG8 0x22
#define LSM9DS1XG_CTRL_REG9 0x23
#define LSM9DS1XG_CTRL_REG10 0x24
#define LSM9DS1XG_INT_GEN_SRC_XL 0x26
// #define LSM9DS1XG_STATUS_REG        0x27 // duplicate of 0x17!
#define LSM9DS1XG_OUT_X_L_XL 0x28
#define LSM9DS1XG_OUT_X_H_XL 0x29
#define LSM9DS1XG_OUT_Y_L_XL 0x2A
#define LSM9DS1XG_OUT_Y_H_XL 0x2B
#define LSM9DS1XG_OUT_Z_L_XL 0x2C
#define LSM9DS1XG_OUT_Z_H_XL 0x2D
#define LSM9DS1XG_FIFO_CTRL 0x2E
#define LSM9DS1XG_FIFO_SRC 0x2F
#define LSM9DS1XG_INT_GEN_CFG_G 0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G 0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G 0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G 0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G 0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G 0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G 0x36
#define LSM9DS1XG_INT_GEN_DUR_G 0x37

// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M 0x05
#define LSM9DS1M_OFFSET_X_REG_H_M 0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M 0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M 0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M 0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M 0x0A
#define LSM9DS1M_WHO_AM_I 0x0F // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M 0x20
#define LSM9DS1M_CTRL_REG2_M 0x21
#define LSM9DS1M_CTRL_REG3_M 0x22
#define LSM9DS1M_CTRL_REG4_M 0x23
#define LSM9DS1M_CTRL_REG5_M 0x24
#define LSM9DS1M_STATUS_REG_M 0x27
#define LSM9DS1M_OUT_X_L_M 0x28
#define LSM9DS1M_OUT_X_H_M 0x29
#define LSM9DS1M_OUT_Y_L_M 0x2A
#define LSM9DS1M_OUT_Y_H_M 0x2B
#define LSM9DS1M_OUT_Z_L_M 0x2C
#define LSM9DS1M_OUT_Z_H_M 0x2D
#define LSM9DS1M_INT_CFG_M 0x30
#define LSM9DS1M_INT_SRC_M 0x31
#define LSM9DS1M_INT_THS_L_M 0x32
#define LSM9DS1M_INT_THS_H_M 0x33

#define ADO 1
#if ADO
#define LSM9DS1XG_ADDRESS 0x6B //  Device address when ADO = 1
#define LSM9DS1M_ADDRESS 0x1E  //  Address of magnetometer
#define MS5611_ADDRESS 0x77    //  Address of altimeter
#else
#define LSM9DS1XG_ADDRESS 0x6A //  Device address when ADO = 0
#define LSM9DS1M_ADDRESS 0x1D  //  Address of magnetometer
#define MS5611_ADDRESS 0x77    //  Address of altimeter
#endif

#define SerialDebug false // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale
{ // set of allowable accel full scale settings
    AFS_2G = 0,
    AFS_16G,
    AFS_4G,
    AFS_8G
};

enum Aodr
{ // set of allowable gyro sample rates
    AODR_PowerDown = 0,
    AODR_10Hz,
    AODR_50Hz,
    AODR_119Hz,
    AODR_238Hz,
    AODR_476Hz,
    AODR_952Hz
};

enum Abw
{ // set of allowable accewl bandwidths
    ABW_408Hz = 0,
    ABW_211Hz,
    ABW_105Hz,
    ABW_50Hz
};

enum Gscale
{ // set of allowable gyro full scale settings
    GFS_245DPS = 0,
    GFS_500DPS,
    GFS_NoOp,
    GFS_2000DPS
};

enum Godr
{ // set of allowable gyro sample rates
    GODR_PowerDown = 0,
    GODR_14_9Hz,
    GODR_59_5Hz,
    GODR_119Hz,
    GODR_238Hz,
    GODR_476Hz,
    GODR_952Hz
};

enum Gbw
{                // set of allowable gyro data bandwidths
    GBW_low = 0, // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
    GBW_med,     // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
    GBW_high,    // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
    GBW_highest  // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale
{ // set of allowable mag full scale settings
    MFS_4G = 0,
    MFS_8G,
    MFS_12G,
    MFS_16G
};

enum Mmode
{
    MMode_LowPower = 0,
    MMode_MedPerformance,
    MMode_HighPerformance,
    MMode_UltraHighPerformance
};

enum Modr
{ // set of allowable mag sample rates
    MODR_0_625Hz = 0,
    MODR_1_25Hz,
    MODR_2_5Hz,
    MODR_5Hz,
    MODR_10Hz,
    MODR_20Hz,
    MODR_80Hz
};

class LSM9DS1_IMU
{
    public:
        BiasesStr GyroSetup(TwoWire *wirePort,
                        uint8_t xgAddr,
                        uint8_t mAddr,
                        int model,
                        bool usePrecal,
                        BiasesStr Biases);
        void GyroLoop(float dt);
        void GyroGetQuaternion(float *dest);
        bool GyroSelfTest();
    private:
        int model;                             // 1 for black and 1 for blue
        TwoWire *wire;
        uint8_t xgAddress;
        uint8_t mAddress;

        // Specify sensor full scale
        uint8_t Gscale = GFS_245DPS;           // gyro full scale
        uint8_t Godr = GODR_238Hz;             // gyro data sample rate
        uint8_t Gbw = GBW_low;                 // gyro data bandwidth
        uint8_t Ascale = AFS_2G;               // accel full scale
        uint8_t Aodr = AODR_238Hz;             // accel data sample rate
        uint8_t Abw = ABW_50Hz;                // accel data bandwidth
        uint8_t Mscale = MFS_4G;               // mag full scale
        uint8_t Modr = MODR_80Hz;              // mag data sample rate
        uint8_t Mmode = MMode_HighPerformance; // magnetometer operation mode
        float aRes, gRes, mRes;                // scale resolutions per LSB for the sensors

        int16_t accelCount[3], gyroCount[3], magCount[3];                                // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
        float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer

        // === Define magnitometer calibration parameters ===
        const float A[3][3] = {
            {1.499726f, 0.048447f, -0.039171f},
            {0.048447f, 1.396507f, 0.026910f},
            {-0.039171f, 0.026910f, 1.280686f}};

        const float b_vec[3] = {-3.113008f, 36.829832f, 9.536436f};

        // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
        float GyroMeasError = PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
        float GyroMeasDrift = PI * (0.0f / 180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

        float beta = 0.02f;                             // compute beta
        float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

        uint32_t count = 0; // used to control display output rate
        float pitch, yaw, roll;

        float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
        float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

        float yawRef = 0.0f;
        float yawOutRef = 0.0f;
        bool yawOutRefSet = false;

        void MadgwickQuaternionUpdateIMU(
            float ax, float ay, float az,
            float gx, float gy, float gz, float deltat);

        void MadgwickQuaternionUpdate(
            float ax, float ay, float az,
            float gx, float gy, float gz,
            float mx, float my, float mz, float deltat);

        void getMres();
        void getGres();
        void getAres();
        void accelgyrocalLSM9DS1(float *dest1, float *dest2);
        void magcalLSM9DS1(float *dest1);
        void initLSM9DS1();
        void readAccelData(int16_t *destination);
        void readGyroData(int16_t *destination);
        void readMagData(int16_t *destination);
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
        float wrap180(float a);
        float wrap360(float a);
};

#endif // GYRO_MODULE_H
