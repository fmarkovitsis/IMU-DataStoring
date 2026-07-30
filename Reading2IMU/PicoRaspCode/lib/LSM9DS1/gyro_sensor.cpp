/*
 See also LSM9DS1 Register Map and Descriptions, http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00103319.pdf

 Hardware setup:
 LSM9DS1Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The LSM9DS1 is an I2C sensor and can use the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using either a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include <Wire.h>
#include <SPI.h>
#include "gyro_sensor.h"

BiasesStr Biases;


void LSM9DS1_IMU::MadgwickQuaternionUpdateIMU(
    float ax, float ay, float az,
    float gx, float gy, float gz, float deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Normalize accelerometer
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Gradient descent step
    s1 = 4.0f * q1 * q3 * q3 + 2.0f * q3 * ax + 4.0f * q1 * q2 * q2 - 2.0f * q2 * ay;
    s2 = 4.0f * q2 * q4 * q4 - 2.0f * q4 * ax + 4.0f * q1 * q1 * q2 - 2.0f * q1 * ay - 4.0f * q2 + 8.0f * q2 * q2 * q2 + 8.0f * q2 * q3 * q3 + 4.0f * q2 * az;
    s3 = 4.0f * q1 * q1 * q3 + 2.0f * q1 * ax + 4.0f * q3 * q4 * q4 - 2.0f * q4 * ay - 4.0f * q3 + 8.0f * q3 * q2 * q2 + 8.0f * q3 * q3 * q3 + 4.0f * q3 * az;
    s4 = 4.0f * q2 * q2 * q4 - 2.0f * q2 * ax + 4.0f * q3 * q3 * q4 - 2.0f * q3 * ay;

    norm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Quaternion rate
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = 1.0f / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void LSM9DS1_IMU::MadgwickQuaternionUpdate(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float mx, float my, float mz, float deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Normalize accelerometer
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalize magnetometer
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return;
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Auxiliary variables
    float _2q1mx = 2.0f * q1 * mx;
    float _2q1my = 2.0f * q1 * my;
    float _2q1mz = 2.0f * q1 * mz;
    float _2q2mx = 2.0f * q2 * mx;

    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Reference direction of Earth’s magnetic field
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;

    // Gradient descent step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - 4.0f * _2bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-4.0f * _2bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - 4.0f * _2bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-4.0f * _2bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);

    norm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Quaternion rate
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;

    norm = 1.0f / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

BiasesStr LSM9DS1_IMU::GyroSetup(TwoWire *wirePort, uint8_t xgAddr, uint8_t mAddr, int model, bool usePrecal, BiasesStr Biases)
{
    wire = wirePort;
    xgAddress = xgAddr;
    mAddress = mAddr;
    this->model = model;

    wire->begin();
    delay(4000);

    // Read the WHO_AM_I registers, this is a good test of communication
    byte c = readByte(xgAddress, LSM9DS1XG_WHO_AM_I); // Read WHO_AM_I register for accel/gyro
    byte d = readByte(mAddress, LSM9DS1M_WHO_AM_I);   // Read WHO_AM_I register for magnetometer

    if (SerialDebug)
    {
        Serial.println("LSM9DS1 9-axis motion sensor...");
        Serial.print("LSM9DS1 accel/gyro");
        Serial.print("I AM ");
        Serial.print(c, HEX);
        Serial.print(" I should be ");
        Serial.println(0x68, HEX);
        Serial.print("LSM9DS1 magnetometer");
        Serial.print("I AM ");
        Serial.print(d, HEX);
        Serial.print(" I should be ");
        Serial.println(0x3D, HEX);
    }

    if (c == 0x68 && d == 0x3D) // WHO_AM_I expected values
    {
        // get sensor resolutions, only need to do this once
        getAres();
        getGres();
        getMres();

        if(SerialDebug)
        {
            Serial.println("LSM9DS1 is online...");
            Serial.print("accel sensitivity is ");
            Serial.print(1. / (1000. * aRes));
            Serial.println(" LSB/mg");
            Serial.print("gyro sensitivity is ");
            Serial.print(1. / (1000. * gRes));
            Serial.println(" LSB/mdps");
            Serial.print("mag sensitivity is ");
            Serial.print(1. / (1000. * mRes));
            Serial.println(" LSB/mGauss");
        }


        // Calibrate gyro and accelerometers, load biases in bias registers
        if (usePrecal == false)    
            accelgyrocalLSM9DS1(gyroBias, accelBias);

        if(SerialDebug)
        {
            Serial.println(" Calibrate gyro and accel");
            Serial.println("accel biases (mg)");
            Serial.println(1000. * accelBias[0]);
            Serial.println(1000. * accelBias[1]);
            Serial.println(1000. * accelBias[2]);
            Serial.println("gyro biases (dps)");
            Serial.println(gyroBias[0]);
            Serial.println(gyroBias[1]);
            Serial.println(gyroBias[2]);
        }
        
        if (usePrecal == false)
        {
            magcalLSM9DS1(magBias);

            for (int i = 0; i < 3; i++)
            {
                Biases.gyroB[i] = gyroBias[i];
            }
            
            for (int i = 0; i < 3; i++)
            {
                Biases.accelB[i] = accelBias[i];
            }

            for (int i = 0; i < 3; i++)
            {
                Biases.magB[i] = magBias[i];
            }
        }
        else
        {
            for (int i=0; i<3; i++)
            {
                gyroBias[i] = Biases.gyroB[i];
                accelBias[i] = Biases.accelB[i];
                magBias[i] = Biases.magB[i];
            }
        }
        
        if(SerialDebug)
        {
            Serial.println("mag biases (mG)");
            Serial.println(1000. * magBias[0]);
            Serial.println(1000. * magBias[1]);
            Serial.println(1000. * magBias[2]);
        }

        initLSM9DS1();

        if(SerialDebug)
            Serial.println("LSM9DS1 initialized for active data mode....");

        count = millis();
        //return true;
    }
    else
    {
        if(SerialDebug)
        {
            Serial.print("Could not connect to LSM9DS1: 0x");
            Serial.println(c, HEX);
        }
        //return false;
    }

    return Biases;
}

void LSM9DS1_IMU::GyroLoop(float deltat)
{
    // ==========================================================
    //  READ SENSORS
    // ==========================================================
    if (readByte(xgAddress, LSM9DS1XG_STATUS_REG) & 0x01)
    {
        readAccelData(accelCount);
        ax = accelCount[0] * aRes;
        ay = accelCount[1] * aRes;
        az = accelCount[2] * aRes;
    }

    if (readByte(xgAddress, LSM9DS1XG_STATUS_REG) & 0x02)
    {
        readGyroData(gyroCount);
        gx = gyroCount[0] * gRes - gyroBias[0]; // deg/s
        gy = gyroCount[1] * gRes - gyroBias[1];
        gz = gyroCount[2] * gRes - gyroBias[2];
    }

    if (readByte(mAddress, LSM9DS1M_STATUS_REG_M) & 0x08)
    {
        readMagData(magCount);
        mx = magCount[0] * mRes;
        my = magCount[1] * mRes;
        mz = magCount[2] * mRes;
    }

    // ==========================================================
    // MAGNETOMETER HARD + SOFT IRON CALIBRATION
    // ==========================================================
    float mx_h = mx - b_vec[0];
    float my_h = my - b_vec[1];
    float mz_h = mz - b_vec[2];

    float mx_c = A[0][0] * mx_h + A[0][1] * my_h + A[0][2] * mz_h;
    float my_c = A[1][0] * mx_h + A[1][1] * my_h + A[1][2] * mz_h;
    float mz_c = A[2][0] * mx_h + A[2][1] * my_h + A[2][2] * mz_h;

    // ==============================
    // IMU → BODY FRAME TRANSFORM
    // Body frame:
    // X = left
    // Y = forward
    // Z = up
    // ==============================

    float ax_b = -ax; // right → left
    float ay_b = ay;  // forward
    float az_b = az;  // up

    // Convert gyro to rad/s
    float gx_b = -gx * DEG_TO_RAD; // roll rate
    float gy_b = gy * DEG_TO_RAD;  // pitch rate
    float gz_b = gz * DEG_TO_RAD;  // yaw rate

    float mx_b, my_b, mz_b;

    if(this->model == 1) // black breakout
    {
        mx_b = -my_c;
        my_b = mx_c;
        mz_b = mz_c;
    }
    else if(this->model == 2) // blue breakout
    {
        mx_b = my_c;
        my_b = -mx_c; 
        mz_b = mz_c;
    }

    // ==========================================================
    // NORMALIZE ACCELEROMETER
    // ==========================================================

    // raw magnitude in g's (because ax,ay,az are already scaled by aRes)
    float accelMagRaw = sqrt(ax_b * ax_b + ay_b * ay_b + az_b * az_b);

    // Now normalize for the filter
    if (accelMagRaw > 0.0f)
    {
        ax_b /= accelMagRaw;
        ay_b /= accelMagRaw;
        az_b /= accelMagRaw;
    }

    float gyroMag = sqrt(gx_b * gx_b + gy_b * gy_b + gz_b * gz_b);

    // ==========================================================
    // ADAPTIVE BETA (dynamic correction strength)
    // ==========================================================

    float accelError = fabs(accelMagRaw - 1.0f);

    // Detect still state
    bool isStill = (accelError < 0.1f) && (gyroMag < 1.0f);

    if (isStill)
    {
        beta = 0.3f; // Strong correction when perfectly still
    }
    else if (gyroMag > 3.0f)
    {
        beta = 0.002f; // Very low correction during fast motion
    }
    else
    {
        beta = 0.015f; // Medium correction
    }

    // ==========================================================
    // NORMALIZE MAGNETOMETER
    // ==========================================================

    float magNorm = sqrtf(mx_b * mx_b + my_b * my_b + mz_b * mz_b);
    if (magNorm > 0.0f)
    {
        mx_b /= magNorm;
        my_b /= magNorm;
        mz_b /= magNorm;
    }

    // ==========================================================
    // INJECT YAW CORRECTION INTO GYRO Z
    // =========================================================

    bool useMag;

    if (isStill)
        useMag = false; // always use mag when still
    else if (gyroMag < 3.0f)
        useMag = false; // moderate motion
    else
        useMag = true; // fast motion → ignore mag

    if (useMag)
    {
        MadgwickQuaternionUpdate(
            ax_b, ay_b, az_b,
            gx_b, gy_b, gz_b,
            mx_b, my_b, mz_b, deltat);
    }
    else
    {
        // fallback to IMU-only version
        MadgwickQuaternionUpdateIMU(
            ax_b, ay_b, az_b,
            gx_b, gy_b, gz_b, deltat);
    }

    // ==========================================================
    //  ONLINE GYRO BIAS ADAPTATION
    // ==========================================================
    if (gyroMag < 0.01f && fabs(accelMagRaw - 1.0f) < 0.02f)
    {
        float k = 0.0005f;
        gyroBias[0] += k * gx;
        gyroBias[1] += k * gy;
        gyroBias[2] += k * gz;

        gyroBias[0] = constrain(gyroBias[0], -5.0f, 5.0f);
        gyroBias[1] = constrain(gyroBias[1], -5.0f, 5.0f);
        gyroBias[2] = constrain(gyroBias[2], -5.0f, 5.0f);
    }
}

//================================================================================================
//====== Set of useful function to access acceleration. gyroscope and magnetometer data
//================================================================================================

void LSM9DS1_IMU::GyroGetQuaternion(float *dest)
{
    dest[0] = q[0];
    dest[1] = q[1];
    dest[2] = q[2];
    dest[3] = q[3];
}

void LSM9DS1_IMU::getMres()
{
    switch (Mscale)
    {
    // Possible magnetometer scales (and their register bit settings) are:
    // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case MFS_4G:
        mRes = 4.0 / 32768.0;
        break;
    case MFS_8G:
        mRes = 8.0 / 32768.0;
        break;
    case MFS_12G:
        mRes = 12.0 / 32768.0;
        break;
    case MFS_16G:
        mRes = 16.0 / 32768.0;
        break;
    }
}

void LSM9DS1_IMU::getGres()
{
    switch (Gscale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11).
    case GFS_245DPS:
        gRes = 245.0 / 32768.0;
        break;
    case GFS_500DPS:
        gRes = 500.0 / 32768.0;
        break;
    case GFS_2000DPS:
        gRes = 2000.0 / 32768.0;
        break;
    }
}

void LSM9DS1_IMU::getAres()
{
    switch (Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11).
    case AFS_2G:
        aRes = 2.0 / 32768.0;
        break;
    case AFS_16G:
        aRes = 16.0 / 32768.0;
        break;
    case AFS_4G:
        aRes = 4.0 / 32768.0;
        break;
    case AFS_8G:
        aRes = 8.0 / 32768.0;
        break;
    }
}

void LSM9DS1_IMU::readAccelData(int16_t *destination)
{
    uint8_t rawData[6];                                                 // x/y/z accel register data stored here
    readBytes(xgAddress, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]); // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];           // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void LSM9DS1_IMU::readGyroData(int16_t *destination)
{
    uint8_t rawData[6];                                                // x/y/z gyro register data stored here
    readBytes(xgAddress, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];          // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void LSM9DS1_IMU::readMagData(int16_t *destination)
{
    uint8_t rawData[6];                                              // x/y/z gyro register data stored here
    readBytes(mAddress, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];        // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];        // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

void LSM9DS1_IMU::initLSM9DS1()
{
    // enable the 3-axes of the gyroscope
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG4, 0x38);
    // configure the gyroscope
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
    delay(200);
    // enable the three axes of the accelerometer
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG5_XL, 0x38);
    // configure the accelerometer-specify bandwidth selection with Abw
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 | Abw);
    delay(200);
    // enable block data update, allow auto-increment during multiple byte read
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG8, 0x44);
    // configure the magnetometer-enable temperature compensation of mag data
    writeByte(mAddress, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG2_M, Mscale << 5);                   // select mag full scale
    writeByte(mAddress, LSM9DS1M_CTRL_REG3_M, 0x00);                          // continuous conversion mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG4_M, Mmode << 2);                    // select z-axis mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG5_M, 0x40);                          // select block update mode
}

bool LSM9DS1_IMU::GyroSelfTest()
{
    bool status = true;
    float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
    float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

    writeByte(xgAddress, LSM9DS1XG_CTRL_REG10, 0x00); // disable self test
    accelgyrocalLSM9DS1(gyro_noST, accel_noST);
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG10, 0x05); // enable gyro/accel self test
    accelgyrocalLSM9DS1(gyro_ST, accel_ST);

    float gyrodx = (gyro_ST[0] - gyro_noST[0]);
    float gyrody = (gyro_ST[1] - gyro_noST[1]);
    float gyrodz = (gyro_ST[2] - gyro_noST[2]);

    if(gyrodx > 250. || gyrodx < 20. || gyrody > 250. || gyrody < 20. || gyrodz > 250. || gyrodz < 20.)
    {
        status = false;
    }

    if (SerialDebug)
    {
        Serial.println("Gyro self-test results: ");
        Serial.print("x-axis = ");
        Serial.print(gyrodx);
        Serial.print(" dps");
        Serial.println(" should be between 20 and 250 dps");
        Serial.print("y-axis = ");
        Serial.print(gyrody);
        Serial.print(" dps");
        Serial.println(" should be between 20 and 250 dps");
        Serial.print("z-axis = ");
        Serial.print(gyrodz);
        Serial.print(" dps");
        Serial.println(" should be between 20 and 250 dps");
    }

    float accdx = 1000. * (accel_ST[0] - accel_noST[0]);
    float accdy = 1000. * (accel_ST[1] - accel_noST[1]);
    float accdz = 1000. * (accel_ST[2] - accel_noST[2]);

    if(accdx > 1700. || accdx < 60. || accdy > 1700. || accdy < 60. || accdz > 1700. || accdz < 60.)
    {
        status = false;
    }

    if (SerialDebug)
    {
        Serial.println("Accelerometer self-test results: ");
        Serial.print("x-axis = ");
        Serial.print(accdx);
        Serial.print(" mg");
        Serial.println(" should be between 60 and 1700 mg");
        Serial.print("y-axis = ");
        Serial.print(accdy);
        Serial.print(" mg");
        Serial.println(" should be between 60 and 1700 mg");
        Serial.print("z-axis = ");
        Serial.print(accdz);
        Serial.print(" mg");
        Serial.println(" should be between 60 and 1700 mg");
    }

    writeByte(xgAddress, LSM9DS1XG_CTRL_REG10, 0x00); // disable self test
    delay(200);

    return status;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void LSM9DS1_IMU::accelgyrocalLSM9DS1(float *dest1, float *dest2)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    uint16_t samples, ii;

    // enable the 3-axes of the gyroscope
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG4, 0x38);
    // configure the gyroscope
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
    delay(200);
    // enable the three axes of the accelerometer
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG5_XL, 0x38);
    // configure the accelerometer-specify bandwidth selection with Abw
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 | Abw);
    delay(200);
    // enable block data update, allow auto-increment during multiple byte read
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG8, 0x44);

    // First get gyro bias
    byte c = readByte(xgAddress, LSM9DS1XG_CTRL_REG9);
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG9, c | 0x02);    // Enable gyro FIFO
    delay(50);                                                      // Wait for change to take effect
    writeByte(xgAddress, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F); // Enable gyro FIFO stream mode and set watermark at 32 samples
    delay(1000);                                                    // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(xgAddress, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

    for (ii = 0; ii < samples; ii++)
    { // Read the gyro data stored in the FIFO
        int16_t gyro_temp[3] = {0, 0, 0};
        readBytes(xgAddress, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
        gyro_temp[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        gyro_temp[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
        gyro_temp[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]);

        gyro_bias[0] += (int32_t)gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }

    gyro_bias[0] /= samples; // average the data
    gyro_bias[1] /= samples;
    gyro_bias[2] /= samples;

    dest1[0] = (float)gyro_bias[0] * gRes; // Properly scale the data to get deg/s
    dest1[1] = (float)gyro_bias[1] * gRes;
    dest1[2] = (float)gyro_bias[2] * gRes;

    c = readByte(xgAddress, LSM9DS1XG_CTRL_REG9);
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG9, c & ~0x02); // Disable gyro FIFO
    delay(50);
    writeByte(xgAddress, LSM9DS1XG_FIFO_CTRL, 0x00); // Enable gyro bypass mode

    // now get the accelerometer bias
    c = readByte(xgAddress, LSM9DS1XG_CTRL_REG9);
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG9, c | 0x02);    // Enable accel FIFO
    delay(50);                                                      // Wait for change to take effect
    writeByte(xgAddress, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F); // Enable accel FIFO stream mode and set watermark at 32 samples
    delay(1000);                                                    // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(xgAddress, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

    for (ii = 0; ii < samples; ii++)
    { // Read the accel data stored in the FIFO
        int16_t accel_temp[3] = {0, 0, 0};
        readBytes(xgAddress, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
        accel_temp[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
        accel_temp[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
    }

    accel_bias[0] /= samples; // average the data
    accel_bias[1] /= samples;
    accel_bias[2] /= samples;

    dest2[0] = (float)accel_bias[0] * aRes; // Properly scale the data to get g
    dest2[1] = (float)accel_bias[1] * aRes;
    dest2[2] = (float)accel_bias[2] * aRes;

    c = readByte(xgAddress, LSM9DS1XG_CTRL_REG9);
    writeByte(xgAddress, LSM9DS1XG_CTRL_REG9, c & ~0x02); // Disable accel FIFO
    delay(50);
    writeByte(xgAddress, LSM9DS1XG_FIFO_CTRL, 0x00); // Enable accel bypass mode
}

void LSM9DS1_IMU::magcalLSM9DS1(float *dest1)
{
    uint8_t data[6]; // data array to hold mag x, y, z, data
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

    // configure the magnetometer-enable temperature compensation of mag data
    writeByte(mAddress, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG2_M, Mscale << 5);                   // select mag full scale
    writeByte(mAddress, LSM9DS1M_CTRL_REG3_M, 0x00);                          // continuous conversion mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG4_M, Mmode << 2);                    // select z-axis mode
    writeByte(mAddress, LSM9DS1M_CTRL_REG5_M, 0x40);                          // select block update mode

    if (SerialDebug)
        Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);

    sample_count = 128;
    for (ii = 0; ii < sample_count; ii++)
    {
        int16_t mag_temp[3] = {0, 0, 0};
        readBytes(mAddress, LSM9DS1M_OUT_X_L_M, 6, &data[0]); // Read the six raw data registers into data array
        mag_temp[0] = (int16_t)(((int16_t)data[1] << 8) | data[0]);   // Form signed 16-bit integer for each sample in FIFO
        mag_temp[1] = (int16_t)(((int16_t)data[3] << 8) | data[2]);
        mag_temp[2] = (int16_t)(((int16_t)data[5] << 8) | data[4]);
        for (int jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        delay(105); // at 10 Hz ODR, new mag data is available every 100 ms
    }

    if (SerialDebug)
    {
       Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
       Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
       Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);
    }

    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    dest1[0] = (float)mag_bias[0] * mRes; // save mag biases in G for main program
    dest1[1] = (float)mag_bias[1] * mRes;
    dest1[2] = (float)mag_bias[2] * mRes;

    // write biases to accelerometermagnetometer offset registers as counts);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t)mag_bias[0] & 0xFF);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t)mag_bias[1] & 0xFF);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t)mag_bias[2] & 0xFF);
    // writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);

    if (SerialDebug)
        Serial.println("Mag Calibration done!");
}

// I2C read/write functions for the LSM9DS1and AK8963 sensors (as class methods)

void LSM9DS1_IMU::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    wire->beginTransmission(address); // Initialize the Tx buffer
    wire->write(subAddress);          // Put slave register address in Tx buffer
    wire->write(data);                // Put data in Tx buffer
    wire->endTransmission();          // Send the Tx buffer
}

uint8_t LSM9DS1_IMU::readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data;                                // `data` will store the register data
    wire->beginTransmission(address);            // Initialize the Tx buffer
    wire->write(subAddress);                     // Put slave register address in Tx buffer
    wire->endTransmission(false);                // Send the Tx buffer, but send a restart to keep connection alive
    wire->requestFrom(address, (size_t)1);       // Read one byte from slave register address
    data = wire->read();                         // Fill Rx buffer with result
    return data;                                 // Return data read from slave register
}

void LSM9DS1_IMU::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    wire->beginTransmission(address);            // Initialize the Tx buffer
    wire->write(subAddress);                     // Put slave register address in Tx buffer
    wire->endTransmission(false);                // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    wire->requestFrom(address, (size_t)count);   // Read bytes from slave register address
    while (wire->available())
    {
        dest[i++] = wire->read();
    } // Put read results in the Rx buffer
}

float LSM9DS1_IMU::wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

float LSM9DS1_IMU::wrap360(float a)
{
    while (a >= 360.0f)
        a -= 360.0f;
    while (a < 0.0f)
        a += 360.0f;
    return a;
}