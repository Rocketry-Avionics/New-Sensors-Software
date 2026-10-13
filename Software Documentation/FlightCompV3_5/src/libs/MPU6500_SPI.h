#ifndef MPU6500_SPI_H
#define MPU6500_SPI_H

#include <Arduino.h>
#include <SPI.h>

// MPU6500 Register Map
#define MPU6500_SMPLRT_DIV            0x19
#define MPU6500_CONFIG                0x1A
#define MPU6500_GYRO_CONFIG           0x1B
#define MPU6500_ACCEL_CONFIG          0x1C
#define MPU6500_ACCEL_CONFIG_2        0x1D
#define MPU6500_USER_CTRL             0x6A
#define MPU6500_PWR_MGMT_1            0x6B
#define MPU6500_PWR_MGMT_2            0x6C
#define MPU6500_WHO_AM_I              0x75

#define MPU6500_ACCEL_XOUT_H          0x3B
#define MPU6500_GYRO_OUT_REGISTER     0x43

#define RAD_2_DEG                     57.29578f // [deg/rad]
#define CALIB_OFFSET_NB_MES           500
#define TEMP_LSB_2_DEGREE             333.87f   // [bit/celsius]
#define TEMP_OFFSET                   21.0f

#define DEFAULT_GYRO_COEFF            0.98f

class MPU6500_SPI {
public:
    MPU6500_SPI(SPIClass &s = SPI, int cs = 7);

    byte begin(int gyro_config_num = 1, int acc_config_num = 0);

    byte writeRegister(byte reg, byte data);
    byte readRegister(byte reg);
    void readRegisters(byte reg, uint8_t count, uint8_t *dest);

    void calcOffsets(bool is_calc_gyro = true, bool is_calc_acc = true);
    void calcGyroOffsets() { calcOffsets(true, false); }
    void calcAccOffsets() { calcOffsets(false, true); }

    void setCsPin(int cs) { csPin = cs; }
    int getCsPin() const { return csPin; }

    // Config Setters
    byte setGyroConfig(int config_num);
    byte setAccConfig(int config_num);

    void setGyroOffsets(float x, float y, float z);
    void setAccOffsets(float x, float y, float z);

    void setFilterGyroCoef(float gyro_coeff);
    void setFilterAccCoef(float acc_coeff);

    // Config Getters
    float getGyroXoffset() const { return gyroXoffset; }
    float getGyroYoffset() const { return gyroYoffset; }
    float getGyroZoffset() const { return gyroZoffset; }

    float getAccXoffset() const { return accXoffset; }
    float getAccYoffset() const { return accYoffset; }
    float getAccZoffset() const { return accZoffset; }

    float getFilterGyroCoef() const { return filterGyroCoef; }
    float getFilterAccCoef() const { return 1.0f - filterGyroCoef; }

    // Data Getters
    float getTemp() const { return temp; }

    float getAccX() const { return accX; }
    float getAccY() const { return accY; }
    float getAccZ() const { return accZ; }

    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }

    float getAccAngleX() const { return angleAccX; }
    float getAccAngleY() const { return angleAccY; }

    float getAngleX() const { return angleX; }
    float getAngleY() const { return angleY; }
    float getAngleZ() const { return angleZ; }

    // In-loop update
    void fetchData();
    void update();

    // Upside down mounting
    bool upsideDownMounting = false;

private:
    SPIClass *spi;
    int csPin;

    SPISettings spiSettingsLow;
    SPISettings spiSettingsHigh;

    float gyro_lsb_to_degsec;
    float acc_lsb_to_g;

    float gyroXoffset, gyroYoffset, gyroZoffset;
    float accXoffset, accYoffset, accZoffset;

    float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;

    unsigned long preInterval;
    float filterGyroCoef;
};

#endif // MPU6500_SPI_H
