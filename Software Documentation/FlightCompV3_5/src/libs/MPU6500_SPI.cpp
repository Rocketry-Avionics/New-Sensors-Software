#include "MPU6500_SPI.h"

MPU6500_SPI::MPU6500_SPI(SPIClass &s, int cs) {
    spi = &s;
    csPin = cs;

    // 1 MHz for setup/registers, 20 MHz for high-speed burst reads
    spiSettingsLow = SPISettings(1000000, MSBFIRST, SPI_MODE0);
    spiSettingsHigh = SPISettings(20000000, MSBFIRST, SPI_MODE0);

    filterGyroCoef = DEFAULT_GYRO_COEFF;
    gyroXoffset = 0.0f;
    gyroYoffset = 0.0f;
    gyroZoffset = 0.0f;
    accXoffset = 0.0f;
    accYoffset = 0.0f;
    accZoffset = 0.0f;

    angleX = 0.0f;
    angleY = 0.0f;
    angleZ = 0.0f;
    preInterval = 0;
}

byte MPU6500_SPI::writeRegister(byte reg, byte data) {
    spi->beginTransaction(spiSettingsLow);
    digitalWrite(csPin, LOW);
    spi->transfer(reg & 0x7F); // Write: bit 7 = 0
    spi->transfer(data);
    digitalWrite(csPin, HIGH);
    spi->endTransaction();
    return 0;
}

byte MPU6500_SPI::readRegister(byte reg) {
    spi->beginTransaction(spiSettingsLow);
    digitalWrite(csPin, LOW);
    spi->transfer(reg | 0x80); // Read: bit 7 = 1
    byte val = spi->transfer(0x00);
    digitalWrite(csPin, HIGH);
    spi->endTransaction();
    return val;
}

void MPU6500_SPI::readRegisters(byte reg, uint8_t count, uint8_t *dest) {
    spi->beginTransaction(spiSettingsHigh);
    digitalWrite(csPin, LOW);
    spi->transfer(reg | 0x80); // Read: bit 7 = 1
    for (uint8_t i = 0; i < count; i++) {
        dest[i] = spi->transfer(0x00);
    }
    digitalWrite(csPin, HIGH);
    spi->endTransaction();
}

byte MPU6500_SPI::begin(int gyro_config_num, int acc_config_num) {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    delay(10);

    // Reset device
    writeRegister(MPU6500_PWR_MGMT_1, 0x80);
    delay(100);

    // Disable I2C Interface, enforce SPI mode
    writeRegister(MPU6500_USER_CTRL, 0x10);
    delay(10);

    // Wake up device & select auto best clock source (PLL with gyro)
    writeRegister(MPU6500_PWR_MGMT_1, 0x01);
    delay(10);

    // Verify WHO_AM_I register (0x70 for MPU6500, 0x71 for MPU9250, 0x73 for MPU9255)
    byte whoami = readRegister(MPU6500_WHO_AM_I);
    if (whoami != 0x70 && whoami != 0x71 && whoami != 0x73 && whoami != 0x68) {
        Serial.print("MPU6500 SPI WHO_AM_I mismatch! Read: 0x");
        Serial.println(whoami, HEX);
        return 1; // Device not found
    }

    // Set sample rate divider: SMPLRT_DIV = 0 (1kHz / (1+0) = 1kHz)
    writeRegister(MPU6500_SMPLRT_DIV, 0x00);

    // Configure DLPF: CONFIG = 0 (Gyro: 250Hz BW, 0.97ms delay)
    writeRegister(MPU6500_CONFIG, 0x00);

    // Configure ranges
    setGyroConfig(gyro_config_num);
    setAccConfig(acc_config_num);

    preInterval = millis();
    return 0; // Success
}

byte MPU6500_SPI::setGyroConfig(int config_num) {
    byte status = writeRegister(MPU6500_GYRO_CONFIG, (config_num & 0x03) << 3);
    switch (config_num) {
        case 0: gyro_lsb_to_degsec = 131.0f; break; // +/- 250 deg/s
        case 1: gyro_lsb_to_degsec = 65.5f;  break; // +/- 500 deg/s
        case 2: gyro_lsb_to_degsec = 32.8f;  break; // +/- 1000 deg/s
        case 3: gyro_lsb_to_degsec = 16.4f;  break; // +/- 2000 deg/s
        default: gyro_lsb_to_degsec = 65.5f; break;
    }
    return status;
}

byte MPU6500_SPI::setAccConfig(int config_num) {
    byte status = writeRegister(MPU6500_ACCEL_CONFIG, (config_num & 0x03) << 3);
    switch (config_num) {
        case 0: acc_lsb_to_g = 16384.0f; break; // +/- 2g
        case 1: acc_lsb_to_g = 8192.0f;  break; // +/- 4g
        case 2: acc_lsb_to_g = 4096.0f;  break; // +/- 8g
        case 3: acc_lsb_to_g = 2048.0f;  break; // +/- 16g
        default: acc_lsb_to_g = 16384.0f; break;
    }
    return status;
}

void MPU6500_SPI::setGyroOffsets(float x, float y, float z) {
    gyroXoffset = x;
    gyroYoffset = y;
    gyroZoffset = z;
}

void MPU6500_SPI::setAccOffsets(float x, float y, float z) {
    accXoffset = x;
    accYoffset = y;
    accZoffset = z;
}

void MPU6500_SPI::setFilterGyroCoef(float gyro_coeff) {
    if (gyro_coeff >= 0.0f && gyro_coeff <= 1.0f) {
        filterGyroCoef = gyro_coeff;
    }
}

void MPU6500_SPI::setFilterAccCoef(float acc_coeff) {
    setFilterGyroCoef(1.0f - acc_coeff);
}

void MPU6500_SPI::calcOffsets(bool is_calc_gyro, bool is_calc_acc) {
    if (is_calc_gyro) {
        setGyroOffsets(0.0f, 0.0f, 0.0f);
    }
    if (is_calc_acc) {
        setAccOffsets(0.0f, 0.0f, 0.0f);
    }

    float ag[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
        fetchData();
        ag[0] += accX;
        ag[1] += accY;
        ag[2] += (accZ - 1.0f); // Gravity along Z
        ag[3] += gyroX;
        ag[4] += gyroY;
        ag[5] += gyroZ;
        delay(2);
    }

    if (is_calc_acc) {
        accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
        accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
        accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
    }
    if (is_calc_gyro) {
        gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
        gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
        gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
    }
}

void MPU6500_SPI::fetchData() {
    uint8_t buffer[14];
    readRegisters(MPU6500_ACCEL_XOUT_H, 14, buffer);

    int16_t rawAccX  = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAccY  = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAccZ  = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t rawTemp  = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t rawGyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t rawGyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);

    accX = ((float)rawAccX / acc_lsb_to_g) - accXoffset;
    accY = ((float)rawAccY / acc_lsb_to_g) - accYoffset;
    accZ = ((float)rawAccZ / acc_lsb_to_g) - accZoffset;

    temp = (((float)rawTemp - 0.0f) / TEMP_LSB_2_DEGREE) + TEMP_OFFSET;

    gyroX = ((float)rawGyroX / gyro_lsb_to_degsec) - gyroXoffset;
    gyroY = ((float)rawGyroY / gyro_lsb_to_degsec) - gyroYoffset;
    gyroZ = ((float)rawGyroZ / gyro_lsb_to_degsec) - gyroZoffset;
}

void MPU6500_SPI::update() {
    fetchData();

    unsigned long currentMillis = millis();
    float dt = (float)(currentMillis - preInterval) * 0.001f;
    preInterval = currentMillis;

    // Prevent huge dt on first execution or delay
    if (dt > 0.2f || dt <= 0.0f) {
        dt = 0.004f; // fallback to nominal ~250Hz period
    }

    float sgZ = (accZ >= 0.0f) ? 1.0f : -1.0f;
    if (upsideDownMounting) {
        sgZ = -sgZ;
    }

    angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) * RAD_2_DEG;
    angleAccY = atan2(-accX, sqrt(accZ * accZ + accY * accY)) * RAD_2_DEG;

    angleX = filterGyroCoef * (angleX + gyroX * dt) + (1.0f - filterGyroCoef) * angleAccX;
    angleY = filterGyroCoef * (angleY + gyroY * dt) + (1.0f - filterGyroCoef) * angleAccY;
    angleZ = angleZ + gyroZ * dt;
}
