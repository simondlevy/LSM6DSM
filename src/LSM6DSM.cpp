#include "LSM6DSM.h"

#include <CrossPlatformI2C_Core.h>

LSM6DSM::LSM6DSM(Ascale_t aScale, Rate_t aRate, Gscale_t gScale, Rate_t gRate)
{
    _aRes = getAres(aScale);
    _gRes = getGres(gScale);

    _aScale = aScale;
    _aRate  = aRate;
    _gScale = gScale;
    _gRate  = gRate;
}

LSM6DSM::Error_t LSM6DSM::begin(void)
{
    _i2c = cpi2c_open(LSM6DSM::ADDRESS);

    delay(100);

    if (getId() != LSM6DSM::ADDRESS) {
        return ERROR_ID;
    }

    // reset device
    writeRegister(CTRL3_C, readRegister(CTRL3_C) | 0x01);

    writeRegister(CTRL1_XL, _aRate << 4 | _aScale << 2);

    writeRegister(CTRL2_G, _gRate << 4 | _gScale << 2);

    // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
    writeRegister(CTRL3_C, readRegister(CTRL3_C) | 0x40 | 0x04); 
    // by default, interrupts active HIGH, push pull, little endian data 
    // (can be changed by writing to bits 5, 4, and 1, resp to above register)

    // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
    writeRegister(CTRL8_XL, 0x80 | 0x40 | 0x08 );

    // interrupt handling
    writeRegister(DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
    writeRegister(INT1_CTRL, 0x40);      // enable significant motion interrupts on INT1
    writeRegister(INT2_CTRL, 0x03);      // enable accel/gyro data ready interrupts on INT2  

    delay(100);

    if (!selfTest()) {
        return ERROR_SELFTEST;
    }

    return LSM6DSM::ERROR_NONE;
}

void LSM6DSM::readData(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
{
    ax = 0;
    ay = 0;
    az = 0;

    gx = 0;
    gy = 0;
    gz = 0;
}

uint8_t LSM6DSM::readRegister(uint8_t subAddress)
{
    uint8_t data=0;
    readRegisters(subAddress, 1, &data);
    return data;
}

void LSM6DSM::readRegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    cpi2c_readRegisters(_i2c, subAddress, count, dest);
}

void LSM6DSM::writeRegister(uint8_t subAddress, uint8_t data)
{
    cpi2c_writeRegister(_i2c, subAddress, data);
}

uint8_t LSM6DSM::getId(void)
{
    return readRegister(WHO_AM_I);  
}

bool LSM6DSM::selfTest(void)
{
    int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
    int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

    readData(temp);
    accelNom[0] = temp[4];
    accelNom[1] = temp[5];
    accelNom[2] = temp[6];
    gyroNom[0]  = temp[1];
    gyroNom[1]  = temp[2];
    gyroNom[2]  = temp[3];

    writeRegister(CTRL5_C, 0x01); // positive accel self test
    delay(100); // let accel respond
    readData(temp);
    accelPTest[0] = temp[4];
    accelPTest[1] = temp[5];
    accelPTest[2] = temp[6];

    writeRegister(CTRL5_C, 0x03); // negative accel self test
    delay(100); // let accel respond
    readData(temp);
    accelNTest[0] = temp[4];
    accelNTest[1] = temp[5];
    accelNTest[2] = temp[6];

    writeRegister(CTRL5_C, 0x04); // positive gyro self test
    delay(100); // let gyro respond
    readData(temp);
    gyroPTest[0] = temp[1];
    gyroPTest[1] = temp[2];
    gyroPTest[2] = temp[3];

    writeRegister(CTRL5_C, 0x0C); // negative gyro self test
    delay(100); // let gyro respond
    readData(temp);
    gyroNTest[0] = temp[1];
    gyroNTest[1] = temp[2];
    gyroNTest[2] = temp[3];

    writeRegister(CTRL5_C, 0x00); // normal mode
    delay(100); // let accel and gyro respond

    Serial.println("Self Test:");
    for (uint8_t k=0; k<3; ++k) {
        Serial.print("+A results:"); 
        Serial.println((accelPTest[k] - accelNom[k]) * _aRes * 1000.0); 
        Serial.print("-A results:"); 
        Serial.println((accelNTest[k] - accelNom[k]) * _aRes * 1000.0);
        Serial.print("+Gx results:"); 
        Serial.println((gyroPTest[k] - gyroNom[k]) * _gRes); 
        Serial.print("-Gx results:"); 
        Serial.println((gyroNTest[k] - gyroNom[k]) * _gRes);
    }
    Serial.println("Accel should be between 20 and 80 dps");
    Serial.println("Gyro should be between 90 and 1700 mg");

    return true;
}

void LSM6DSM::readData(int16_t * data)
{
    uint8_t rawData[14];  // x/y/z accel register data stored here
    readRegisters(OUT_TEMP_L, 14, rawData);  // Read the 14 raw data registers into data array
    data[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    data[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    data[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    data[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;   
    data[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
    data[5] = ((int16_t)rawData[11] << 8) | rawData[10] ;  
    data[6] = ((int16_t)rawData[13] << 8) | rawData[12] ; 
}

float LSM6DSM::getAres(Ascale_t ascale) 
{
    switch (ascale) {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            return 2.0f/32768.0f;
            break;
        case AFS_4G:
            return 4.0f/32768.0f;
            break;
        case AFS_8G:
            return 8.0f/32768.0f;
            break;
        case AFS_16G:
            return 16.0f/32768.0f;
            break;
    }

    return 0;
}

float LSM6DSM::getGres(Gscale_t gscale) 
{
    switch (gscale)  {

        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        case GFS_250DPS:
            return 245.0f/32768.0f;;
            break;
        case GFS_500DPS:
            return 500.0f/32768.0f;;
            break;
        case GFS_1000DPS:
            return 1000.0f/32768.0f;
            break;
        case GFS_2000DPS:
            return 2000.0f/32768.0f;
            break;
    }

    return 0;
}

