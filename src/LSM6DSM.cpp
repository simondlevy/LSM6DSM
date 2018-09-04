/* 
   LSM6DSM.cpp: Implementation of LSM6DSM class

   Copyright (C) 2018 Simon D. Levy

   Adapted from https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB

   This file is part of LSM6DSM.

   LSM6DSM is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   LSM6DSM is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with LSM6DSM.  If not, see <http://www.gnu.org/licenses/>.
*/

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

bool LSM6DSM::begin(void)
{
    _i2c = cpi2c_open(ADDRESS);

    delay(100);

    if (getId() != ADDRESS) {
        return false;
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

    computeBiases();

    return true;
}

void LSM6DSM::readData(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
{
    int16_t data[7];

    readData(data);

    ax = (float)data[4]*_aRes - _accelBias[0]; 
    ay = (float)data[5]*_aRes - _accelBias[1];   
    az = (float)data[6]*_aRes - _accelBias[2];  

    gx = (float)data[1]*_gRes - _gyroBias[0];  
    gy = (float)data[2]*_gRes - _gyroBias[1];  
    gz = (float)data[3]*_gRes - _gyroBias[2]; 
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



void LSM6DSM::computeBiases(void)
{
    int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

    for (int k = 0; k < 128; ++k) {
        readData(temp);
        sum[1] += temp[1];
        sum[2] += temp[2];
        sum[3] += temp[3];
        sum[4] += temp[4];
        sum[5] += temp[5];
        sum[6] += temp[6];
        delay(50);
    }

    _gyroBias[0] = sum[1]*_gRes/128.0f;
    _gyroBias[1] = sum[2]*_gRes/128.0f;
    _gyroBias[2] = sum[3]*_gRes/128.0f;
    _accelBias[0] = sum[4]*_aRes/128.0f;
    _accelBias[1] = sum[5]*_aRes/128.0f;
    _accelBias[2] = sum[6]*_aRes/128.0f;

    if(_accelBias[0] > 0.8f)  {_accelBias[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(_accelBias[0] < -0.8f) {_accelBias[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(_accelBias[1] > 0.8f)  {_accelBias[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(_accelBias[1] < -0.8f) {_accelBias[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(_accelBias[2] > 0.8f)  {_accelBias[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
    if(_accelBias[2] < -0.8f) {_accelBias[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

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

