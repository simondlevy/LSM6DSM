/* 
   Basic.ino: LSM6DSM basic example

   Copyright (C) 2018 Simon D. Levy

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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#define NOSTOP I2C_NOSTOP
#else
#include <Wire.h>
#define NOSTOP false
#endif

// params
static const  LSM6DSM::Ascale_t ASCALE = LSM6DSM::AFS_2G;
static const  LSM6DSM::Gscale_t GSCALE = LSM6DSM::GFS_250DPS;
static const  LSM6DSM::Rate_t   RATE   = LSM6DSM::ODR_208Hz;

// Instantiate LSM6DSM class
static LSM6DSM lsm6dsm(ASCALE, RATE, GSCALE, RATE);

static void reportAcceleration(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-acceleration: ");
    Serial.print(1000*val);
    Serial.print(" mg "); 
}

static void reportGyroRate(const char * dim, float val)
{
    Serial.print(dim);
    Serial.print("-gyro rate: ");
    Serial.print(val, 1);
    Serial.print(" degrees/sec "); 
}

void setup()
{
    Serial.begin(115200);

#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
#else
    Wire.begin();
#endif

    delay(100);

    if (!lsm6dsm.begin()) {
        while (true) {
            Serial.println("Unable to connect to LSM6DSM");
        }
    }
}

void loop()
{  
    static uint32_t millisPrev;
    static float ax, ay, az;
    static float gx, gy, gz;

    // If data ready bit set, all data registers have new data
    if (lsm6dsm.checkNewData()) {  // check if data ready interrupt
        lsm6dsm.readData(ax, ay, az, gx, gy, gz);
    }  

    // Report data periodically
    if ((millis() - millisPrev) > 100) {

        reportAcceleration("X", ax);
        reportAcceleration("Y", ay);
        reportAcceleration("Z", az);

        Serial.println();

        reportGyroRate("X", gx);
        reportGyroRate("Y", gy);
        reportGyroRate("Z", gz);

        Serial.println("\n");

        millisPrev = millis();
    }
}
