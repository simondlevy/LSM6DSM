/* 
   Basic.cpp: LSM6DSM basic example

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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

// in timing.cpp
uint32_t millis(void);

// params
static const  LSM6DSM::Ascale_t ASCALE = LSM6DSM::AFS_2G;
static const  LSM6DSM::Gscale_t GSCALE = LSM6DSM::GFS_250DPS;
static const  LSM6DSM::Rate_t   RATE   = LSM6DSM::ODR_208Hz;

// Instantiate LSM6DSM class
static LSM6DSM lsm6dsm(ASCALE, RATE, GSCALE, RATE);

static void reportAcceleration(const char * dim, float val)
{
    printf("%s-acceleration: %f mg ", dim, 1000*val);
}

static void reportGyroRate(const char * dim, float val)
{
    printf("%s-gyro rate: %f degrees/sec ", dim, val);
}

void setup()
{
    if (!lsm6dsm.begin(0)) {
        while (true) {
            printf("Unable to connect to LSM6DSM\n");
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

        printf("\n");

        reportGyroRate("X", gx);
        reportGyroRate("Y", gy);
        reportGyroRate("Z", gz);

        printf("\n\n");

        millisPrev = millis();
    }
}
