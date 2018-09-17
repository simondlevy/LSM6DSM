/* 
   Interrupt.ino: LSM6DSM interrupt-driven example

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

#include <Arduino.h>
#include <Wire.h>

// Parameter settings
static LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_2G;
static LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_245DPS;
static LSM6DSM::Rate_t   AODR   = LSM6DSM::ODR_833Hz;
static LSM6DSM::Rate_t   GODR   = LSM6DSM::ODR_833Hz;

static uint8_t myLed = 38;
static uint8_t LSM6DSM_intPin1 = 2;  

static bool newLSM6DSMData;

static LSM6DSM lsm6dsm(Ascale, Gscale, AODR, GODR);

static void myinthandler1()
{
    newLSM6DSMData = true;
}

static void error(const char * msg)
{
    while (true) {
        Serial.print("Error: ");
        Serial.println(msg);
    }
}

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

void setup() {

    // put your setup code here, to run once:
    Serial.begin(115200);
    delay(1000);

    // Configure led
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH); // start with led off

    pinMode(LSM6DSM_intPin1, INPUT);

    Wire.begin(TWI_PINS_20_21); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(1000);

    switch (lsm6dsm.begin()) {

        case LSM6DSM::ERROR_CONNECT:
            error("no connection");
            break;

        case LSM6DSM::ERROR_ID:
            error("bad ID");
            break;

        case LSM6DSM::ERROR_SELFTEST:
            //error("failed self-test");
            break;

         case LSM6DSM::ERROR_NONE:
            break;

    }

    // Un-comment these lines to calibrate the IMU
    //Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
    //delay(4000);
    //lsm6dsm.calibrate();

    attachInterrupt(LSM6DSM_intPin1, myinthandler1, RISING);  // define interrupt for intPin2 output of LSM6DSM

    // Turn on the LED
    digitalWrite(myLed, LOW);

}

void loop() 
{
    // If intPin goes high, either all data registers have new data
    if(newLSM6DSMData) {   // On interrupt, read data

        newLSM6DSMData = false;     // reset newData flag

        float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

        lsm6dsm.readData(ax, ay, az, gx, gy, gz);

        reportAcceleration("X", ax);
        reportAcceleration("Y", ay);
        reportAcceleration("Z", az);

        Serial.println();

        reportGyroRate("X", gx);
        reportGyroRate("Y", gy);
        reportGyroRate("Z", gz);

        Serial.println("\n");
    }

    // Slow down, you move too fast
    delay(100);
}


