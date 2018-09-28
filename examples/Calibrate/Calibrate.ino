/* 
   Calibrate.ino: LSM6DSM calibrating sketch

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

#include <Wire.h>

#define LED_PIN 38

//LSM6DSM definitions
#define INTERRUPT_PIN 2  // interrupt1 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
 AFS_2G, AFS_4G, AFS_8G, AFS_16G  
 GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
 AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
 GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
 */ 
static LSM6DSM::Ascale_t Ascale = LSM6DSM::AFS_2G;
static LSM6DSM::Gscale_t Gscale = LSM6DSM::GFS_245DPS;
static LSM6DSM::Rate_t AODR = LSM6DSM::ODR_833Hz;
static LSM6DSM::Rate_t GODR = LSM6DSM::ODR_833Hz;

// Random Bias to initiate the object
static float ACCEL_BIAS[3] = {0.0, 0.0, 0.0};
static float GYRO_BIAS[3]  = {0.0, 0.0, 0.0}; 

static LSM6DSM lsm6dsm(Ascale, Gscale, AODR, GODR, ACCEL_BIAS, GYRO_BIAS);

static void error(const char * errstr)
{
    while (true) {
        Serial.print("Error: ");
        Serial.println(errstr);
    }
}

// Real biases
float accel_bias[3];
float gyro_bias[3];
String sensor_msg;

void setup() 
{
    Serial.begin(115200);

    // Configure led
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // start with led off

    // Configure interrupt
    pinMode(INTERRUPT_PIN, INPUT);

    // Start I^2C 
    Wire.begin(TWI_PINS_20_21); // set master mode 
    Wire.setClock(400000); // I2C frequency at 400 kHz  
    delay(100);

    switch (lsm6dsm.begin()) {

        case LSM6DSM::ERROR_ID:
            error("chip ID");
            break;

        case LSM6DSM::ERROR_SELFTEST:
            //error("self-test");
            break;

         default:
            break;
    }

    delay(1000);

    // Turn on LED
    digitalWrite(LED_PIN, LOW);
    

    Serial.print("DO NOT move the IMU. The Calibration will start in: 3");
    delay(1000);
    Serial.print("\n");
    Serial.print(", 2");
    delay(1000);
    Serial.print("\n");
    Serial.print(", 1");
    delay(1000);
    Serial.print("\n");
    
    lsm6dsm.calibrate(gyro_bias, accel_bias);
    
    Serial.println("The IMU biases are the following:");
    
    for (int ii=0; ii<3; ++ii)
    {
      sensor_msg = "Giro. " + ii;
      Serial.println(sensor_msg + ": ");    
      Serial.println(gyro_bias[ii], 6);
      Serial.print("\n");
    }
    
    for (int ii=0; ii<3; ++ii)
    {
      sensor_msg = "Accel. " + ii ;
      Serial.println(sensor_msg + ": ");  
      Serial.println(accel_bias[ii], 6);
      Serial.print("\n");
    }
    
}

void loop() {

}


