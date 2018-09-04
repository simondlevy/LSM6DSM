#include "LSM6DSM.h"

#include <CrossPlatformI2C_Core.h>

LSM6DSM::LSM6DSM(Ascale_t ascale, Rate_t arate, Gscale_t gscale, Rate_t grate)
{
    _ascale = ascale;
    _arate  = arate;
    _gscale = gscale;
    _grate  = grate;
}

LSM6DSM::Error_t LSM6DSM::begin(void)
{
    _i2c = cpi2c_open(LSM6DSM::ADDRESS);

    delay(100);

    if (getId() != LSM6DSM::ADDRESS) {
        return ERROR_ID;
    }

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
    cpi2c_readRegisters(_i2c, subAddress, 1, &data);
    return data;
}

uint8_t LSM6DSM::getId(void)
{
    return readRegister(LSM6DSM::WHO_AM_I);  
}

bool LSM6DSM::selfTest(void)
{
    return false;
}
