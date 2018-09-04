#include "LSM6DSM.h"

LSM6DSM::LSM6DSM(Ascale_t ascale, Gscale_t gscale, Grate_t grate)
{
    (void)ascale;
    (void)gscale;
    (void)grate;
}

LSM6DSM::Error_t LSM6DSM::begin(void)
{
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
