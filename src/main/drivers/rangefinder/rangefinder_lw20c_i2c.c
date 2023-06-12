/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

// #if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_LW20C_I2C)

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lw20c_i2c.h"

#define LW20C_I2C_MAX_RANGE_CM 10000
#define LW20C_I2C_MAX_RANGE_MM LW20C_I2C_MAX_RANGE_CM * 10

#define LW20C_I2C_DETECTION_CONE_DECIDEGREES 900
#define LW20C_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES 900

#define LW20C_MEASUREMENT_REGISTER 0x00

#define LW20C_I2C_ADDRESS 0x66 // (factory default -- this is permanently configurable via the lightware studio software)

volatile int32_t lw20cMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static bool isLw20cResponding = true;

static void lw20ci2cInit(rangefinderDev_t *rangefinderDev)
{
    UNUSED(rangefinderDev);
    delay(100);
}

void lw20ci2cUpdate(rangefinderDev_t *rangefinderDev)
{
    uint8_t buffer[2];
    uint16_t distance_cm;

    isLw20cResponding = busReadBuf(rangefinderDev->busDev, LW20C_MEASUREMENT_REGISTER, buffer, sizeof(buffer));

    if (!isLw20cResponding) {
        return;
    }

    distance_cm = (buffer[0] << 8) | buffer[1];

    if (distance_cm >= LW20C_I2C_MAX_RANGE_CM) {
        lw20cMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
        return;
    }

    lw20cMeasurementCm = (int32_t)distance_cm;
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t lw20ci2cGetDistance(rangefinderDev_t *rangefinderDev)
{
    UNUSED(rangefinderDev);

    if (isLw20cResponding) {
        return lw20cMeasurementCm;
    } else {
        return RANGEFINDER_HARDWARE_FAILURE;
    }
}

static bool deviceDetect(rangefinderDev_t *rangefinderDev)
{
    uint8_t buffer[2];
    return busReadBuf(rangefinderDev->busDev, LW20C_MEASUREMENT_REGISTER, buffer, sizeof(buffer));
}

bool lw20cDetect(rangefinderDev_t *rangefinderDev, bool useForCollisionDetection)
{
    if (!useForCollisionDetection) {
        rangefinderDev->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_LW20C_I2C, 0, OWNER_RANGEFINDER);
    }
    
    if (rangefinderDev->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(rangefinderDev)) {
        busDeviceDeInit(rangefinderDev->busDev);
        return false;
    }

    rangefinderDev->delayMs = RANGEFINDER_LW20C_I2C_TASK_PERIOD_MS;
    rangefinderDev->maxRangeCm = LW20C_I2C_MAX_RANGE_CM;
    rangefinderDev->detectionConeDeciDegrees = LW20C_I2C_DETECTION_CONE_DECIDEGREES;
    rangefinderDev->detectionConeExtendedDeciDegrees = LW20C_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinderDev->init = &lw20ci2cInit;
    rangefinderDev->update = &lw20ci2cUpdate;
    rangefinderDev->read = &lw20ci2cGetDistance;

    return true;
}
// #endif
