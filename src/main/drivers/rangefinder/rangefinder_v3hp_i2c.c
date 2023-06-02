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

#if defined(USE_RANGEFINDER) && defined(USE_RANGEFINDER_V3HP_I2C)

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_v3hp_i2c.h"

#define V3HP_I2C_MAX_RANGE_CM 4000
#define V3HP_I2C_MAX_RANGE_MM V3HP_I2C_MAX_RANGE_CM * 10
#define V3HP_I2C_DETECTION_CONE_DECIDEGREES 900
#define V3HP_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES 900

#define V3HP_I2C_ADDRESS 0x62

volatile int32_t v3hpMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
static bool isV3hpResponding = true;

static void v3hpi2cInit(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);
    delay(100);
}

void v3hpi2cUpdate(rangefinderDev_t *rangefinder)
{
    uint8_t lsb;
    uint8_t msb;
    uint8_t response;
    uint16_t distance_cm;

    isV3hpResponding = busWrite(rangefinder->busDev, 0x00, 0x04);

    if (!isV3hpResponding) {
        return;
    }

    isV3hpResponding = busRead(rangefinder->busDev, 0x01, &response);

    if (!isV3hpResponding) {
        return;
    }

    isV3hpResponding = busRead(rangefinder->busDev, 0x0f, &msb);

    if (!isV3hpResponding) {
        return;
    }

    isV3hpResponding = busRead(rangefinder->busDev, 0x10, &lsb);

    if (!isV3hpResponding) {
        return;
    }

    distance_cm = (msb << 8) | (lsb);

    if (distance_cm >= V3HP_I2C_MAX_RANGE_CM) {
        v3hpMeasurementCm = RANGEFINDER_OUT_OF_RANGE;
        return;
    }

    v3hpMeasurementCm = (int32_t)distance_cm;
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t v3hpi2cGetDistance(rangefinderDev_t *rangefinder)
{
    UNUSED(rangefinder);

    if (isV3hpResponding) {
        return v3hpMeasurementCm;
    } else {
        return RANGEFINDER_HARDWARE_FAILURE;
    }
}

static bool deviceDetect(rangefinderDev_t *rangefinder)
{
    uint8_t response;
    return busRead(rangefinder->busDev, 0x01, &response);
}

bool v3hpDetect(rangefinderDev_t *rangefinder)
{
    rangefinder->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_V3HP_I2C, 0, OWNER_RANGEFINDER);
    if (rangefinder->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(rangefinder)) {
        busDeviceDeInit(rangefinder->busDev);
        return false;
    }

    rangefinder->delayMs = RANGEFINDER_V3HP_I2C_TASK_PERIOD_MS;
    rangefinder->maxRangeCm = V3HP_I2C_MAX_RANGE_CM;
    rangefinder->detectionConeDeciDegrees = V3HP_I2C_DETECTION_CONE_DECIDEGREES;
    rangefinder->detectionConeExtendedDeciDegrees = V3HP_I2C_DETECTION_CONE_EXTENDED_DECIDEGREES;

    rangefinder->init = &v3hpi2cInit;
    rangefinder->update = &v3hpi2cUpdate;
    rangefinder->read = &v3hpi2cGetDistance;

    return true;
}
#endif
