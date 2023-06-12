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
#include "drivers/collision/collision.h"
#include "drivers/collision/collision_lw20c_i2c.h"

#define COLLISION_LW20C_I2C_MAX_RANGE_CM 10000
#define COLLISION_LW20C_I2C_MAX_RANGE_MM LW20C_I2C_MAX_RANGE_CM * 10

#define COLLISION_LW20C_MEASUREMENT_REGISTER 0x00

volatile int32_t collisionLw20cMeasurementCm = COLLISION_LW20C_I2C_MAX_RANGE_CM;
static bool isCollisionLw20cResponding = true;

static void collisionLw20ci2cInit(collisionDev_t *collisionDev)
{
    UNUSED(collisionDev);
    delay(100);
}

void collisionLw20ci2cUpdate(collisionDev_t *collisionDev)
{
    uint8_t buffer[2];
    uint16_t distance_cm;

    isCollisionLw20cResponding = busReadBuf(collisionDev->busDev, COLLISION_LW20C_MEASUREMENT_REGISTER, buffer, sizeof(buffer));

    if (!isCollisionLw20cResponding) {
        return;
    }

    distance_cm = (buffer[0] << 8) | buffer[1];

    if (distance_cm > COLLISION_LW20C_I2C_MAX_RANGE_CM) {
        collisionLw20cMeasurementCm = COLLISION_LW20C_I2C_MAX_RANGE_CM;
        return;
    }

    collisionLw20cMeasurementCm = (int32_t)distance_cm;
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t collisionLw20ci2cGetDistance(collisionDev_t *collisionDev)
{
    UNUSED(collisionDev);

    if (isCollisionLw20cResponding) {
        return collisionLw20cMeasurementCm;
    } else {
        return COLLISION_HARDWARE_FAILURE;
    }
}

static bool deviceDetect(collisionDev_t *collisionDev)
{
    uint8_t buffer[2];
    return busReadBuf(collisionDev->busDev, COLLISION_LW20C_MEASUREMENT_REGISTER, buffer, sizeof(buffer));
}

bool collisionLw20cDetect(collisionDev_t *collisionDev)
{
    collisionDev->busDev = busDeviceInit(BUSTYPE_I2C, DEVHW_LW20C_I2C_COLLISION, 0, OWNER_COLLISION);
    
    if (collisionDev->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(collisionDev)) {
        busDeviceDeInit(collisionDev->busDev);
        return false;
    }

    collisionDev->delayMs = COLLISION_LW20C_I2C_TASK_PERIOD_MS;
    collisionDev->maxRangeCm = COLLISION_LW20C_I2C_MAX_RANGE_CM;

    collisionDev->init = &collisionLw20ci2cInit;
    collisionDev->update = &collisionLw20ci2cUpdate;
    collisionDev->read = &collisionLw20ci2cGetDistance;

    return true;
}
// #endif
