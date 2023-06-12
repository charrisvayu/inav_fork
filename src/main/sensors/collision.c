/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/collision/collision.h"
#include "drivers/collision/collision_lw20c_i2c.h"

#include "navigation/navigation_private.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"

#include "sensors/collision.h"

#include "io/rangefinder.h"

#include "scheduler/scheduler.h"

#define USE_COLLISION

collision_t collision_1;

#define COLLISION_HARDWARE_TIMEOUT_MS         500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define COLLISION_DYNAMIC_THRESHOLD           600     //Used to determine max. usable COLLISION distance
#define COLLISION_DYNAMIC_FACTOR              75

#ifdef USE_COLLISION
PG_REGISTER_WITH_RESET_TEMPLATE(collisionConfig_t, collisionConfig, PG_COLLISION_CONFIG, 3);

PG_RESET_TEMPLATE(collisionConfig_t, collisionConfig,
    .collision_hardware = SETTING_COLLISION_HARDWARE_DEFAULT,
    .use_median_filtering_for_collision = SETTING_COLLISION_MEDIAN_FILTER_DEFAULT,
);

/*
 * Detect which collision is present
 */
static bool collisionDetect(collisionDev_t * dev, uint8_t collisionHardwareToUse)
{
    collisionType_e collisionHardware = COLLISION_LW20CI2C;
    requestedSensors[SENSOR_INDEX_COLLISION] = collisionHardwareToUse;

    switch (collisionHardwareToUse) {
            case COLLISION_LW20CI2C:
#ifdef USE_COLLISION_LW20C_I2C
            if (collisionLw20cDetect(dev)) {
                collisionHardware = COLLISION_LW20CI2C;
                rescheduleTask(TASK_COLLISION, TASK_PERIOD_MS(COLLISION_LW20C_I2C_TASK_PERIOD_MS));
            }
#endif
            break;

            case COLLISION_NONE:
            collisionHardware = COLLISION_NONE;
            break;
    }

    if (collisionHardware == COLLISION_NONE) {
        sensorsClear(SENSOR_COLLISION);
        return false;
    }

    detectedSensors[SENSOR_INDEX_COLLISION] = collisionHardware;
    sensorsSet(SENSOR_COLLISION);
    return true;
}

bool collisionInit(collision_t* coll)
{
    if (!collisionDetect(&coll->dev, collisionConfig()->collision_hardware)) {
        return false;
    }

    coll->dev.init(&coll->dev);
    coll->rawDistance = COLLISION_OUT_OF_RANGE;
    coll->calculatedDistance = COLLISION_OUT_OF_RANGE;
    coll->lastValidResponseTimeMs = millis();

    return true;
}

static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 3
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > COLLISION_OUT_OF_RANGE) {// only accept samples that are in range
        filterSamples[filterSampleIndex] = newReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter3(filterSamples) : newReading;
}

/*
 * This is called periodically by the scheduler
 */
timeDelta_t collisionUpdate(collision_t* coll)
{
    if (coll->dev.update) {
        coll->dev.update(&coll->dev);
    }

    return MS2US(coll->dev.delayMs);
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, COLLISION_OUT_OF_RANGE is returned.
 */
bool collisionProcess(collision_t* coll)
{
    if (coll->dev.read) {
        const int32_t distance = coll->dev.read(&coll->dev);

        // if (distance < 200.0 && distance >= 0.0) {
        //     posControl.flags.isCollisionDetected = true;
        // } else {
        //     posControl.flags.isCollisionDetected = false;
        // }

        // If driver reported no new measurement - don't do anything
        if (distance == COLLISION_NO_NEW_DATA) {
            return false;
        }

        if (distance >= 0) {
            coll->lastValidResponseTimeMs = millis();
            coll->rawDistance = distance;

            if (collisionConfig()->use_median_filtering_for_collision) {
                coll->rawDistance = applyMedianFilter(coll->rawDistance);
            }
        }
        else if (distance == COLLISION_OUT_OF_RANGE) {
            coll->lastValidResponseTimeMs = millis();
            coll->rawDistance = COLLISION_OUT_OF_RANGE;
        }
        else {
            // Invalid response / hardware failure
            coll->rawDistance = COLLISION_HARDWARE_FAILURE;
        }
    }
    else {
        // Bad configuration
        coll->rawDistance = COLLISION_OUT_OF_RANGE;
    }

    /**
    * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
    * the altitude. Returns the computed altitude in centimeters.
    *
    * When the ground is too far away or the tilt is too large, COLLISION_OUT_OF_RANGE is returned.
    */

   coll->calculatedDistance = coll->rawDistance;

    return true;
}

/**
 * Get the latest altitude that was computed, or COLLISION_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t collisionGetLatestDistance(collision_t* coll)
{
    return coll->calculatedDistance;
}

int32_t collisionGetLatestRawDistance(collision_t* coll) {
    return coll->rawDistance;
}

bool collisionIsHealthy(collision_t* coll)
{
    return (millis() - coll->lastValidResponseTimeMs) < COLLISION_HARDWARE_TIMEOUT_MS;
}
#endif
