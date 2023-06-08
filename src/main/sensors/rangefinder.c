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
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_srf10.h"
#include "drivers/rangefinder/rangefinder_vl53l0x.h"
#include "drivers/rangefinder/rangefinder_vl53l1x.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "drivers/rangefinder/rangefinder_us42.h"
#include "drivers/rangefinder/rangefinder_tof10120_i2c.h"
#include "drivers/rangefinder/rangefinder_v3hp_i2c.h"
#include "drivers/rangefinder/rangefinder_lw20c_i2c.h"

#include "navigation/navigation_private.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "sensors/sensors.h"
#include "sensors/rangefinder.h"
#include "sensors/battery.h"

#include "io/rangefinder.h"

#include "scheduler/scheduler.h"

rangefinder_t rangefinder;
rangefinder_t rangefinder_1;
// rangefinder_t rangefinder_right;
// rangefinder_t rangefinder_rear;
// rangefinder_t rangefinder_left;

#define RANGEFINDER_HARDWARE_TIMEOUT_MS         500     // Accept 500ms of non-responsive sensor, report HW failure otherwise

#define RANGEFINDER_DYNAMIC_THRESHOLD           600     //Used to determine max. usable rangefinder disatance
#define RANGEFINDER_DYNAMIC_FACTOR              75

#ifdef USE_RANGEFINDER
PG_REGISTER_WITH_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 3);

PG_RESET_TEMPLATE(rangefinderConfig_t, rangefinderConfig,
    .rangefinder_hardware = SETTING_RANGEFINDER_HARDWARE_DEFAULT,
    .use_median_filtering = SETTING_RANGEFINDER_MEDIAN_FILTER_DEFAULT,
);

/*
 * Detect which rangefinder is present
 */
static bool rangefinderDetect(rangefinderDev_t * dev, uint8_t rangefinderHardwareToUse, int8_t rangefinder_position)
{
    rangefinderType_e rangefinderHardware = RANGEFINDER_NONE;
    requestedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardwareToUse;

    switch (rangefinderHardwareToUse) {
        case RANGEFINDER_SRF10:
#ifdef USE_RANGEFINDER_SRF10
            if (srf10Detect(dev)) {
                rangefinderHardware = RANGEFINDER_SRF10;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_SRF10_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_VL53L0X:
#if defined(USE_RANGEFINDER_VL53L0X)
            if (vl53l0xDetect(dev)) {
                rangefinderHardware = RANGEFINDER_VL53L0X;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VL53L0X_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_VL53L1X:
#if defined(USE_RANGEFINDER_VL53L1X)
            if (vl53l1xDetect(dev)) {
                rangefinderHardware = RANGEFINDER_VL53L1X;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VL53L1X_TASK_PERIOD_MS));
            }
#endif
            break;

        case RANGEFINDER_MSP:
#if defined(USE_RANGEFINDER_MSP)
            if (virtualRangefinderDetect(dev, &rangefinderMSPVtable)) {
                rangefinderHardware = RANGEFINDER_MSP;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VIRTUAL_TASK_PERIOD_MS));
            }
#endif
            break;

        case RANGEFINDER_BENEWAKE:
#if defined(USE_RANGEFINDER_BENEWAKE)
            if (virtualRangefinderDetect(dev, &rangefinderBenewakeVtable)) {
                rangefinderHardware = RANGEFINDER_BENEWAKE;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VIRTUAL_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_US42:
#ifdef USE_RANGEFINDER_US42
            if (us42Detect(dev)) {
                rangefinderHardware = RANGEFINDER_US42;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_US42_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_TOF10102I2C:
#ifdef USE_RANGEFINDER_TOF10120_I2C
            if (tof10120Detect(dev)) {
                rangefinderHardware = RANGEFINDER_TOF10102I2C;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_TOF10120_I2C_TASK_PERIOD_MS));
            }
#endif
            break;
            
            case RANGEFINDER_FAKE:
#if defined(USE_RANGEFINDER_FAKE)
            if(virtualRangefinderDetect(dev, &rangefinderFakeVtable)) {
                rangefinderHardware = RANGEFINDER_FAKE;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_VIRTUAL_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_V3HPI2C:
#ifdef USE_RANGEFINDER_V3HP_I2C
            if (v3hpDetect(dev)) {
                rangefinderHardware = RANGEFINDER_V3HPI2C;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_V3HP_I2C_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_LW20CI2C:
#ifdef USE_RANGEFINDER_LW20C_I2C
            if (lw20cDetect(dev, 0)) { // can set position to 0 to test
                rangefinderHardware = RANGEFINDER_LW20CI2C;
                rescheduleTask(TASK_RANGEFINDER, TASK_PERIOD_MS(RANGEFINDER_LW20C_I2C_TASK_PERIOD_MS));
            }
#endif
            break;

            case RANGEFINDER_NONE:
            rangefinderHardware = RANGEFINDER_NONE;
            break;
    }

    if (rangefinderHardware == RANGEFINDER_NONE) {
        sensorsClear(SENSOR_RANGEFINDER);
        return false;
    }

    detectedSensors[SENSOR_INDEX_RANGEFINDER] = rangefinderHardware;
    sensorsSet(SENSOR_RANGEFINDER);
    return true;
}

bool rangefinderInit(rangefinder_t* rf, int8_t rangefinder_position)
{
    if (!rangefinderDetect(&rf->dev, rangefinderConfig()->rangefinder_hardware, rangefinder_position)) { // TODO: ADDED BY CAM
        return false;
    }

    // RANGE FINDER DETECT ISN'T ACTUALLY USED BY ANYTHING IMPORTANT, SO I CAN SPECIFY WHICH RANGEFINDER (F,R,Rr,L) TO DETECT AND INIT HERE
    // IT ALSO MAKES SENSE TO DEFINE CLI VARIABLES WHICH CAN CONTROL WHICH DIRECTIONS I HAVE LIDARS FOR COLLISION DETECTION

    rf->dev.init(&rf->dev);
    rf->rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    rf->calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    rf->maxTiltCos = cos_approx(DECIDEGREES_TO_RADIANS(rf->dev.detectionConeExtendedDeciDegrees / 2.0f));
    rf->lastValidResponseTimeMs = millis();

    return true;
}

static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 3
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > RANGEFINDER_OUT_OF_RANGE) {// only accept samples that are in range
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
timeDelta_t rangefinderUpdate(rangefinder_t* rf)
{
    if (rf->dev.update) {
        rf->dev.update(&rf->dev);
    }

    return MS2US(rf->dev.delayMs);
}

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, RANGEFINDER_OUT_OF_RANGE is returned.
 */
bool rangefinderProcess(rangefinder_t* rf, float cosTiltAngle)
{
    if (rf->dev.read) {
        const int32_t distance = rf->dev.read(&rf->dev);

        if (distance < 200.0 && distance >= 0.0) {
            posControl.flags.isCollisionDetected = true;
        } else {
            posControl.flags.isCollisionDetected = false;
        }

        // If driver reported no new measurement - don't do anything
        if (distance == RANGEFINDER_NO_NEW_DATA) {
            return false;
        }

        if (distance >= 0) {
            rf->lastValidResponseTimeMs = millis();
            rf->rawAltitude = distance;

            if (rangefinderConfig()->use_median_filtering) {
                rf->rawAltitude = applyMedianFilter(rf->rawAltitude);
            }
        }
        else if (distance == RANGEFINDER_OUT_OF_RANGE) {
            rf->lastValidResponseTimeMs = millis();
            rf->rawAltitude = RANGEFINDER_OUT_OF_RANGE;
        }
        else {
            // Invalid response / hardware failure
            rf->rawAltitude = RANGEFINDER_HARDWARE_FAILURE;
        }
    }
    else {
        // Bad configuration
        rf->rawAltitude = RANGEFINDER_OUT_OF_RANGE;
    }

    /**
    * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
    * the altitude. Returns the computed altitude in centimeters.
    *
    * When the ground is too far away or the tilt is too large, RANGEFINDER_OUT_OF_RANGE is returned.
    */
    if (cosTiltAngle < rf->maxTiltCos || rf->rawAltitude < 0) {
        rf->calculatedAltitude = RANGEFINDER_OUT_OF_RANGE;
    } else {
        rf->calculatedAltitude = rf->rawAltitude * cosTiltAngle;
    }

    return true;
}

/**
 * Get the latest altitude that was computed, or RANGEFINDER_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t rangefinderGetLatestAltitude(rangefinder_t* rf)
{
    return rf->calculatedAltitude;
}

int32_t rangefinderGetLatestRawAltitude(rangefinder_t* rf) {
    return rf->rawAltitude;
}

bool rangefinderIsHealthy(rangefinder_t* rf)
{
    return (millis() - rf->lastValidResponseTimeMs) < RANGEFINDER_HARDWARE_TIMEOUT_MS;
}
#endif
