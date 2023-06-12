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

#pragma once

#include <stdint.h>
#include "config/parameter_group.h"
#include "drivers/rangefinder/rangefinder.h"

typedef enum {
    RANGEFINDER_NONE        = 0,
    RANGEFINDER_SRF10       = 1,
    RANGEFINDER_VL53L0X     = 2,
    RANGEFINDER_MSP         = 3,
    RANGEFINDER_BENEWAKE    = 4,
    RANGEFINDER_VL53L1X     = 5,
    RANGEFINDER_US42        = 6,
    RANGEFINDER_TOF10102I2C = 7,
    RANGEFINDER_FAKE        = 8,
    RANGEFINDER_V3HPI2C     = 9,
    RANGEFINDER_LW20CI2C    = 10,
} rangefinderType_e;

typedef struct rangefinderConfig_s {
    uint8_t rangefinder_hardware;
    uint8_t use_median_filtering;
} rangefinderConfig_t;

PG_DECLARE(rangefinderConfig_t, rangefinderConfig);

typedef struct rangefinder_s {
    rangefinderDev_t dev;
    float maxTiltCos;
    int32_t rawAltitude;
    int32_t calculatedAltitude;
    timeMs_t lastValidResponseTimeMs;
    bool useForCollisionDetection;
} rangefinder_t;

extern rangefinder_t rangefinder;

bool rangefinderInit(rangefinder_t* rf, bool useForCollisionDetection);

int32_t rangefinderGetLatestAltitude(rangefinder_t* rf);
int32_t rangefinderGetLatestRawAltitude(rangefinder_t* rf);

timeDelta_t rangefinderUpdate(rangefinder_t* rf);
bool rangefinderProcess(float cosTiltAngle, rangefinder_t* rf);
bool rangefinderIsHealthy(rangefinder_t* rf);
