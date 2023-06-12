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
#include "drivers/collision/collision.h"

typedef enum {
    COLLISION_NONE        = 0,
    COLLISION_LW20CI2C    = 1,
} collisionType_e;

typedef struct collisionConfig_s {
    uint8_t collision_hardware;
    uint8_t use_median_filtering_for_collision;
} collisionConfig_t;

PG_DECLARE(collisionConfig_t, collisionConfig);

typedef struct collision_s {
    collisionDev_t dev;
    int32_t rawDistance;
    int32_t calculatedDistance;
    timeMs_t lastValidResponseTimeMs;
} collision_t;

extern collision_t collision_1;

bool collisionInit(collision_t* coll);

int32_t collisionGetLatestDistance(collision_t* coll);
int32_t collisionGetLatestRawDistance(collision_t* coll);

timeDelta_t collisionUpdate(collision_t* coll);
bool collisionProcess(collision_t* coll);
bool collisionIsHealthy(collision_t* coll);
