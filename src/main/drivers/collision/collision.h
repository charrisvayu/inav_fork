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

#include "common/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"

#define COLLISION_OUT_OF_RANGE        (-1)
#define COLLISION_HARDWARE_FAILURE    (-2)
#define COLLISION_NO_NEW_DATA         (-3)

struct collisionDev_s;

typedef void (*collisionOpInitFuncPtr)(struct collisionDev_s * dev);
typedef void (*collisionOpStartFuncPtr)(struct collisionDev_s * dev);
typedef int32_t (*collisionOpReadFuncPtr)(struct collisionDev_s * dev);

typedef struct collisionDev_s {
    busDevice_t * busDev;           // Device on a bus (if applicable)

    timeMs_t delayMs;
    int16_t maxRangeCm;

    // function pointers
    collisionOpInitFuncPtr init;
    collisionOpStartFuncPtr update;
    collisionOpReadFuncPtr read;
} collisionDev_t;
