/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */


#include <math.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f


static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;

    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];
} rpmNotchFilter_t;

FAST_RAM_ZERO_INIT static float   erpmToHz;     // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
FAST_RAM_ZERO_INIT static float   filteredMotorErpm[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT static float   minMotorFrequency;
FAST_RAM_ZERO_INIT static uint8_t numberFilters;
FAST_RAM_ZERO_INIT static uint8_t numberRpmNotchFilters;
FAST_RAM_ZERO_INIT static uint8_t filterUpdatesPerIteration;
FAST_RAM_ZERO_INIT static float   pidLooptime;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t filters[2];
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* gyroFilter;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t* dtermFilter;

FAST_RAM_ZERO_INIT static uint8_t currentMotor;
FAST_RAM_ZERO_INIT static uint8_t currentHarmonic;
FAST_RAM_ZERO_INIT static uint8_t currentFilterNumber;
FAST_RAM static rpmNotchFilter_t* currentFilter = &filters[0];



PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 3;
    config->gyro_rpm_notch_min = 100;
    config->gyro_rpm_notch_q = 500;

    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 100;
    config->dterm_rpm_notch_q = 500;

    config->rpm_lpf = 150;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < harmonics; i++) {
                biquadFilterInit(
                    &filter->notch[axis][motor][i], minHz * i, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentFilter = &filters[0];
    currentMotor = currentHarmonic = currentFilterNumber = 0;

    numberRpmNotchFilters = 0;
    if (!motorConfig()->dev.useDshotTelemetry) {
        gyroFilter = dtermFilter = NULL;
        return;
    }

    pidLooptime = gyro.targetLooptime * pidConfig()->pid_process_denom;
    if (config->gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, config->gyro_rpm_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    } else {
        gyroFilter = NULL;
    }
    if (config->dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(dtermFilter, config->dterm_rpm_notch_harmonics,
                           config->dterm_rpm_notch_min, config->dterm_rpm_notch_q, pidLooptime);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f / (pidLooptime * 1e-6f);
    } else {
        dtermFilter = NULL;
    }

    // HF3D TODO:  Add RPM filters for head rpm and tail rpm
    //   Tail rotor rpm should be tail motor rpm if motor-driven tail, or calculated from tail gear ratio if gear/belt driven
    
    for (int i = 0; i < getMotorCount(); i++) {
        // init PT1 filter with dT= 0.0005  (Default for 2kHz = 500 pidLooptime)
        // Default rpm_lpf cutoff = 150Hz with dT=0.0005 ==> k = 0.32
        //   ===>  1/.32 ~= 3 samples delay for rpm filter to reach steady state
        //   ===>  3 samples * 0.5ms per sample ~= 1.5ms delay on RPM signal
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    // HF3D TODO:  Change erpmToHz and motorPoleCount to array (and update cli/configurator) to allow for 2 different motors (main and tail)
    //  For now this will only be used by the main motor (motor[0])
    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);
    //  Tail motor (motor[1]) erpmToHz for OMP M2 tail motor (12 bell magnets):
    erpmToHz1 = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (12.0f / 2.0f);

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
}

static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    
    for (int motor = 0; motor < getMotorCount(); motor++) {
     
        // Loop over and apply each set of gyro or dterm filters created for this axis and motor.  
        //   Default is 3 harmonic filters per motor for each axis
        //   Filter center frequency is updated separately in rpmFilterUpdate()
        for (int i = 0; i < filter->harmonics; i++) {
            value = biquadFilterApplyDF1(&filter->notch[axis][motor][i], value);
        }
    }
    return value;   // Return the resulting value after all filters are applied
}

// Called by gyroUpdate() in gyro.c 
//   Runs at Gyro looptime (equal to or faster than pidLooptime)
float rpmFilterGyro(int axis, float value)
{
    return applyFilter(gyroFilter, axis, value);
}

// Called by pidController() in pid.c
//   Runs at pidLooptime  (equal to or slower than Gyro looptime)
float rpmFilterDterm(int axis, float value)
{
    return applyFilter(dtermFilter, axis, value);
}

FAST_RAM_ZERO_INIT static float motorFrequency[MAX_SUPPORTED_MOTORS];

// rpmFilterUpdate() is called by pidController() in pid.c
//   Runs at pidLooptime  (equal to or slower than Gyro looptime)
FAST_CODE_NOINLINE void rpmFilterUpdate()
{
    if (gyroFilter == NULL && dtermFilter == NULL) {
        return;
    }

    for (int motor = 0; motor < getMotorCount(); motor++) {
        // Get the motor RPM using bi-directional DSHOT and then filter it using PT1 low-pass filter
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }
    }

    for (int i = 0; i < filterUpdatesPerIteration; i++) {
        float frequency = constrainf(
            (currentHarmonic + 1) * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);
        biquadFilter_t* template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t* clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
            clone->b0 = template->b0;
            clone->b1 = template->b1;
            clone->b2 = template->b2;
            clone->a1 = template->a1;
            clone->a2 = template->a2;
        }

        if (++currentHarmonic == currentFilter->harmonics) {
            currentHarmonic = 0;
            if (++currentFilterNumber == numberRpmNotchFilters) {
                currentFilterNumber = 0;
                if (++currentMotor == getMotorCount()) {
                    currentMotor = 0;
                }
                // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
                if currentMotor == 1 {
                    // Tail motor uses erpmToHz1
                    motorFrequency[currentMotor] = erpmToHz1 * filteredMotorErpm[currentMotor];
                } else {
                    motorFrequency[currentMotor] = erpmToHz * filteredMotorErpm[currentMotor];
                }
                minMotorFrequency = 0.0f;
            }
            currentFilter = &filters[currentFilterNumber];
        }

    }
}

bool isRpmFilterEnabled(void)
{
    return (motorConfig()->dev.useDshotTelemetry && (rpmFilterConfig()->gyro_rpm_notch_harmonics || rpmFilterConfig()->dterm_rpm_notch_harmonics));
}

float rpmMinMotorFrequency()
{
    if (minMotorFrequency == 0.0f) {
        minMotorFrequency = 10000.0f;
        for (int i = getMotorCount(); i--;) {
            if (motorFrequency[i] < minMotorFrequency) {
                minMotorFrequency = motorFrequency[i];
            }
        }
    }
    return minMotorFrequency;
}

// Return low pass filtered motor RPM for HF3D governor or tail motor control
float rpmGetFilteredMotorRPM(int motor)
{
    // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
    if motor == 1 {
        return (filteredMotorErpm[motor] * erpmToHz1 * SECONDS_PER_MINUTE);    // return filtered tail motor RPM
    } else {
        return (filteredMotorErpm[motor] * erpmToHz * SECONDS_PER_MINUTE);     // return filtered main/other motor RPM
    }
}

#endif
