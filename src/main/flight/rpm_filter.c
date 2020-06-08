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

#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/freq.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"
#include "sensors/esc_sensor.h"

#include "rpm_filter.h"

#define RPM_FILTER_MAXHARMONICS 10
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;

    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];

} rpmNotchFilter_t;

FAST_RAM_ZERO_INIT static float    erpmToHz;      // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
FAST_RAM_ZERO_INIT static float    erpmToHz1;     // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
FAST_RAM_ZERO_INIT static float    tailGearRatio; // HF3D
FAST_RAM_ZERO_INIT static float    minMotorFrequency;
FAST_RAM_ZERO_INIT static uint32_t pidLooptime;

FAST_RAM_ZERO_INIT static float motorFrequency[MAX_SUPPORTED_MOTORS];
FAST_RAM_ZERO_INIT static float filteredMotorErpm[MAX_SUPPORTED_MOTORS];

FAST_RAM_ZERO_INIT static rpmNotchFilter_t filters[2];

FAST_RAM_ZERO_INIT static rpmNotchFilter_t * gyroFilter;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t * dtermFilter;
FAST_RAM_ZERO_INIT static rpmNotchFilter_t * currentFilter;

FAST_RAM_ZERO_INIT static uint8_t currentMotor;
FAST_RAM_ZERO_INIT static uint8_t currentHarmonic;
FAST_RAM_ZERO_INIT static uint8_t currentFilterNumber;
FAST_RAM_ZERO_INIT static uint8_t numberRpmNotchFilters;

FAST_RAM_ZERO_INIT static uint8_t rpmSource;    // HF3D:  Dshot telemetry = 0, RPM sensor = 1, ESC_Sensor = 2

FAST_RAM_ZERO_INIT static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];


PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 6;
    config->gyro_rpm_notch_min = 20;
    config->gyro_rpm_notch_q = 100;

    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 40;
    config->dterm_rpm_notch_q = 100;

    config->rpm_lpf = 10;
    config->rpm_tail_gear_ratio = 0;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    filter->harmonics = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int harmonic = 0; harmonic < harmonics; harmonic++) {
                // Init all filters @100Hz. As soon as the motor is running, the notches are updated to the real RPM.
                biquadFilterInit(&filter->notch[axis][motor][harmonic], 100.0, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentMotor = currentHarmonic = currentFilterNumber = 0;
    currentFilter = &filters[0];

    if (config->rpm_tail_gear_ratio > 1000) {
        tailGearRatio = config->rpm_tail_gear_ratio / 1000.0f;
    } else {
        tailGearRatio = 4.5;  // HF3D: This won't harm
    }

    numberRpmNotchFilters = 0;
    
    // HF3D TODO:  Make all the RPM filter code work even if we weren't built with USE_DSHOT.
    if (motorConfig()->dev.useDshotTelemetry) {
        rpmSource = 0;
    // Can't actually check if it initialized properly because Freq sensor is initialized after rpm_filter
    } else if (featureIsEnabled(FEATURE_FREQ_SENSOR)) {
        rpmSource = 1;
    // Can't check to see if ESC Sensor is initialized propertly because escSensorInit() isn't called until after rpmFilterInit
    //   } else if (isEscSensorActive()) {
    } else if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        rpmSource = 2;
    } else {
        // No RPM source available
        rpmSource = 255;
        gyroFilter = dtermFilter = NULL;
        return;
    }
    
    if (config->gyro_rpm_notch_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config->gyro_rpm_notch_harmonics,
                           config->gyro_rpm_notch_min, config->gyro_rpm_notch_q, gyro.targetLooptime);
        // don't go quite to nyquist to avoid oscillations
        gyroFilter->maxHz = 0.48f / (gyro.targetLooptime * 1e-6f);
    } else {
        gyroFilter = NULL;
    }

    pidLooptime = gyro.targetLooptime * pidConfig()->pid_process_denom;

    if (config->dterm_rpm_notch_harmonics) {
        dtermFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(dtermFilter, config->dterm_rpm_notch_harmonics,
                           config->dterm_rpm_notch_min, config->dterm_rpm_notch_q, pidLooptime);
        // don't go quite to nyquist to avoid oscillations
        dtermFilter->maxHz = 0.48f / (pidLooptime * 1e-6f);
    } else {
        dtermFilter = NULL;
    }

    // Tail rotor rpm should be tail motor rpm if motor-driven tail, or calculated from tail gear ratio if gear/belt driven
    for (int i = 0; i < getMotorCount(); i++) {
        // init PT1 filter with dT= 0.0005  (Default for 2kHz = 500 pidLooptime)
        // Default rpm_lpf cutoff = 150Hz with dT=0.0005 ==> k = 0.32  (calculated by pt1FilterGain)
        //   ===>  1/.32 ~= 3 samples delay for rpm to reach reasonable approximation of change
        //   ===>  3 samples * 0.5ms per sample ~= 1.5ms delay on RPM signal (10% step change in signal will converge to 1% offset in 5 samples, or ~2.5ms)
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_lpf, pidLooptime * 1e-6f));
    }

    // HF3D TODO:  Change erpmToHz and motorPoleCount to array (and update cli/configurator) to allow for 2 different motors (main and tail)
    //  For now this will only be used by the main motor (motor[0])
    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);
    //  Tail motor (motor[1]) erpmToHz for OMP M2 tail motor (12 bell magnets):
    erpmToHz1 = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (12.0f / 2.0f);
}


// Called by functions below, which are called by gyro.c and pid.c to apply RPM filters
static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    
    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < filter->harmonics; i++) {
            value = biquadFilterApplyDF1(&filter->notch[axis][motor][i], value);
        }
    }
    return value;
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


// rpmFilterUpdate() is called by pidController() in pid.c
//   Runs at pidLooptime  (equal to or slower than Gyro looptime)
//   Updates filter coefficients for the new motor rpm
FAST_CODE_NOINLINE void rpmFilterUpdate()
{
    // Don't calculate any filter updates if we're not doing any filtering.
    if (gyroFilter == NULL && dtermFilter == NULL) {
        return;
    }

    // Loop over all the motors and low-pass filter their RPM values to stabilize it
    for (int motor = 0; motor < getMotorCount(); motor++) {
        // Get motor eRPM/100 from our RPM signal source and then filter it using PT1 low-pass filter
        uint16_t motorRpm = 0;
        if (rpmSource == 0) {
            motorRpm = getDshotTelemetry(motor);
#ifdef USE_FREQ_SENSOR
        } else if (rpmSource == 1) {
            motorRpm = getFreqSensorRPM(motor);
#endif
        } else if (rpmSource == 2) {
            motorRpm = getEscSensorRPM(motor);
        } else {
            motorRpm = 0;
        }
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], motorRpm);
    }

    // Heli motor RPM is changing slowly. No need to hurry up with updates.
    {
        // Calculate the frequency of the harmonic we're updating.
        float mult = 1;

        // Approx. order of the harmonic importance
        switch (currentHarmonic) {
        case 0:
            mult = 1;
            break;
        case 1:
            mult = 2;
            break;
        case 2:
            mult = 3;
            break;
        case 3:
            mult = 4;
            break;
        case 4:
            mult = tailGearRatio;
            break;
        case 5:
            mult = 5;
            break;
        case 6:
            mult = 6;
            break;
        case 7:
            mult = 2 * tailGearRatio;
            break;
        case 8:
            mult = 7;
            break;
        case 9:
            mult = 8;
            break;
        }
        
        float frequency = constrainf(mult * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);

        DEBUG_SET(DEBUG_RPM_FILTER, 0, currentMotor);
        DEBUG_SET(DEBUG_RPM_FILTER, 1, currentHarmonic);
        DEBUG_SET(DEBUG_RPM_FILTER, 2, motorFrequency[currentMotor]);
        DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency);
        
        // Update the roll axis filter coefficients
        biquadFilter_t * template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        biquadFilterUpdate(template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);

        // Transfer the filter coefficients from the updated roll axis filter into pitch and yaw axis
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t * clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
            clone->b0 = template->b0;
            clone->b1 = template->b1;
            clone->b2 = template->b2;
            clone->a1 = template->a1;
            clone->a2 = template->a2;
        }

        // Check to see if we've updated all the harmonics, if so, go back to the first harmonic.
        if (++currentHarmonic == currentFilter->harmonics) {
            currentHarmonic = 0;
            // If we've updated all the filters, go back to the first filter next time.
            if (++currentFilterNumber == numberRpmNotchFilters) {
                currentFilterNumber = 0;
                // See if we've updated the filters and harmonics for each motor.  If so, go back to the first motor.
                if (++currentMotor == getMotorCount()) {
                    currentMotor = 0;
                }
                // Update the speed of this motor.
                // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
                if (currentMotor == 1) {
                    // Tail motor uses erpmToHz1
                    motorFrequency[currentMotor] = erpmToHz1 * filteredMotorErpm[currentMotor];
                } else {
                    // HF3D:  The main blades/head will be causing the main vibrations, so use gearRatio to get headspeed
                    // Note that the main vibrations to filter out will be at headspeed * #_of_blades (essentially 2nd or 3rd harmonic)
                    motorFrequency[currentMotor] = erpmToHz * filteredMotorErpm[currentMotor] / mixerGetGovGearRatio();
                }
                minMotorFrequency = 0.0f;
            }
            // Set the currentFilter to be the filter we just incremented to (or reset to)
            currentFilter = &filters[currentFilterNumber];
        }
    }
}


bool isRpmFilterEnabled(void)
{
    return (rpmSource < 255 && (rpmFilterConfig()->gyro_rpm_notch_harmonics || rpmFilterConfig()->dterm_rpm_notch_harmonics));
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
    if (motor == 1) {
        return (filteredMotorErpm[motor] * erpmToHz1 * SECONDS_PER_MINUTE);    // return filtered tail motor RPM
    } else {
        return (filteredMotorErpm[motor] * erpmToHz * SECONDS_PER_MINUTE);     // return filtered main/other motor RPM
    }
}

#endif
