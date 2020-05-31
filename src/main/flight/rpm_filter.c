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

#include "config/feature.h"    // For checking if ESC_SENSOR is enabled.  The might be some other better method if init order is re-arranged or something.

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#include "sensors/esc_sensor.h"
#include "drivers/freq.h"

// HF3D:  Increasing MAXHARMONICS from 3 to 12 took ITCM_RAM from 15,472B @ 94.43% used to 17,680B @ 107.91% used.
// Reducing from 12 to 9 took it down to 16,944B with 103.42% used
// Reducing MAX_SUPPORTED_MOTORS from 8 to 4 with MAXHARMONICS = 9 took it to... almost no difference.  Which was surprising.
// Reducing MAX_SUPPORTED_MOTORS to 2 took it to 16,792B @ 102.49%
// Changing MAX_SUPPORTED_MOTORS to 4 and decreasing harmonics to 6 took it down to 16,200B @ 98.88% used
// Changing MAX_SUPPORTED_MOTORS to 2 and decreasing harmonics to 6 took it down to 16,048B @ 97.95% used
// Changing MAX_SUPPORTED_MOTORS to 4 and decreasing harmonics to 1 took it down to 14,952B @ 91.26% used
// MUST CHANGE gyro_rpm_notch_harmonics and dterm_rpm_notch_harmonics max limits in settings.c when this value is changed!
#define RPM_FILTER_MAXHARMONICS 6
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f

static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];

typedef struct rpmNotchFilter_s
{
    uint8_t harmonics;
    uint8_t harmonicsPerFreq;
    float   minHz;
    float   maxHz;
    float   q;
    float   loopTime;

    biquadFilter_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];
} rpmNotchFilter_t;

FAST_RAM_ZERO_INIT static float   erpmToHz;      // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
FAST_RAM_ZERO_INIT static float   erpmToHz1;     // HF3D TODO:  Change erpmToHz to array to allow for 2 different motors (main and tail)
FAST_RAM_ZERO_INIT static float   tailGearRatio; // HF3D
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

FAST_RAM_ZERO_INIT static uint8_t rpmSource;    // HF3D:  Dshot telemetry = 0, RPM sensor = 1, ESC_Sensor = 2


PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 3);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->gyro_rpm_notch_harmonics = 2;
    config->gyro_rpm_notch_min = 25;
    config->gyro_rpm_notch_q = 500;

    config->dterm_rpm_notch_harmonics = 0;
    config->dterm_rpm_notch_min = 100;
    config->dterm_rpm_notch_q = 500;

    config->rpm_lpf = 10;
    config->rpm_tail_gear_ratio = 0;
}

static void rpmNotchFilterInit(rpmNotchFilter_t* filter, int harmonics, int minHz, int q, float looptime)
{
    // If tail gear ratio != 0, harmonics will be created for the tail rpm as well
    // If main gear ratio > 1.1, harmonics will be created for the main motor rpm as well
    int totalHarmonicsCount = harmonics;
    if (mixerGetGovGearRatio() > 1.1f) {
        totalHarmonicsCount += harmonics;
    }
    if (tailGearRatio > 0) {
        totalHarmonicsCount += harmonics;
    }
    filter->harmonics = totalHarmonicsCount;
    filter->harmonicsPerFreq = harmonics;
    filter->minHz = minHz;
    filter->q = q / 100.0f;
    filter->loopTime = looptime;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int currentHarmonic = 0; currentHarmonic < totalHarmonicsCount; currentHarmonic++) {
                // Initialize each filter to minHz * harmonic
                float frequencyMultiplier = 0.0f;
                int workingHarmonic = currentHarmonic % harmonics;
                // Figure out if we're on a head, main, or tail harmonic
                if (currentHarmonic < harmonics) {
                    // First set of harmonics are always headspeed
                    frequencyMultiplier = (workingHarmonic + 1);           
                } else if (currentHarmonic / harmonics == 1) {
                    // Calculate main motor frequency using headspeed
                    // HF3D TODO:  Kind of inefficient right now how we're using mainGearRatio to calculate headspeed and then undoing it here?
                    frequencyMultiplier = (workingHarmonic + 1) * mixerGetGovGearRatio();
                } else if (currentHarmonic / harmonics == 2) {
                    // Calculate tail frequency using headspeed
                    frequencyMultiplier = (workingHarmonic + 1) * tailGearRatio;
                }
                // HF3D:  This used to be minHz*i, but that initializes the first filter to 0Hz... which probably isn't right?
                //   But it also probably doesn't matter since the filter coefficients will be updated on the next loop through.
                biquadFilterInit(
                    &filter->notch[axis][motor][currentHarmonic], minHz * frequencyMultiplier, looptime, filter->q, FILTER_NOTCH);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentFilter = &filters[0];
    currentMotor = currentHarmonic = currentFilterNumber = 0;
    tailGearRatio = config->rpm_tail_gear_ratio / 100.0f;       // HF3D

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

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    // HF3D TODO:  May need to fix this numberFilters count and filter init for a geared main motor + motor-driven tail combo
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
}


// Called by functions below, which are called by gyro.c and pid.c to apply RPM filters
static float applyFilter(rpmNotchFilter_t* filter, int axis, float value)
{
    // If we don't have a filter for this, then just return the original gyro value
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
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }
    }

    // Implement a simple load balancer that splits the filter updates up so that they update their frequency over the space of ~1ms.
    for (int i = 0; i < filterUpdatesPerIteration; i++) {
        // Calculate the frequency of the harmonic we're updating.  Harmonic 0 = fundamental = 1*frequency
        float frequency = 0.0f;
        int workingHarmonic = currentHarmonic % currentFilter->harmonicsPerFreq;
        // Figure out if we're on a head, main, or tail harmonic
        if (currentHarmonic < currentFilter->harmonicsPerFreq) {
            // First set of harmonics are always headspeed
            frequency = constrainf(
            (workingHarmonic + 1) * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);            
        } else if (currentHarmonic / currentFilter->harmonicsPerFreq == 1) {
            // Calculate main motor frequency using headspeed
            // HF3D TODO:  Kind of inefficient right now how we're using mainGearRatio to calculate headspeed and then undoing it here?
            frequency = constrainf(
            (workingHarmonic + 1) * motorFrequency[currentMotor] * mixerGetGovGearRatio(), currentFilter->minHz, currentFilter->maxHz);
        } else if (currentHarmonic / currentFilter->harmonicsPerFreq == 2) {
            // Calculate tail frequency using headspeed
            frequency = constrainf(
            (workingHarmonic + 1) * motorFrequency[currentMotor] * tailGearRatio, currentFilter->minHz, currentFilter->maxHz);            
        }
        // Update the roll axis filter coefficients for this motor & harmonic
        biquadFilter_t* template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */
        biquadFilterUpdate(
            template, frequency, currentFilter->loopTime, currentFilter->q, FILTER_NOTCH);
        // Transfer the filter coefficients from the updated roll axis filter into the filters on the other gyro axis for this motor + harmonic combo
        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilter_t* clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
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
