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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/interpolated_setpoint.h"
#include "flight/gps_rescue.h"
#include "flight/pid.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"

#include "rc.h"


typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);

#ifdef USE_INTERPOLATED_SP
// Setpoint in degrees/sec before RC-Smoothing is applied
static float rawSetpoint[XYZ_AXIS_COUNT];
// Stick deflection [-1.0, 1.0] before RC-Smoothing is applied
static float rawDeflection[XYZ_AXIS_COUNT];
static float oldRcCommand[XYZ_AXIS_COUNT];
#endif
static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
static float throttlePIDAttenuation;
static bool reverseMotors = false;
static applyRatesFn *applyRates;
uint16_t currentRxRefreshRate;

FAST_RAM_ZERO_INIT uint8_t interpolationChannels;
static FAST_RAM_ZERO_INIT uint32_t rcFrameNumber;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
    COLLECTIVE_FLAG = 1 << AUX1,      // HF3D:  Updated to allow AUX1 channel smoothing
};

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_IDENTITY_FREQUENCY         80    // Used in the formula to convert a BIQUAD cutoff frequency to PT1
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_RX_RATE_MIN_US             1000  // 1ms
#define RC_SMOOTHING_RX_RATE_MAX_US             50000 // 50ms or 20hz
#define RC_SMOOTHING_INTERPOLATED_FEEDFORWARD_DERIVATIVE_PT1_HZ 100 // The value to use for "auto" when interpolated feedforward is enabled

static FAST_RAM_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
#endif // USE_RC_SMOOTHING_FILTER

uint32_t getRcFrameNumber() 
{
    return rcFrameNumber;
}

float getSetpointRate(int axis)
{
    return setpointRate[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

float getThrottlePIDAttenuation(void)
{
    return throttlePIDAttenuation;
}

#ifdef USE_INTERPOLATED_SP
float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getRawDeflection(int axis)
{
    return rawDeflection[axis];
}

#endif

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT 1998
STATIC_ASSERT(CONTROL_RATE_CONFIG_RATE_LIMIT_MAX <= SETPOINT_RATE_LIMIT, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX_too_large);

#define RC_RATE_INCREMENTAL 14.54f

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + 0.01f * currentControlRateProfile->rcExpo[axis] * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = 10.0f * currentControlRateProfile->rcRates[axis] * rcCommandf;
    angleRate = angleRate * (1 + rcCommandfAbs * (float)currentControlRateProfile->rates[axis] * 0.01f);

    return angleRate;
}

float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = constrainf(((2000.0f * kissRpyUseRates) * kissRcCommandf), -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);

    return kissAngle;
}

float applyCurve(int axis, float deflection)
{
    return applyRates(axis, deflection, fabsf(deflection));
}

float getRcCurveSlope(int axis, float deflection)
{
    return (applyCurve(axis, deflection + 0.01f) - applyCurve(axis, deflection)) * 100.0f;
}

static void calculateSetpointRate(int axis)
{
    float angleRate;
    
#ifdef USE_GPS_RESCUE
    if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // If GPS Rescue is active then override the setpointRate used in the
        // pid controller with the value calculated from the desired heading logic.
        angleRate = gpsRescueGetYawRate();

        // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
        rcDeflection[axis] = 0;
        rcDeflectionAbs[axis] = 0;
    } else
#endif
    {
        // scale rcCommandf to range [-1.0, 1.0]
        float rcCommandf = rcCommand[axis] / 500.0f;
        rcDeflection[axis] = rcCommandf;
        const float rcCommandfAbs = fabsf(rcCommandf);
        rcDeflectionAbs[axis] = rcCommandfAbs;

        angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);
    }
    // Rate limit from profile (deg/sec)
    setpointRate[axis] = constrainf(angleRate, -1.0f * currentControlRateProfile->rate_limit[axis], 1.0f * currentControlRateProfile->rate_limit[axis]);

    DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);
}

/* static void scaleRcCommandToFpvCamAngle(void)
{
    //recalculate sin/cos only when rxConfig()->fpvCamAngleDegrees changed
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosFactor = 1.0;
    static float sinFactor = 0.0;

    if (lastFpvCamAngleDegrees != rxConfig()->fpvCamAngleDegrees) {
        lastFpvCamAngleDegrees = rxConfig()->fpvCamAngleDegrees;
        cosFactor = cos_approx(rxConfig()->fpvCamAngleDegrees * RAD);
        sinFactor = sin_approx(rxConfig()->fpvCamAngleDegrees * RAD);
    }

    float roll = setpointRate[ROLL];
    float yaw = setpointRate[YAW];
    setpointRate[ROLL] = constrainf(roll * cosFactor -  yaw * sinFactor, -SETPOINT_RATE_LIMIT * 1.0f, SETPOINT_RATE_LIMIT * 1.0f);
    setpointRate[YAW]  = constrainf(yaw  * cosFactor + roll * sinFactor, -SETPOINT_RATE_LIMIT * 1.0f, SETPOINT_RATE_LIMIT * 1.0f);
} */

/* #define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100

static void checkForThrottleErrorResetState(uint16_t rxRefreshRate)
{
    static int index;
    static int16_t rcCommandThrottlePrevious[THROTTLE_BUFFER_MAX];

    const int rxRefreshRateMs = rxRefreshRate / 1000;
    const int indexMax = constrain(THROTTLE_DELTA_MS / rxRefreshRateMs, 1, THROTTLE_BUFFER_MAX);
    const int16_t throttleVelocityThreshold = (featureIsEnabled(FEATURE_3D)) ? currentPidProfile->itermThrottleThreshold / 2 : currentPidProfile->itermThrottleThreshold;

    rcCommandThrottlePrevious[index++] = rcCommand[THROTTLE];
    if (index >= indexMax) {
        index = 0;
    }

    const int16_t rcCommandSpeed = rcCommand[THROTTLE] - rcCommandThrottlePrevious[index];

    if (currentPidProfile->antiGravityMode == ANTI_GRAVITY_STEP) {
        if (ABS(rcCommandSpeed) > throttleVelocityThreshold) {
            pidSetItermAccelerator(CONVERT_PARAMETER_TO_FLOAT(currentPidProfile->itermAcceleratorGain));
        } else {
            pidSetItermAccelerator(1.0f);
        }
    }
} */

static FAST_CODE uint8_t processRcInterpolation(void)
{
    static FAST_RAM_ZERO_INIT float rcCommandInterp[5];
    static FAST_RAM_ZERO_INIT float rcStepSize[5];
    static FAST_RAM_ZERO_INIT int16_t rcInterpolationStepCount;

    uint16_t rxRefreshRate;
    uint8_t updatedChannel = 0;

    if (rxConfig()->rcInterpolation) {
         // Set RC refresh rate for sampling and channels to filter
        switch (rxConfig()->rcInterpolation) {
        case RC_SMOOTHING_AUTO:
            rxRefreshRate = currentRxRefreshRate + 1000; // Add slight overhead to prevent ramps
            break;
        case RC_SMOOTHING_MANUAL:
            rxRefreshRate = 1000 * rxConfig()->rcInterpolationInterval;
            break;
        case RC_SMOOTHING_OFF:
        case RC_SMOOTHING_DEFAULT:
        default:
            rxRefreshRate = rxGetRefreshRate();
        }

        if (isRXDataNew && rxRefreshRate > 0) {
            rcInterpolationStepCount = rxRefreshRate / targetPidLooptime;

            for (int channel = 0; channel < PRIMARY_CHANNEL_COUNT; channel++) {
                if ((1 << channel) & interpolationChannels) {
                    rcStepSize[channel] = (rcCommand[channel] - rcCommandInterp[channel]) / (float)rcInterpolationStepCount;
                }
            }

           DEBUG_SET(DEBUG_RC_INTERPOLATION, 0, lrintf(rcCommand[0]));
           DEBUG_SET(DEBUG_RC_INTERPOLATION, 1, lrintf(currentRxRefreshRate / 1000));
        } else {
            rcInterpolationStepCount--;
        }

        // Interpolate steps of rcCommand
        if (rcInterpolationStepCount > 0) {
            for (updatedChannel = 0; updatedChannel < PRIMARY_CHANNEL_COUNT; updatedChannel++) {
                if ((1 << updatedChannel) & interpolationChannels) {
                    rcCommandInterp[updatedChannel] += rcStepSize[updatedChannel];
                    rcCommand[updatedChannel] = rcCommandInterp[updatedChannel];
                }
            }
        }
    } else {
        rcInterpolationStepCount = 0; // reset factor in case of level modes flip flopping
    }

    DEBUG_SET(DEBUG_RC_INTERPOLATION, 2, rcInterpolationStepCount);

    return updatedChannel;

}

#ifdef USE_RC_SMOOTHING_FILTER
// Determine a cutoff frequency based on filter type and the calculated
// average rx frame time
FAST_CODE_NOINLINE int calcRcSmoothingCutoff(int avgRxFrameTimeUs, bool pt1, uint8_t autoSmoothnessFactor)
{
    if (avgRxFrameTimeUs > 0) {
        const float cutoffFactor = (100 - autoSmoothnessFactor) / 100.0f;
        float cutoff = (1 / (avgRxFrameTimeUs * 1e-6f)) / 2;  // calculate the nyquist frequency
        cutoff = cutoff * cutoffFactor;

        if (pt1) {
            cutoff = sq(cutoff) / RC_SMOOTHING_IDENTITY_FREQUENCY; // convert to a cutoff for pt1 that has similar characteristics
        }
        return lrintf(cutoff);
    } else {
        return 0;
    }
}

// Preforms a reasonableness check on the rx frame time to avoid bad data
// skewing the average.
static FAST_CODE bool rcSmoothingRxRateValid(int currentRxRefreshRate)
{
    return (currentRxRefreshRate >= RC_SMOOTHING_RX_RATE_MIN_US && currentRxRefreshRate <= RC_SMOOTHING_RX_RATE_MAX_US);
}

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    const float dT = targetPidLooptime * 1e-6f;
    uint16_t oldCutoff = smoothingData->inputCutoffFrequency;
    
    if (smoothingData->inputCutoffSetting == 0) {
        smoothingData->inputCutoffFrequency = calcRcSmoothingCutoff(smoothingData->averageFrameTimeUs, (smoothingData->inputFilterType == RC_SMOOTHING_INPUT_PT1), smoothingData->autoSmoothnessFactor);
    }

    // initialize or update the input filter
    if ((smoothingData->inputCutoffFrequency != oldCutoff) || !smoothingData->filterInitialized) {
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if ((1 << i) & interpolationChannels) {  // only update channels specified by rc_interp_ch
                switch (smoothingData->inputFilterType) {
                
                    case RC_SMOOTHING_INPUT_PT1:
                        if (!smoothingData->filterInitialized) {
                            pt1FilterInit((pt1Filter_t*) &smoothingData->filter[i], pt1FilterGain(smoothingData->inputCutoffFrequency, dT));
                        } else {
                            pt1FilterUpdateCutoff((pt1Filter_t*) &smoothingData->filter[i], pt1FilterGain(smoothingData->inputCutoffFrequency, dT));
                        }
                        break;
                        
                    case RC_SMOOTHING_INPUT_BIQUAD:
                    default:
                        if (!smoothingData->filterInitialized) {
                            biquadFilterInitLPF((biquadFilter_t*) &smoothingData->filter[i], smoothingData->inputCutoffFrequency, targetPidLooptime);
                        } else {
                            biquadFilterUpdateLPF((biquadFilter_t*) &smoothingData->filter[i], smoothingData->inputCutoffFrequency, targetPidLooptime);
                        }
                        break;
                }
            }
        }
    }

    // update or initialize the derivative filter
    oldCutoff = smoothingData->derivativeCutoffFrequency;
    if ((rcSmoothingData.derivativeFilterType != RC_SMOOTHING_DERIVATIVE_OFF)
        && (currentPidProfile->ff_interpolate_sp == FF_INTERPOLATE_OFF)
        && (rcSmoothingData.derivativeCutoffSetting == 0)) {

        smoothingData->derivativeCutoffFrequency = calcRcSmoothingCutoff(smoothingData->averageFrameTimeUs, (smoothingData->derivativeFilterType == RC_SMOOTHING_DERIVATIVE_PT1), smoothingData->autoSmoothnessFactor);
    }

    if (!smoothingData->filterInitialized) {
        pidInitSetpointDerivativeLpf(smoothingData->derivativeCutoffFrequency, smoothingData->debugAxis, smoothingData->derivativeFilterType);
    } else if (smoothingData->derivativeCutoffFrequency != oldCutoff) {
        pidUpdateSetpointDerivativeLpf(smoothingData->derivativeCutoffFrequency);
    }
}

FAST_CODE_NOINLINE void rcSmoothingResetAccumulation(rcSmoothingFilter_t *smoothingData)
{
    smoothingData->training.sum = 0;
    smoothingData->training.count = 0;
    smoothingData->training.min = UINT16_MAX;
    smoothingData->training.max = 0;
}

// Accumulate the rx frame time samples. Once we've collected enough samples calculate the
// average and return true.
static FAST_CODE bool rcSmoothingAccumulateSample(rcSmoothingFilter_t *smoothingData, int rxFrameTimeUs)
{
    smoothingData->training.sum += rxFrameTimeUs;
    smoothingData->training.count++;
    smoothingData->training.max = MAX(smoothingData->training.max, rxFrameTimeUs);
    smoothingData->training.min = MIN(smoothingData->training.min, rxFrameTimeUs);

    // if we've collected enough samples then calculate the average and reset the accumulation
    const int sampleLimit = (rcSmoothingData.filterInitialized) ? RC_SMOOTHING_FILTER_RETRAINING_SAMPLES : RC_SMOOTHING_FILTER_TRAINING_SAMPLES;
    if (smoothingData->training.count >= sampleLimit) {
        smoothingData->training.sum = smoothingData->training.sum - smoothingData->training.min - smoothingData->training.max; // Throw out high and low samples
        smoothingData->averageFrameTimeUs = lrintf(smoothingData->training.sum / (smoothingData->training.count - 2));
        rcSmoothingResetAccumulation(smoothingData);
        return true;
    }
    return false;
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely 
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    // if the input cutoff is 0 (auto) then we need to calculate cutoffs
    if (rcSmoothingData.inputCutoffSetting == 0) {
        return true;
    }

    // if the derivative type isn't OFF, and the cutoff is 0, and interpolated feedforward is not enabled then we need to calculate
    if ((rcSmoothingData.derivativeFilterType != RC_SMOOTHING_DERIVATIVE_OFF)
        && (currentPidProfile->ff_interpolate_sp == FF_INTERPOLATE_OFF)
        && (rcSmoothingData.derivativeCutoffSetting == 0)) {
        return true;
    }
    return false;
}

static FAST_CODE uint8_t processRcSmoothingFilter(void)
{
    uint8_t updatedChannel = 0;
    static FAST_RAM_ZERO_INIT float lastRxData[5];
    static FAST_RAM_ZERO_INIT bool initialized;
    static FAST_RAM_ZERO_INIT timeMs_t validRxFrameTimeMs;
    static FAST_RAM_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.averageFrameTimeUs = 0;
        rcSmoothingData.autoSmoothnessFactor = rxConfig()->rc_smoothing_auto_factor;
        rcSmoothingData.debugAxis = rxConfig()->rc_smoothing_debug_axis;
        rcSmoothingData.inputFilterType = rxConfig()->rc_smoothing_input_type;
        rcSmoothingData.inputCutoffSetting = rxConfig()->rc_smoothing_input_cutoff;

        rcSmoothingData.derivativeFilterTypeSetting = rxConfig()->rc_smoothing_derivative_type;
        if (rxConfig()->rc_smoothing_derivative_type == RC_SMOOTHING_DERIVATIVE_AUTO) {
            // for derivative filter type "AUTO" set to BIQUAD for classic FF and PT1 for interpolated FF
            if (currentPidProfile->ff_interpolate_sp == FF_INTERPOLATE_OFF) {
                rcSmoothingData.derivativeFilterType = RC_SMOOTHING_DERIVATIVE_BIQUAD;
            } else {
                rcSmoothingData.derivativeFilterType = RC_SMOOTHING_DERIVATIVE_PT1;
            }
        } else {
            rcSmoothingData.derivativeFilterType = rxConfig()->rc_smoothing_derivative_type;
        }

        rcSmoothingData.derivativeCutoffSetting = rxConfig()->rc_smoothing_derivative_cutoff;
        rcSmoothingResetAccumulation(&rcSmoothingData);
        
        rcSmoothingData.inputCutoffFrequency = rcSmoothingData.inputCutoffSetting;
        
        if (rcSmoothingData.derivativeFilterType != RC_SMOOTHING_DERIVATIVE_OFF) {
            if ((currentPidProfile->ff_interpolate_sp != FF_INTERPOLATE_OFF) && (rcSmoothingData.derivativeCutoffSetting == 0)) {
                // calculate the fixed derivative cutoff used for interpolated feedforward
                const float cutoffFactor = (100 - rcSmoothingData.autoSmoothnessFactor) / 100.0f;
                float derivativeCutoff = RC_SMOOTHING_INTERPOLATED_FEEDFORWARD_DERIVATIVE_PT1_HZ * cutoffFactor;  // PT1 cutoff frequency
                if (rcSmoothingData.derivativeFilterType == RC_SMOOTHING_DERIVATIVE_BIQUAD) {
                    // convert to an equivalent BIQUAD cutoff
                    derivativeCutoff = sqrt(derivativeCutoff * RC_SMOOTHING_IDENTITY_FREQUENCY);
                }
                rcSmoothingData.derivativeCutoffFrequency = lrintf(derivativeCutoff);
            } else {
                rcSmoothingData.derivativeCutoffFrequency = rcSmoothingData.derivativeCutoffSetting;
            }
        }
        
        calculateCutoffs = rcSmoothingAutoCalculate();

        // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
        if (!calculateCutoffs) {
            rcSmoothingSetFilterCutoffs(&rcSmoothingData);
            rcSmoothingData.filterInitialized = true;
        }
    }

    if (isRXDataNew) {

        // store the new raw channel values
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if ((1 << i) & interpolationChannels) {
                lastRxData[i] = rcCommand[i];
            }
        }

        // for dynamically calculated filters we need to examine each rx frame interval
        if (calculateCutoffs) {
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;

            // If the filter cutoffs are set to auto and we have good rx data, then determine the average rx frame rate
            // and use that to calculate the filter cutoff frequencies
            if ((currentTimeMs > RC_SMOOTHING_FILTER_STARTUP_DELAY_MS) && (targetPidLooptime > 0)) { // skip during FC initialization
                if (rxIsReceivingSignal()  && rcSmoothingRxRateValid(currentRxRefreshRate)) {

                    // set the guard time expiration if it's not set
                    if (validRxFrameTimeMs == 0) {
                        validRxFrameTimeMs = currentTimeMs + (rcSmoothingData.filterInitialized ? RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS : RC_SMOOTHING_FILTER_TRAINING_DELAY_MS);
                    } else {
                        sampleState = 1;
                    }

                    // if the guard time has expired then process the rx frame time
                    if (currentTimeMs > validRxFrameTimeMs) {
                        sampleState = 2;
                        bool accumulateSample = true;

                        // During initial training process all samples.
                        // During retraining check samples to determine if they vary by more than the limit percentage.
                        if (rcSmoothingData.filterInitialized) {
                            const float percentChange = (ABS(currentRxRefreshRate - rcSmoothingData.averageFrameTimeUs) / (float)rcSmoothingData.averageFrameTimeUs) * 100;
                            if (percentChange < RC_SMOOTHING_RX_RATE_CHANGE_PERCENT) {
                                // We received a sample that wasn't more than the limit percent so reset the accumulation
                                // During retraining we need a contiguous block of samples that are all significantly different than the current average
                                rcSmoothingResetAccumulation(&rcSmoothingData);
                                accumulateSample = false;
                            }
                        }

                        // accumlate the sample into the average
                        if (accumulateSample) {
                            if (rcSmoothingAccumulateSample(&rcSmoothingData, currentRxRefreshRate)) {
                                // the required number of samples were collected so set the filter cutoffs
                                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                                rcSmoothingData.filterInitialized = true;
                                validRxFrameTimeMs = 0;
                            }
                        }

                    }
                } else {
                    // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                    rcSmoothingResetAccumulation(&rcSmoothingData);
                }
            }

            // rx frame rate training blackbox debugging
            if (debugMode == DEBUG_RC_SMOOTHING_RATE) {
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxRefreshRate);              // log each rx frame interval
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.training.count);    // log the training step count
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.averageFrameTimeUs);// the current calculated average
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState);                       // indicates whether guard time is active
            }
        }
    }

    if (rcSmoothingData.filterInitialized && (debugMode == DEBUG_RC_SMOOTHING)) {
        // after training has completed then log the raw rc channel and the calculated
        // average rx frame rate that was used to calculate the automatic filter cutoffs
        DEBUG_SET(DEBUG_RC_SMOOTHING, 0, lrintf(lastRxData[rcSmoothingData.debugAxis]));
        DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.averageFrameTimeUs);
    }

    // each pid loop continue to apply the last received channel value to the filter
    for (updatedChannel = 0; updatedChannel < PRIMARY_CHANNEL_COUNT; updatedChannel++) {
        if ((1 << updatedChannel) & interpolationChannels) {  // only smooth selected channels base on the rc_interp_ch value
            if (rcSmoothingData.filterInitialized) {
                switch (rcSmoothingData.inputFilterType) {
                    case RC_SMOOTHING_INPUT_PT1:
                        rcCommand[updatedChannel] = pt1FilterApply((pt1Filter_t*) &rcSmoothingData.filter[updatedChannel], lastRxData[updatedChannel]);
                        break;

                    case RC_SMOOTHING_INPUT_BIQUAD:
                    default:
                        rcCommand[updatedChannel] = biquadFilterApplyDF1((biquadFilter_t*) &rcSmoothingData.filter[updatedChannel], lastRxData[updatedChannel]);
                        break;
                }
            } else {
                // If filter isn't initialized yet then use the actual unsmoothed rx channel data
                rcCommand[updatedChannel] = lastRxData[updatedChannel];
            }
        }
    }

    return interpolationChannels;
}
#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE void processRcCommand(void)
{
    uint8_t updatedChannel;

    if (isRXDataNew) {
        rcFrameNumber++;
    }

    // if (isRXDataNew && pidAntiGravityEnabled()) {
        // checkForThrottleErrorResetState(currentRxRefreshRate);
    // }

#ifdef USE_INTERPOLATED_SP
    if (isRXDataNew) {
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            oldRcCommand[i] = rcCommand[i];
            const float rcCommandf = rcCommand[i] / 500.0f;
            const float rcCommandfAbs = fabsf(rcCommandf);
            rawSetpoint[i] = applyRates(i, rcCommandf, rcCommandfAbs);
            rawDeflection[i] = rcCommandf;
        }
    }
#endif

    switch (rxConfig()->rc_smoothing_type) {
#ifdef USE_RC_SMOOTHING_FILTER
    case RC_SMOOTHING_TYPE_FILTER:
        updatedChannel = processRcSmoothingFilter();
        break;
#endif // USE_RC_SMOOTHING_FILTER
    case RC_SMOOTHING_TYPE_INTERPOLATION:
    default:
        updatedChannel = processRcInterpolation();
        break;
    }

    if (isRXDataNew || updatedChannel) {
        const uint8_t maxUpdatedAxis = isRXDataNew ? FD_YAW : MIN(updatedChannel, FD_YAW); // throttle channel doesn't require rate calculation
#if defined(SIMULATOR_BUILD)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunsafe-loop-optimizations"
#endif
        for (int axis = FD_ROLL; axis <= maxUpdatedAxis; axis++) {
#if defined(SIMULATOR_BUILD)
#pragma GCC diagnostic pop
#endif
            calculateSetpointRate(axis);
        }

        DEBUG_SET(DEBUG_RC_INTERPOLATION, 3, setpointRate[0]);

/*         // Scaling of AngleRate to camera angle (Mixing Roll and Yaw)
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
            scaleRcCommandToFpvCamAngle();
        } */
    }

    if (isRXDataNew) {
        isRXDataNew = false;
    }
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    int32_t prop;
    if (rcData[THROTTLE] < currentControlRateProfile->tpa_breakpoint) {
        prop = 100;
        throttlePIDAttenuation = 1.0f;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop = 100 - (uint16_t)currentControlRateProfile->dynThrPID * (rcData[THROTTLE] - currentControlRateProfile->tpa_breakpoint) / (2000 - currentControlRateProfile->tpa_breakpoint);
        } else {
            prop = 100 - currentControlRateProfile->dynThrPID;
        }
        throttlePIDAttenuation = prop / 100.0f;
    }

    int32_t tmp;
    for (int axis = 0; axis < 3; axis++) {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.

        tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (tmp > rcControlsConfig()->deadband) {
                tmp -= rcControlsConfig()->deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        } else {
            if (tmp > rcControlsConfig()->yaw_deadband) {
                tmp -= rcControlsConfig()->yaw_deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        }
        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
    }

    if (featureIsEnabled(FEATURE_3D)) {
        tmp = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - PWM_RANGE_MIN);
    } else {
        tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);
    }

    if (getLowVoltageCutoff()->enabled) {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (featureIsEnabled(FEATURE_3D) && !failsafeIsActive()) {
        if (!flight3DConfig()->switched_mode3d) {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        } else {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                reverseMotors = true;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MIN - rxConfig()->midrc);
            } else {
                reverseMotors = false;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        }
    }
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        static t_fp_vector_def  rcCommandBuff;

        rcCommandBuff.X = rcCommand[ROLL];
        rcCommandBuff.Y = rcCommand[PITCH];
        if ((!FLIGHT_MODE(ANGLE_MODE) && (!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommandBuff.Z = rcCommand[YAW];
        } else {
            rcCommandBuff.Z = 0;
        }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&rcCommandBuff);
        rcCommand[ROLL] = rcCommandBuff.X;
        rcCommand[PITCH] = rcCommandBuff.Y;
        if ((!FLIGHT_MODE(ANGLE_MODE)&&(!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommand[YAW] = rcCommandBuff.Z;
        }
    }
    
    // HF3D:  Adding collective to rcCommands, constrain to -500/+500 range around midrc
    tmp = constrain(rcData[COLLECTIVE] - rxConfig()->midrc, -500, 500);
    rcCommand[COLLECTIVE] = tmp;
    
#if defined(USE_ACC)
    // HF3D:  Rescue (angle) mode and autoflip mode override the user's collective pitch rcCommand
    // Check if rescue (angle) mode or autoflip mode (airmode) is active
    if (FLIGHT_MODE(ANGLE_MODE)) {    // || pidGetAutoflipInProgress()  //  || FLIGHT_MODE(GPS_RESCUE_MODE)
        // attitude.values.roll/pitch = 0 when level, 1800 when fully inverted (decidegrees)
        // Pitch is +90 / -90 at straight down and straight up.  Invert the absolute value of that so 0 = straight up/down
        const float pitchCurrentInclination = 90.0f - (ABS(attitude.values.pitch) / 10.0f);
        // Roll is +90/-90 when sideways, and +180/-180 when inverted
        float rollCurrentInclination = 0.0f;
        if (ABS(attitude.values.roll) > 900.0f) {
            rollCurrentInclination = 90.0f - (180.0f - (ABS(attitude.values.roll) / 10.0f));
        } else {
            rollCurrentInclination = 90.0f - (ABS(attitude.values.roll) / 10.0f);
        }
        const float vertCurrentInclination = MIN(pitchCurrentInclination, rollCurrentInclination);

        // adjust collective pitch so heli has no collective while past vertical, and pitch is added as it approaches level
        // vertCurrentInclination between 0 (vertical) and 90 (level)  -- inverted from normal attitude values
        // Desired rescue collective at various vertical inclination angles:
        //   0deg(knife-edge)=0%, 30deg=11%, 45deg=25%, 60deg=45%, 80deg=79%, 90deg(level)=100%
        const float collectiveInclinationFactor = 0.000123456789f * vertCurrentInclination * vertCurrentInclination;
        
        // HF3D TODO:  Add rescue collective agression parameter to reduce tail blowout from large collective swings
        //   Or just rate limit the collective change?  Basically it's the big load and governor change that can overwhelm
        //   the tail authority of the heli.
        // HF3D TODO:  acc angleTrim setting isn't taken into account here... but is in pidLevel.
        //   Same issue up above.  It's minor as long as angleTrim is small, so maybe not worth the calculation overhead?
        if ( ((attitude.values.roll / 10.0f) > 90.0f) || ((attitude.values.roll / 10.0f) < -90.0f) ) {
            // We're closer to inverted.  Use negative collective pitch.
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // pidLevel will be rolling to inverted.
                rcCommand[COLLECTIVE] = pidGetRescueCollectiveSetting() * -collectiveInclinationFactor;
            } else if (pidGetAutoflipInProgress()) {
                // Scale the user's collective command during autoflips so the flip remains centered on the x/y axis
                //rcCommand[COLLECTIVE] *= -1.6 * collectiveInclinationFactor;
                rcCommand[COLLECTIVE] *= -1.45f * sin_approx(vertCurrentInclination * (M_PIf / 180.0f));
            }
        } else {
            // We're closer to upright.  Use positive collective pitch.
            if (FLIGHT_MODE(ANGLE_MODE)) {
                // pidLevel will be rolling to upright.
                rcCommand[COLLECTIVE] = pidGetRescueCollectiveSetting() * collectiveInclinationFactor;
            } else if (pidGetAutoflipInProgress()) {
                // Scale the user's collective command during autoflips so it remains centered on the x/y axis
                //rcCommand[COLLECTIVE] *= 1.6 * collectiveInclinationFactor;
                rcCommand[COLLECTIVE] *= 1.45f * sin_approx(vertCurrentInclination * (M_PIf / 180.0f));
            }  
        }
    } else if (pidGetAutoflipInProgress()) {
        
        const timeDelta_t elapsedFlipTime = micros() - pidGetAutoflipEngagedTime();
        
        if (elapsedFlipTime < 0) {
            // Create a collective pitch pump at the beginning of the flip
            rcCommand[COLLECTIVE] += rcCommand[COLLECTIVE] * 0.60f * constrainf((250000+elapsedFlipTime)/250000.0f,0.0f,1.0f);
        } else {
            // We're in the middle of the flip
            // Determine our orientation using 1 flip time = 360 degrees of rotation
            // Use that to drive the collective in open loop based on our assumed rotational position
            rcCommand[COLLECTIVE] *= 1.60f * cos_approx(2.0f * M_PIf * elapsedFlipTime / pidGetAutoflipFlipTime());
        }
        
        // Note:  Collective will immediately drop back down to the stick position when the autoflip is finished.
    }
#endif
    
}

void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}

bool isMotorsReversed(void)
{
    return reverseMotors;
}

void initRcProcessing(void)
{
    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;

        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;

        break;
    case RATES_TYPE_KISS:
        applyRates = applyKissRates;

        break;
    }

    interpolationChannels = 0;
    switch (rxConfig()->rcInterpolationChannels) {
    case INTERPOLATION_CHANNELS_RPYT:
        interpolationChannels |= THROTTLE_FLAG;
        interpolationChannels |= COLLECTIVE_FLAG;          // HF3D:  Updated to allow AUX1 channel smoothing

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RPY:
        interpolationChannels |= YAW_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_RP:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;

        break;
    case INTERPOLATION_CHANNELS_RPT:
        interpolationChannels |= ROLL_FLAG | PITCH_FLAG;

        FALLTHROUGH;
    case INTERPOLATION_CHANNELS_T:
        interpolationChannels |= THROTTLE_FLAG;
        interpolationChannels |= COLLECTIVE_FLAG;         // HF3D:  Updated to allow AUX1 channel smoothing

        break;
    }
}

bool rcSmoothingIsEnabled(void)
{
    return !(
#if defined(USE_RC_SMOOTHING_FILTER)
        rxConfig()->rc_smoothing_type == RC_SMOOTHING_TYPE_INTERPOLATION &&
#endif
        rxConfig()->rcInterpolation == RC_SMOOTHING_OFF);
}

#ifdef USE_RC_SMOOTHING_FILTER
rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothingData;
}

bool rcSmoothingInitializationComplete(void) {
    return (rxConfig()->rc_smoothing_type != RC_SMOOTHING_TYPE_FILTER) || rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER
