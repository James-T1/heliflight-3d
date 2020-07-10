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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_reset.h"

#include "drivers/dshot_command.h"
#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/interpolated_setpoint.h"
#include "flight/servos.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

// HF3D:  Inits for PID Delay Compensation
// Delay length = 8000Hz * 35ms = 280 loop times
/*#define DELAYLENGTH  280            
FAST_RAM_ZERO_INIT int delayArrayPos = 0;
int16_t delayArrayRoll[DELAYLENGTH] = {0};
int16_t delayArrayPitch[DELAYLENGTH] = {0};
FAST_RAM_ZERO_INIT float delayCompSum[3] = {0.0f};
FAST_RAM_ZERO_INIT float delayCompAlpha = 0.0f; */
// End inits for PID Delay Compensation

static bool autoflipInProgress = false;
static timeUs_t autoflipEngagedTime = 0;
static timeDelta_t autoflipFlipTime;
static float autoflipCollectiveMultiplier = 0.0f;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

//static FAST_RAM_ZERO_INIT uint8_t antiGravityMode;
//static FAST_RAM_ZERO_INIT float antiGravityThrottleHpf;
//static FAST_RAM_ZERO_INIT uint16_t itermAcceleratorGain;
//static FAST_RAM float antiGravityOsdCutoff = 1.0f;
//static FAST_RAM_ZERO_INIT bool antiGravityEnabled;
static FAST_RAM_ZERO_INIT bool zeroThrottleItermReset;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#ifdef STM32F10X
#define PID_PROCESS_DENOM_DEFAULT       1
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT       4
#else
#define PID_PROCESS_DENOM_DEFAULT       2
#endif
#if defined(USE_D_MIN)
#define D_MIN_GAIN_FACTOR 0.00005f
#define D_MIN_SETPOINT_GAIN_FACTOR 0.00005f
#define D_MIN_RANGE_HZ 80    // Biquad lowpass input cutoff to peak D around propwash frequencies
#define D_MIN_LOWPASS_HZ 10  // PT1 lowpass cutoff to smooth the boost effect
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#ifdef USE_AIRMODE_LPF
static FAST_RAM_ZERO_INIT float airmodeThrottleOffsetLimit;
#endif

#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 13);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90 },
            [PID_PITCH] = { 46, 90, 38, 95 },
            [PID_YAW] =   { 30, 90, 0, 90 },
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 100,
        .vbatPidCompensation = 0,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .feedForwardTransition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 250,
        .itermAcceleratorGain = 3500,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .dterm_lowpass_hz = 150,    // NOTE: dynamic lpf is enabled by default so this setting is actually
                                    // overridden and the static lowpass 1 is disabled. We can't set this
                                    // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                    // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lowpass2_hz = 150,   // second Dterm LPF ON by default
        .dterm_filter_type = FILTER_PT1,
        .dterm_filter2_type = FILTER_PT1,
        .dyn_lpf_dterm_min_hz = 70,
        .dyn_lpf_dterm_max_hz = 170,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_min = { 20, 22, 0 },      // roll, pitch, yaw
        .d_min_gain = 27,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .idle_min_rpm = 0,
        .idle_adjustment_speed = 50,
        .idle_p = 50,
        .idle_pid_limit = 200,
        .idle_max_increase = 150,
        .ff_interpolate_sp = FF_INTERPOLATE_ON,
        .ff_spike_limit = 60,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        // HF3D parameters
        .yawColKf = 300,
        .yawColPulseKf = 300,
        .yawCycKf = 0,
        .yawBaseThrust = 900,
        .rescue_collective = 200,
        .error_decay_always = 0,
        .error_decay_rate = 7,
        .collective_ff_impulse_freq = 100,
        .elevator_filter_gain = 50,
        .elevator_filter_window_time = 75,
        .elevator_filter_window_size = 30,
        .elevator_filter_hz = 15,
        .autoflip_yaw_rate = 0,
        .autoflip_flip_rate = 0,
        .autoflip_collective_multiplier = 100,
    );
#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 30;
    pidProfile->pid[PID_PITCH].D = 32;
#endif
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

// HF3D TODO:  Remove iTermAccelerator from all the code
//static FAST_RAM float itermAccelerator = 1.0f;

// void pidSetItermAccelerator(float newItermAccelerator)
// {
    // itermAccelerator = newItermAccelerator;
// }

// bool pidOsdAntiGravityActive(void)
// {
    // return (itermAccelerator > antiGravityOsdCutoff);
// }

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermNotchApplyFn;
static FAST_RAM_ZERO_INIT biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpassApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr dtermLowpass2ApplyFn;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT filterApplyFnPtr ptermYawLowpassApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t ptermYawLowpass;

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static uint8_t itermRelaxCutoff;
static FAST_RAM_ZERO_INIT float itermRelaxSetpointThreshold;
#endif

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_RAM_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acGain;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acErrorLimit;
static FAST_RAM_ZERO_INIT float acCutoff;
static FAST_RAM_ZERO_INIT pt1Filter_t acLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float oldSetpointCorrection[XYZ_AXIS_COUNT];
#endif

#if defined(USE_D_MIN)
static FAST_RAM_ZERO_INIT biquadFilter_t dMinRange[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT pt1Filter_t dMinLowpass[XYZ_AXIS_COUNT];
#endif

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_AIRMODE_LPF
static FAST_RAM_ZERO_INIT pt1Filter_t airmodeThrottleLpf1;
static FAST_RAM_ZERO_INIT pt1Filter_t airmodeThrottleLpf2;
#endif

//static FAST_RAM_ZERO_INIT pt1Filter_t antiGravityThrottleLpf;

static FAST_RAM_ZERO_INIT float ffBoostFactor;
static FAST_RAM_ZERO_INIT float ffSmoothFactor;
static FAST_RAM_ZERO_INIT float ffSpikeLimitInverse;

static FAST_RAM_ZERO_INIT float collectivePulseFilterGain;

static FAST_RAM_ZERO_INIT filterApplyFnPtr elevatorFilterLowpassApplyFn;
static FAST_RAM_ZERO_INIT pt1Filter_t elevatorFilterLowpass;

float pidGetSpikeLimitInverse()
{
    return ffSpikeLimitInverse;
}


float pidGetFfBoostFactor()
{
    return ffBoostFactor;
}

float pidGetFfSmoothFactor()
{
    return ffSmoothFactor;
}


void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        dtermNotchApplyFn = nullFilterApply;
        dtermLowpassApplyFn = nullFilterApply;
        ptermYawLowpassApplyFn = nullFilterApply;
        elevatorFilterLowpassApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lowpass_hz = pidProfile->dterm_lowpass_hz;

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz) {
        dterm_lowpass_hz = pidProfile->dyn_lpf_dterm_min_hz;
    }
#endif

    if (dterm_lowpass_hz > 0 && dterm_lowpass_hz < pidFrequencyNyquist) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(dterm_lowpass_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
#ifdef USE_DYN_LPF
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApplyDF1;
#else
            dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
#endif
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, dterm_lowpass_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpassApplyFn = nullFilterApply;
            break;
        }
    } else {
        dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
    	dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        switch (pidProfile->dterm_filter2_type) {
        case FILTER_PT1:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                pt1FilterInit(&dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass2_hz, dT));
            }
            break;
        case FILTER_BIQUAD:
            dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
            for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
                biquadFilterInitLPF(&dtermLowpass2[axis].biquadFilter, pidProfile->dterm_lowpass2_hz, targetPidLooptime);
            }
            break;
        default:
            dtermLowpass2ApplyFn = nullFilterApply;
            break;
        }
    }

    if (pidProfile->yaw_lowpass_hz == 0 || pidProfile->yaw_lowpass_hz > pidFrequencyNyquist) {
        ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, dT));
    }

// #if defined(USE_THROTTLE_BOOST)
    // pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, dT));
// #endif
#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif
#if defined(USE_ABSOLUTE_CONTROL)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&acLpf[i], pt1FilterGain(acCutoff, dT));
        }
    }
#endif
#if defined(USE_D_MIN)

    // Initialize the filters for all axis even if the d_min[axis] value is 0
    // Otherwise if the pidProfile->d_min_xxx parameters are ever added to
    // in-flight adjustments and transition from 0 to > 0 in flight the feature
    // won't work because the filter wasn't initialized.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        biquadFilterInitLPF(&dMinRange[axis], D_MIN_RANGE_HZ, targetPidLooptime);
        pt1FilterInit(&dMinLowpass[axis], pt1FilterGain(D_MIN_LOWPASS_HZ, dT));
     }
#endif
#if defined(USE_AIRMODE_LPF)
    if (pidProfile->transient_throttle_limit) {
        pt1FilterInit(&airmodeThrottleLpf1, pt1FilterGain(7.0f, dT));
        pt1FilterInit(&airmodeThrottleLpf2, pt1FilterGain(20.0f, dT));
    }
#endif

    //pt1FilterInit(&antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, dT));

    ffBoostFactor = (float)pidProfile->ff_boost / 10.0f;
    ffSpikeLimitInverse = pidProfile->ff_spike_limit ? 1.0f / ((float)pidProfile->ff_spike_limit / 10.0f) : 0.0f;
    
    // HF3D
    // Collective input impulse high-pass filter.  Setting is for cutoff frequency in Hz * 100.
    // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
    //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
    //   k = dT / (RC + dT);                  ==>  k = 0.0000393 for 8kHz
    collectivePulseFilterGain = dT / (dT + (1 / ( 2 * 3.14159f * (float)pidProfile->collective_ff_impulse_freq / 100.0f)));
    
    // HF3D:  Elevator Filter (helicopter tail de-bounce)
    if (pidProfile->elevator_filter_hz == 0 || pidProfile->elevator_filter_hz > pidFrequencyNyquist) {
        elevatorFilterLowpassApplyFn = nullFilterApply;
    } else {
        elevatorFilterLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&elevatorFilterLowpass, pt1FilterGain(pidProfile->elevator_filter_hz, dT));
    }
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType)
{
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        setpointDerivativeLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterInit(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterInitLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff)
{
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
                case RC_SMOOTHING_DERIVATIVE_PT1:
                    pt1FilterUpdateCutoff(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                    break;
                case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                    biquadFilterUpdateLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                    break;
            }
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float feedForwardTransition;
static FAST_RAM_ZERO_INIT float levelGain, horizonGain, horizonTransition, horizonCutoffDegrees, horizonFactorRatio;
static FAST_RAM_ZERO_INIT float itermWindupPointInv;
static FAST_RAM_ZERO_INIT uint8_t horizonTiltExpertMode;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeLimitUs;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeDelayUs;
static FAST_RAM_ZERO_INIT int32_t crashRecoveryAngleDeciDegrees;
static FAST_RAM_ZERO_INIT float crashRecoveryRate;
static FAST_RAM_ZERO_INIT float crashDtermThreshold;
static FAST_RAM_ZERO_INIT float crashGyroThreshold;
static FAST_RAM_ZERO_INIT float crashSetpointThreshold;
static FAST_RAM_ZERO_INIT float crashLimitYaw;
static FAST_RAM_ZERO_INIT float itermLimit;
#if defined(USE_THROTTLE_BOOST)
FAST_RAM_ZERO_INIT float throttleBoost;
//pt1Filter_t throttleLpf;
#endif
static FAST_RAM_ZERO_INIT bool itermRotation;

#ifdef USE_INTEGRATED_YAW_CONTROL
static FAST_RAM_ZERO_INIT bool useIntegratedYaw;
static FAST_RAM_ZERO_INIT uint8_t integratedYawRelax;
#endif

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

#ifdef USE_ACRO_TRAINER
static FAST_RAM_ZERO_INIT float acroTrainerAngleLimit;
static FAST_RAM_ZERO_INIT float acroTrainerLookaheadTime;
static FAST_RAM_ZERO_INIT uint8_t acroTrainerDebugAxis;
static FAST_RAM_ZERO_INIT bool acroTrainerActive;
static FAST_RAM_ZERO_INIT int acroTrainerAxisState[2];  // only need roll and pitch
static FAST_RAM_ZERO_INIT float acroTrainerGain;
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
FAST_RAM_ZERO_INIT float thrustLinearization;
FAST_RAM_ZERO_INIT float thrustLinearizationReciprocal;
FAST_RAM_ZERO_INIT float thrustLinearizationB;
#endif

// void pidUpdateAntiGravityThrottleFilter(float throttle)
// {
    // if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        // antiGravityThrottleHpf = throttle - pt1FilterApply(&antiGravityThrottleLpf, throttle);
    // }
// }

#ifdef USE_DYN_LPF
static FAST_RAM uint8_t dynLpfFilter = DYN_LPF_NONE;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMin;
static FAST_RAM_ZERO_INIT uint16_t dynLpfMax;
#endif

#ifdef USE_D_MIN
static FAST_RAM_ZERO_INIT float dMinPercent[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float dMinGyroGain;
static FAST_RAM_ZERO_INIT float dMinSetpointGain;
#endif

#ifdef USE_INTERPOLATED_SP
static FAST_RAM_ZERO_INIT ffInterpolationType_t ffFromInterpolatedSetpoint;
#endif

// HF3D
static FAST_RAM_ZERO_INIT float rescueCollective;

void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedForwardTransition == 0) {
        feedForwardTransition = 0;
    } else {
        feedForwardTransition = 100.0f / pidProfile->feedForwardTransition;
    }
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // Scale down Roll & Pitch axis PID terms for helicopters.  Leave Yaw axis alone.
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P / ((axis == FD_YAW) ? 1.0f : 10.0f);
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I / ((axis == FD_YAW) ? 1.0f : 5.0f);
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D / ((axis == FD_YAW) ? 1.0f : 10.0f);
        pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
    }
    
    // HF3D:  Yaw integral gain does NOT need boosted on a helicopter.
// #ifdef USE_INTEGRATED_YAW_CONTROL
    // if (!pidProfile->use_integrated_yaw)
// #endif
    // {
        // pidCoefficient[FD_YAW].Ki *= 2.5f;
    // }

    levelGain = pidProfile->pid[PID_LEVEL].P / 10.0f;
    horizonGain = pidProfile->pid[PID_LEVEL].I / 10.0f;
    horizonTransition = (float)pidProfile->pid[PID_LEVEL].D;
    horizonTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
    horizonCutoffDegrees = (175 - pidProfile->horizon_tilt_effect) * 1.8f;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    itermWindupPointInv = 1.0f;
    if (pidProfile->itermWindupPointPercent < 100) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        itermWindupPointInv = 1.0f / (1.0f - itermWindupPoint);
    }
    //itermAcceleratorGain = pidProfile->itermAcceleratorGain;
    crashTimeLimitUs = pidProfile->crash_time * 1000;
    crashTimeDelayUs = pidProfile->crash_delay * 1000;
    crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    crashRecoveryRate = pidProfile->crash_recovery_rate;
    crashGyroThreshold = pidProfile->crash_gthreshold;
    crashDtermThreshold = pidProfile->crash_dthreshold;
    crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    crashLimitYaw = pidProfile->crash_limit_yaw;
    itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost;         // HF3D:  Now using this for flat piro compensation parameter testing
#endif
    itermRotation = pidProfile->iterm_rotation;
    //antiGravityMode = pidProfile->antiGravityMode;
    
    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    // antiGravityOsdCutoff = 1.0f;
    // if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        // antiGravityOsdCutoff += ((itermAcceleratorGain - 1000) / 1000.0f) * 0.25f;
    // }

#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
    // adapt setpoint threshold to user changes from default cutoff value
    itermRelaxSetpointThreshold = ITERM_RELAX_SETPOINT_THRESHOLD * ITERM_RELAX_CUTOFF_DEFAULT / itermRelaxCutoff;
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#if defined(USE_ABSOLUTE_CONTROL)
    acGain = (float)pidProfile->abs_control_gain;
    acLimit = (float)pidProfile->abs_control_limit;
    acErrorLimit = (float)pidProfile->abs_control_error_limit;
    acCutoff = (float)pidProfile->abs_control_cutoff;
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float iCorrection = -acGain * PTERM_SCALE / ITERM_SCALE * pidCoefficient[axis].Kp;
        pidCoefficient[axis].Ki = MAX(0.0f, pidCoefficient[axis].Ki + iCorrection);
    }
#endif

#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_min_hz > 0) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_PT1:
            dynLpfFilter = DYN_LPF_PT1;
            break;
        case FILTER_BIQUAD:
            dynLpfFilter = DYN_LPF_BIQUAD;
            break;
        default:
            dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        dynLpfFilter = DYN_LPF_NONE;
    }
    dynLpfMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynLpfMax = pidProfile->dyn_lpf_dterm_max_hz;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    useIntegratedYaw = pidProfile->use_integrated_yaw;
    integratedYawRelax = pidProfile->integrated_yaw_relax;
#endif

#ifdef USE_THRUST_LINEARIZATION
    thrustLinearization = pidProfile->thrustLinearization / 100.0f;
    if (thrustLinearization != 0.0f) {
        thrustLinearizationReciprocal = 1.0f / thrustLinearization;
        thrustLinearizationB = (1.0f - thrustLinearization) / (2.0f * thrustLinearization);
    }
#endif    
#if defined(USE_D_MIN)
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        const uint8_t dMin = pidProfile->d_min[axis];
        if ((dMin > 0) && (dMin < pidProfile->pid[axis].D)) {
            dMinPercent[axis] = dMin / (float)(pidProfile->pid[axis].D);
        } else {
            dMinPercent[axis] = 0;
        }
    }
    dMinGyroGain = pidProfile->d_min_gain * D_MIN_GAIN_FACTOR / D_MIN_LOWPASS_HZ;
    dMinSetpointGain = pidProfile->d_min_gain * D_MIN_SETPOINT_GAIN_FACTOR * pidProfile->d_min_advance * pidFrequency / (100 * D_MIN_LOWPASS_HZ);
    // lowpass included inversely in gain since stronger lowpass decreases peak effect
#endif
#if defined(USE_AIRMODE_LPF)
    airmodeThrottleOffsetLimit = pidProfile->transient_throttle_limit / 100.0f;
#endif
#ifdef USE_INTERPOLATED_SP
    ffFromInterpolatedSetpoint = pidProfile->ff_interpolate_sp;
    ffSmoothFactor = 1.0f - ((float)pidProfile->ff_smooth_factor) / 100.0f;
    interpolatedSpInit(pidProfile);
#endif

    // HF3D
    rescueCollective = pidProfile->rescue_collective;
    autoflipCollectiveMultiplier = pidProfile->autoflip_collective_multiplier / 100.0f;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
#endif

    // HF3D:  Setup our PID Delay Compensation Alpha multiplier with a maximum of 0.10 and a minimum of 0.001
    //  280 samples ==> 0.0036 would give the average of the last 280 samples of control output = subtracted off the output
    //  2.5x average is where you probably want to be... around 0.009, or a setting of 9
    // Uses crashflip_motor_percent for the setting since we have no use for that in a heli.
    /* delayCompAlpha = mixerConfig()->crashflip_motor_percent / 1000.0f; */
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    acroTrainerAxisState[FD_ROLL] = 0;
    acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorOutput)
{
    if (thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            motorOutput = sqrtf(motorOutput * thrustLinearizationReciprocal +
                                thrustLinearizationB * thrustLinearizationB) - thrustLinearizationB;
        }
    }
    return motorOutput;
}
#endif

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

#if defined(USE_ACC)
// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (horizonTiltExpertMode) {
        if (horizonTransition > 0 && horizonCutoffDegrees > 0) {
                    // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((horizonCutoffDegrees-currentInclination) / horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (horizonFactorRatio < 1.01f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180-currentInclination)/180 * (1.0f-horizonFactorRatio) + horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

STATIC_UNIT_TESTED float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {

    if (FLIGHT_MODE(ANGLE_MODE)) {
        // Angle mode is now rescue mode.
        autoflipInProgress = false;
        autoflipEngagedTime = 0;
        float errorAngle = 0.0f;

        // -90 Pitch is straight up and +90 is straight down
        // We are always pitching to "zero" whether up-right or inverted
        //   but the control direction needed to get to up-right is different than inverted
        // Determine if we're closer to up-right or inverted by checking for abs(roll attitude) > 90
        // HF3D TODO:  Evaluate using hysteresis to ensure we don't get "stuck" at one of the inflection points below.
        if (((attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) / 10.0f) > 90.0f) {
            // Rolled right closer to inverted, continue to roll right to inverted (+180 degrees)
            if (axis == FD_PITCH) {
                errorAngle = 0.0f + ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            } else if (axis == FD_ROLL) {
                errorAngle = 180.0f - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            }
        } else if (((attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) / 10.0f) < -90.0f) {    
            // Rolled left closer to inverted, continue to roll left to inverted (-180 degrees)
            if (axis == FD_PITCH) {
                errorAngle = 0.0f + ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            } else if (axis == FD_ROLL) {
                errorAngle = -180.0f - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            }
        } else {
            // We're rolled left or right between -90 and 90, and thus are closer to up-right (0 degrees aka skids down)
            if (axis == FD_PITCH) {
                errorAngle = 0.0f - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            } else if (axis == FD_ROLL) {
                errorAngle = 0.0f - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
            }
        }
        // NOTE:  If you want to level to only up-right, it would probably be best to just level inverted and
        //   then flip to upright later on.
        //   But if you really can only level to up-right (no negative collective avaiable), then it's probably
        //     best to flip use the pitch direction logic above but just always roll back to up-right.
        //   That way you will be putting in the correct pitch direction while you're inverted also.  Otherwise
        //     you will be putting in the wrong pitch correction during the time you're inverted.  Not good, especially
        //     since "roll" gets wonky near straight up/down.
        currentPidSetpoint = errorAngle * levelGain;   
        return currentPidSetpoint;

    } else if (airmodeIsEnabled() || autoflipInProgress) {
        // Auto-piroflip mode uses Airmode switch toggle to activate
        // Rescue (angle) mode has priority over autoflip (airmode), but autoflip has priority over horizon & gps rescue modes
        // NOTE:  Remember that this gets called for EACH axis on each PID iteration.
        
        //static timeDelta_t autoflipFlipTime = 0;
        static timeDelta_t autoflipYawTime = 0;
        //static int autoflipYawRotationsRemaining = 0;
        
        if (!autoflipInProgress) {
            // Check to ensure we've been engaged for at least 1.0 seconds before starting an autoflip
            if (autoflipEngagedTime <= 0) {
                // We weren't engaged... before now, so start our counter
                autoflipEngagedTime = micros();
            } else if ((micros() - autoflipEngagedTime) > 500000) {
                // Been engaged for > 0.5 seconds, wait for upright level and no cyclic stick inputs.
                if (isUpright() && MAX(fabsf(rcCommand[ROLL]), fabsf(rcCommand[PITCH])) < 15) {
                    // For now we will only support at least 90deg/s yaw rate and 90deg/s flip rate.
                    if (pidProfile->autoflip_yaw_rate >= 90 && pidProfile->autoflip_flip_rate >= 90) {
                        autoflipInProgress = true;
                        // Reset our engaged time for the start of the flip
                        autoflipEngagedTime = micros() + 250000;
                        autoflipYawTime = (1000000 * 360) / pidProfile->autoflip_yaw_rate;
                        autoflipFlipTime = (1000000 * 360) / pidProfile->autoflip_flip_rate;
                        // Set us up so we always return to the same tail orientation that we started in?
                        //autoflipYawRotationsRemaining = MIN(autoflip_yaw_rate / autoflip_flip_rate, 1);
                        // Begin the flip on whichever axis we're on at the start
                        if (axis == FD_PITCH) {
                            currentPidSetpoint = pidProfile->autoflip_flip_rate;
                        } else if (axis == FD_ROLL) {
                            currentPidSetpoint = 0;
                        } else if (axis == FD_YAW) {
                            currentPidSetpoint = pidProfile->autoflip_yaw_rate;
                        }
                    }
                }
            }
        } else {
            // Auto-flip is in progress
            // Engage Rescue (angle) mode if you want to bail out of an autoflip while it's still in progress.
            
            const timeDelta_t elapsedFlipTime = micros() - autoflipEngagedTime;
            if (elapsedFlipTime < 0) {
                // Wait for collective pump at beginning of flip
                return currentPidSetpoint;
            }
                
            // If we allow for non-piroflips in the future then this needs to be checked for autoflipYawTime == 0
            const float stirRadians = (M_PIf / 180.0f) * pidProfile->autoflip_yaw_rate * (elapsedFlipTime % autoflipYawTime) / 1000000;

            // Calculate the axis rate setpoints during the piroflip
            // The elapsedFlipTime checks are unnecessary, but keep them in case we decided to always rotate the tail back to the original orientation after the flip is completed.
            if (axis == FD_PITCH && elapsedFlipTime <= autoflipFlipTime) {
                // Start flip on the pitch axis (cosine).  Positive elevator is pitch forward.
                currentPidSetpoint = pidProfile->autoflip_flip_rate * cos_approx(stirRadians);
            } else if (axis == FD_ROLL && elapsedFlipTime <= autoflipFlipTime) {
                // Continue flip on the roll axis (sine).  Positive aileron is roll right.
                currentPidSetpoint = pidProfile->autoflip_flip_rate * sin_approx(stirRadians);
            } else if (axis == FD_YAW) {
                // Always rotate at our yaw_rate setting
                // For now we're only going to rotate nose left (negative rcCommand/positive setpoint)
                currentPidSetpoint = pidProfile->autoflip_yaw_rate;
            }
        
            // Check to see if we've completed our flip and decide what to do.
            if (!airmodeIsEnabled() && elapsedFlipTime >= autoflipFlipTime) {
                // If we reach the end of the flip and the autoflip mode is no longer active, then stop the flip.
                autoflipInProgress = false;
                autoflipEngagedTime = 0;
            } else if (elapsedFlipTime >= autoflipFlipTime) {
                // If we reach the end of our flip and autoflip mode is still active, then reset our timer so that we continue our flip
                autoflipEngagedTime = micros();
            }
        }
        
        return currentPidSetpoint;
    
    } else {
        // Horizon and GPS Rescue modes
        // calculate error angle and limit the angle to the max inclination
        // rcDeflection is in range [-1.0, 1.0]
        float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);

        // HF3D TODO:  Think about fixing GPS rescue for level/inverted rescue... probably just need to only make it worth it non-inverted rescue
#ifdef USE_GPS_RESCUE
        angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
        angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);

        const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);

        if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
            // ANGLE mode - control is angle based
            currentPidSetpoint = errorAngle * levelGain;
        } else {
            // HORIZON mode - mix of ANGLE and ACRO modes
            // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
            const float horizonLevelStrength = calcHorizonLevelStrength();
            currentPidSetpoint = currentPidSetpoint + (errorAngle * horizonGain * horizonLevelStrength);
        }
        
        return currentPidSetpoint;
    }
}

/* static timeUs_t crashDetectedAtUs;

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * levelGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset iterm, since accumulated error before crash is now meaningless
        // and iterm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees) {
                    inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
} */

/* static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !inCrashRecoveryMode
                && fabsf(delta) > crashDtermThreshold
                && fabsf(errorRate) > crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < crashSetpointThreshold) {
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm();
                } else {
                    inCrashRecoveryMode = true;
                    crashDetectedAtUs = currentTimeUs;
                }
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (fabsf(errorRate) < crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > crashSetpointThreshold)) {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (inCrashRecoveryMode) {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
} */
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((acroTrainerAxisState[axis] != 0) && (acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > acroTrainerAngleLimit) && (acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((acroTrainerAngleLimit * angleSign) - currentAngle) * acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {
        
        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }
 
        if (axis == acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

// Iterm Rotation:  Rotate the current iTerm vector properly as the craft rotates on other axes   (Pirouette Compensation)
// HF3D TODO:  Evaluate applicability of full iTerm rotation including Yaw axis.  Unsure if appropriate for helicopter use.
//      ArduCopter Piro Comp = Rotate integrator terms on X & Y axis based on pirouette rotation rate around the Z axis
//      Goal is to keep the swashplate tilted in the original direction of travel as the heli pirouettes
//      They do not rotate iTerm from the Z axis.
// HF3D TODO:  What is the lag from calculation of PID components to the actuation of the servos / change in blade pitch during fast rotations?
//      If the delay is significant, will the heli be in a slightly different z-axis orientation when the control output impacts flight?
//      Should look-forward prediction be used to rotate the error compensation to where it will actually occur in time for all PID gains on Roll & Pitch axis?
//      500 deg/s = 1 degree of rotation in 2ms.   dRonin measured response times are ~20ms.  So 10 degrees of rotation worst case?
//      2000rpm headspeed = 33Hz rotation of blades, and gyroscopic precession means that inputs take 90-degrees of rotation to occur
//        So worst case is that input determined while blade is at the point it needs to be controlled, rotates 90 degrees where CCPM mixing happens, then pitches 90 degrees later.
//        Total lag of 180-degrees of rotation worst case = 15ms @ 2000rpm   (After servos have been commanded and started to actually move)

//   Absolute control was made to address the same attitude issues as iterm_rotation, but without some of the downsides.
//   Absolute Control continuously measures the error of the quads path over stick input, properly rotated into the quads coordinate
//     system, and mixes a correction proportional to that error into the setpoint.  It's as if you noticed every tiny attitude 
//     error the quad incurs and provided an instantaneous correction on your TX.  The result is significantly better tracking
//     to sticks, particularly during rotations involving yaw and other difficult situations like throttle blips.
//   Absolute Control will likely eventually replace iterm_rotation, but it is not yet enabled by default.

// YOU SHOULD NOT ENABLE ABSOLUTE CONTROL AND ITERM ROTATION AT THE SAME TIME!
STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            // convert the deg/s rotation rate sensed by the gyro into the total radians of angle change during the last time step
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            // Rotate the calculated absolute control error for each axis based on the 3d rotation sensed by the gryo during the last time step
            rotateVector(axisError, rotationRads);
        }
#endif
        // itermRotation not to be used with absolute control
        if (itermRotation) {
            float v[XYZ_AXIS_COUNT];
            // Grab the old iTerm for each axis
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            // Rotate the iTerm among each axis based on the 3d rotation sensed by the gyro during the last time step
            rotateVector(v, rotationRads );
            // Overwrite the old iTerms with the rotated iTerms
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingDerivativeFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (setpointDerivativeLpfInitialized) {
        switch (rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                ret = pt1FilterApply(&setpointDerivativePt1[axis], pidSetpointDelta);
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                ret = biquadFilterApplyDF1(&setpointDerivativeBiquad[axis], pidSetpointDelta);
                break;
        }
        if (axis == rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER


//  YOU SHOULD NOT ENABLE ABSOLUTE CONTROL AND ITERM ROTATION AT THE SAME TIME!
//    HF3D TODO:  At least not until I make them compatible with each other.  :)
//  Absolute Control needs to be used with iTermRelax to avoid bounce-backs due to the latency between stick movement and quad response.
//    iTermRelax will then suspend AbsoluteControl error accumulation as well during quick moves.  Finally, AbsoluteControl only kicks in
//    once the throttle minimum for airmode activation is exceeded to avoid undue corrections on the ground.
//  Absolute control was made to address the same attitude issues as iterm_rotation, but without some of the downsides.
//  Absolute Control continuously measures the error of the quads orientation versus stick input, properly rotated into the quads coordinate
//     system, and mixes a correction proportional to that error into the setpoint.  It's as if you noticed every tiny attitude 
//     error the quad incurs and provided an instantaneous correction on your TX.  The result is significantly better tracking
//     to sticks, particularly during rotations involving yaw and other difficult situations like throttle blips.
#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        // Apply low-pass filter that was initialized with the pidProfile->abs_control_cutoff frequency to the roll rate command on this axis
        const float setpointLpf = pt1FilterApply(&acLpf[axis], *currentPidSetpoint);
        // Create high-pass filter by subtracting the commanded roll rate from the low-pass filtered version of itself
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        
        // NOTE:  This function runs on EACH AXIS INDEPENDENTLY.
        //   Even though one axis may have a fast stick movement, the other two axes may not.
        //   The other two axes will have full absolute control correction applied to them.
        
        // Create window around the low-pass filtered signal value
        //   No change in stick position ==> Lpf = commanded rate, Hpf = 0
        //      So, no stick movement = no window.  The window collapses to just the commanded rate on that axis.
        //   Fast stick movement ==>  Lpf = smoothed command rate, 1>Hpf>0
        // Note:  Commanded rate could also be a result of self-leveling or other modes
        // If roll rate sensed by Gyro is inside a window around the user's commanded setpoint, 
        //   then we'll accumulate the full error.
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        // Check to see if the roll rate sensed by the gyro is within the window of roll rates we created
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            // axisError is initialized to zero with pidInit and is only used by Absolute Control to accumulate error for each axis
            // Note:  axisError has already been compensated for the rotation of the aircraft that occurred during the last timestep dT
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidFrequency;
            }
        } else {
            // Roll rate sensed by gyro was outside of the window around the user's commanded Setpoint
            // Set the absolute control error rate to the maximum or minimum edges of the window and subtract the actual gyro rate
            // If no change in stick position, then this will simply be rcCommandRate - gyroRate   (so just the angle rate error)
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        // Check to ensure we are spooled up at a reasonable level
        if (isHeliSpooledUp()) {
            // Integrate the angle rate error, which gives us the accumulated angle error for this axis
            //  Limit the total angle error to the range defined by pidProfile->abs_control_error_limit
			if (axis == FD_ROLL || axis == FD_PITCH) {
				// Don't accumulate error if we hit our pidsumLimit on the previous loop through.
				if (fabsf(pidData[axis].Sum) >= PIDSUM_LIMIT) {
					acErrorRate = 0;
				}
			}
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * dT,
                -acErrorLimit, acErrorLimit);
            // Apply a proportional gain (abs_control_gain) to get a desired amount of correction
            //  Limit the total correction to the range defined by pidProfile->abs_control_limit
            const float acCorrection = constrainf(axisError[axis] * acGain, -acLimit, acLimit);
            // Manipulate the commanded roll rate on this axis by adding in the absolute control correction
            *currentPidSetpoint += acCorrection;
            // iTermErrorRate = currentPidSetpoint - gyroRate, but then is manipulated by iTermRelax to prevent wind-up during fast stick movements
            // Manipulate the iTerm Error Rate on this axis by adding in the absolute control correction
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }
        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

// Absolute Control needs to be used with iTermRelax to avoid bounce-backs due to the latency between stick movement and quad response.
// iTerm Relax:     https://www.youtube.com/watch?v=VhBL5u_g2LQ
//                  https://www.youtube.com/watch?v=QfiGTG5LfCk
//  iTerm accumulates error over the time (it looks into the past) and applies a correction for steady-state error
//  iTerm Relax accounts for the fact that fast rotation of the aircraft always has a slight lag versus the commanded rotation
//  This mechanical lag causes the iTerm to wind up over the course of a fast maneuver, which causes "bounceback" 
//  iTerm Relax prevents iTerm from growing very fast during fast accelerations of the control stick.
//    This means that the iTerm will not wind up during those "transition" periods at the beginning or the end of a movement of the control stick.
//  When the control stick is in a fixed position iTerm Relax will not impact the growth of iTerm.

// The lower the cutoff frequency the more that iTerm Relax will limit the change of iTerm during stick accelerations
//   The higher the cutoff frequency the shorter the window of time will be where the limitation is occuring
//   Default is 20Hz which is good for 5" miniquads.  Bigger quads can probably be lowered to 15Hz.  Requires testing.

// RP = Roll, Pitch axis only
// RPY = Roll, Pitch, Yaw
// RP_INC = Inc versions are only limiting the GROWTH of the iterm, while lowering of iTerm is not constrained
// RPY_INC = Inc versions are only limiting the GROWTH of the iterm, while lowering of iTerm is not constrained
// Setpoint mode applies a high-pass filter to RC input, resulting in a value that gets higher whenever the sticks are moved quickly.
//   When the rate of change is zero (sticks are not moving), iTerm accumulation is normal.  
//   Accumulation is then attenuated linearly as the stick movement approaches a threshold.  Above threshold, no iTerm accumulation occurs at all.
//   Tracks the actual movement on your stick, more suited for racer where smoothness is not required 
//   Setpoint is when you want the craft to go exactly where you told it to go
// Gyro mode uses a high-pass filter based on rate of change of stick movement, and uses this to create a window either side of the gyro value inside
//   which the quad should be tracking.  While inside the window, no iTerm accumulation occurs.  If the sticks are held still, the window compresses
//   back to nothing, and iTerm accumulation becomes normal again.
//   Gyro algorithm gives you more freestyle feel where result will be slightly smoother flight


// With iTerm relax we can push iTerm much higher than before without unfortunately rollback/bounceback
//    Up to 50% higher iTerm can work when using iTermRelax.
// HF3D TODO:  Evaluate iTerm relax cutoff frequency for helicopter use
//   (stick - stick lpf) = stick hpf, nice explanation Pawel. Also for planes ? I know current implementation for planes.
//   "No, planes will still have current Iterm limiting in iNav, we are not changing that. 
//      Planes are not that agile as drones and it would require very low cutoff frequency for Iterm relax to work"
STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    // Apply low-pass filter that was initialized with the pidProfile->iterm_relax_cutoff frequency to the roll rate command on this axis
    const float setpointLpf = pt1FilterApply(&windupLpf[axis], *currentPidSetpoint);
    // High pass filter = original signal minus the low-pass filtered version of that signal
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
    //   No change in stick position ==> Lpf = commanded rate, Hpf = 0
    //   Fast stick movement ==>  Lpf = smoothed command rate, 1>Hpf>0
    // Note:  Commanded rate could also be a result of self-leveling or other modes

    if (itermRelax) {
        if (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / itermRelaxSetpointThreshold);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing and allow iTerm to decrease normally.  Use the precalculed itermErrorRate
            } else if (itermRelaxType == ITERM_RELAX_SETPOINT) {
                // Change iTerm accumulation factor based only on the speed of the stick movement
                *itermErrorRate *= itermRelaxFactor;
            } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                // Accumulate iTerm if our gyro movement rate is within a window defined by our stick movement rate
                // Otherwise don't allow iTerm accumulation if the aircraft isn't tracking the commanded roll rate yet.
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        // Manipulate currentPidSetpoint and iTermErrorRate
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif  // end USE_ITERM_RELAX

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset)
{
    if (airmodeThrottleOffsetLimit == 0.0f) {
        return;
    }

    float offsetHpf = currentOffset * 2.5f;
    offsetHpf = offsetHpf - pt1FilterApply(&airmodeThrottleLpf2, offsetHpf);

    // During high frequency oscillation 2 * currentOffset averages to the offset required to avoid mirroring of the waveform
    pt1FilterApply(&airmodeThrottleLpf1, offsetHpf);
    // Bring offset up immediately so the filter only applies to the decline
    if (currentOffset * airmodeThrottleLpf1.state >= 0 && fabsf(currentOffset) > airmodeThrottleLpf1.state) {
        airmodeThrottleLpf1.state = currentOffset;
    }
    airmodeThrottleLpf1.state = constrainf(airmodeThrottleLpf1.state, -airmodeThrottleOffsetLimit, airmodeThrottleOffsetLimit);
}

float pidGetAirmodeThrottleOffset()
{
    return airmodeThrottleLpf1.state;
}
#endif


// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
// Called from subTaskPidController() in pid.c
//   Runs at equal to or slower than the gyro loop update frequency
static FAST_RAM_ZERO_INIT float yawPidSetpoint;         // HF3D:  Hold the previous corrected Yaw setpoint for flat piro compensation
static FAST_RAM_ZERO_INIT float collectiveStickPercent;        // HF3D
static FAST_RAM_ZERO_INIT float collectiveStickLPF;            // HF3D:  Collective stick values
static FAST_RAM_ZERO_INIT float collectiveStickHPF;

void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
#ifdef USE_INTERPOLATED_SP
    static FAST_RAM_ZERO_INIT uint32_t lastFrameNumber;
#endif

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
#endif

    const float tpaFactor = getThrottlePIDAttenuation();

#if defined(USE_ACC)
    // Get user-set accelerometer trims to trim the leveling modes
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

#ifdef USE_TPA_MODE
    const float tpaFactorKp = (currentControlRateProfile->tpaMode == TPA_MODE_PD) ? tpaFactor : 1.0f;
#else
    const float tpaFactorKp = tpaFactor;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    // Check for ICM gyro overflow "to the moon" yaw spin
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

#if defined(USE_ACC)
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    const bool levelModeActive = FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive;

    // Keep track of when we entered a self-level mode so that we can
    // add a guard time before crash recovery can activate.
    // Also reset the guard time whenever GPS Rescue is activated.
    if (levelModeActive) {
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }
    gpsRescuePreviousState = gpsRescueIsActive;
#endif

/*     // Dynamic i component,
    if ((antiGravityMode == ANTI_GRAVITY_SMOOTH) && antiGravityEnabled) {
        itermAccelerator = 1 + fabsf(antiGravityThrottleHpf) * 0.01f * (itermAcceleratorGain - 1000);
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(antiGravityThrottleHpf * 1000));
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(itermAccelerator * 1000)); */

    // gradually scale back integration when above windup point
    //float dynCi = dT * itermAccelerator;                   // itermAccelerator = 1.0f when antigravity disabled
    float dynCi = dT;
    // if (itermWindupPointInv > 1.0f) {                      // disabled if iterm_windup = 100, so dynCi = dT
        // dynCi *= constrainf((1.0f - getMotorMixRange()) * itermWindupPointInv, 0.0f, 1.0f);
    // }

    // Precalculate gyro data for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
#ifdef USE_RPM_FILTER
        gyroRateDterm[axis] = rpmFilterDterm(axis,gyroRateDterm[axis]);
#endif
        gyroRateDterm[axis] = dtermNotchApplyFn((filter_t *) &dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = dtermLowpass2ApplyFn((filter_t *) &dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    // HF3D:  iTermRotation acts as FFF Pirouette Compensation on a heli.
    //   Will not work properly unless the hover roll compensation is outside of the roll integral in the PID controller.
    rotateItermAndAxisError();
#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_INTERPOLATED_SP
    bool newRcFrame = false;
    if (lastFrameNumber != getRcFrameNumber()) {
        lastFrameNumber = getRcFrameNumber();
        newRcFrame = true;
    }
#endif

    // HF3D TODO:  Roll/Pitch PID compensation based on main rotor RPM
    //  Blade thrust varies with RPM^2.  Servos should need to move further at lower headspeeds to offset a given rotational error?
    //  Compensating PID gains with RPM (possibly throttle as a backup) might be helpful in reducing the need for banks of gains for each headspeed.
    //  Yaw axis PID RPM compensation would depend on whether it was a motor driven or variable pitch tail?

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

        // Get the user's roll rate command on this axis after expo and other rate modifications have been made
        //  Units are deg/s... maximum set in profile, default max rate is 1998 deg/s
        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
#if defined(USE_ACC)
        if (levelModeActive && (axis != FD_YAW)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        } else if (FLIGHT_MODE(ANGLE_MODE) && (axis == FD_YAW)) {
            // HF3D:  Don't allow user to give yaw input while rescue (angle) mode corrections are occuring
            currentPidSetpoint = 0.0f;
        } else if (airmodeIsEnabled() || autoflipInProgress) {
            // HF3D:  Autoflip mode call to pidlevel
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && acroTrainerActive && !inCrashRecoveryMode) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // HF3D TODO:  Flat pirouette compensation
        //  Compensate for the fact that the main shaft axis is not aligned with the Z axis due to the roll tilt required to compensate for tail blade thrust
        //  Create a wobble of the main shaft axis around the Z axis by adding roll and pitch commands proportional to the yaw rotation rate
        if (axis == FD_ROLL && throttleBoost > 1.0f) {
            // HF3D TODO:  Change roll compensation sign based on main motor rotation direction (tail thrust direction)
            currentPidSetpoint += fabsf(yawPidSetpoint) / throttleBoost;    // Roll compensation direction is same regardless of yaw direction
        } else if (axis == FD_PITCH && throttleBoost > 1.0f) {
            currentPidSetpoint += yawPidSetpoint / throttleBoost;          // Pitch compensation direction depends on yaw direction
        }

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y
/* #if defined(USE_ACC)
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);
#endif
 */
        const float previousIterm = pidData[axis].I;
        float itermErrorRate = errorRate;
#ifdef USE_ABSOLUTE_CONTROL
        float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        if (!inCrashRecoveryMode) {
            // Absolute Control is applied after iTermRelax as part of this function
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // HF3D:  After all setpoint changes are done, get Yaw setpoint for flat pirouette compensation
        if (axis == FD_YAW) {
            yawPidSetpoint = currentPidSetpoint;
        }

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidCoefficient[axis].Kp * errorRate * tpaFactorKp;
        if (axis == FD_YAW) {
            pidData[axis].P = ptermYawLowpassApplyFn((filter_t *) &ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki = pidCoefficient[axis].Ki;
		if (axis == FD_ROLL || axis == FD_PITCH) {
			// Don't accumulate error if we hit our pidsumLimit on the previous loop through.
			if (fabsf(pidData[axis].Sum) >= pidProfile->pidSumLimit) {
				Ki = 0;
			}
		} else {
            // Yaw axis
            if ((pidData[axis].Sum >= mixerGetYawPidsumAssistLimit()) || (pidData[axis].Sum <= -pidProfile->pidSumLimitYaw)) {
                Ki = 0;
            }
        }
        // dynCi = dT if airmode disabled and iterm_windup = 100
        pidData[axis].I = constrainf(previousIterm + Ki * itermErrorRate * dynCi, -itermLimit, itermLimit);
        
        // Decay accumulated error if appropriate
#define signorzero(x) ((x < 0) ? -1 : (x > 0) ? 1 : 0)
        if (!isHeliSpooledUp() || pidProfile->error_decay_always) {
            // Calculate number of degrees to remove from the accumulated error
            const float decayFactor = pidProfile->error_decay_rate * dT;

            pidData[axis].I -= signorzero(pidData[axis].I) * decayFactor * Ki;
#if defined(USE_ABSOLUTE_CONTROL)
            axisError[axis] -= signorzero(axisError[axis]) * decayFactor;
#endif
        }

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_INTERPOLATED_SP
        if (ffFromInterpolatedSetpoint) {
            pidSetpointDelta = interpolatedSpApply(axis, newRcFrame, ffFromInterpolatedSetpoint);
        } else {
            pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
        }
#else
        pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];
#endif
        previousPidSetpoint[axis] = currentPidSetpoint;


#ifdef USE_RC_SMOOTHING_FILTER
        pidSetpointDelta = applyRcSmoothingDerivativeFilter(axis, pidSetpointDelta);
#endif // USE_RC_SMOOTHING_FILTER

        // -----calculate D component
        if (pidCoefficient[axis].Kd > 0){

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta =
                - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency;

/* #if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif */

            float dMinFactor = 1.0f;
#if defined(USE_D_MIN)
            if (dMinPercent[axis] > 0) {
                float dMinGyroFactor = biquadFilterApply(&dMinRange[axis], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * dMinGyroGain;
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = dMinPercent[axis] + (1.0f - dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt1FilterApply(&dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_MIN, 0, lrintf(dMinGyroFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 1, lrintf(dMinSetpointFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 2, lrintf(pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_MIN, 3, lrintf(pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                }
            }
#endif
            pidData[axis].D = pidCoefficient[axis].Kd * delta * tpaFactor * dMinFactor;
        } else {
            pidData[axis].D = 0;
        }
        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component
#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in FF
        pidSetpointDelta += setpointCorrection - oldSetpointCorrection[axis];
        oldSetpointCorrection[axis] = setpointCorrection;
#endif

        // Only enable feedforward for rate mode (flightModeFlag=0 is acro/rate mode)
        //  HF3D:  Changed this so horizon & angle modes have feedforward also
        //const float feedforwardGain = (flightModeFlags) ? 0.0f : pidCoefficient[axis].Kf;
        const float feedforwardGain = pidCoefficient[axis].Kf;
        static timeUs_t lastTimeEleOutsideWindow = 0;
        float eleOffset = 0.0f;
        
        if (feedforwardGain > 0) {
            // transition = 1 if feedForwardTransition == 0   (no transition)
            float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1.0f;

            // Apply elevator filter to stop bounces due to sudden stops on the pitch axis near zero deg/s
            //  These do not seem to be related to I term or Absolute control, or really any of the PID terms
            //  High feedforward gain seems to make them worse due to the aggressive nature of the return to zero.
            if (axis == FD_PITCH && pidProfile->elevator_filter_gain > 0) {
                
                float elevatorSetpointLPF = elevatorFilterLowpassApplyFn((filter_t *) &elevatorFilterLowpass, currentPidSetpoint);
                
                // Store the last time we were outside of the deg/s window we decided upon
                if (fabsf(currentPidSetpoint) >= pidProfile->elevator_filter_window_size) {
                    lastTimeEleOutsideWindow = currentTimeUs;
            
                } else if (cmpTimeUs(currentTimeUs, lastTimeEleOutsideWindow) < (pidProfile->elevator_filter_window_time * 1000)) {
                    // We're inside the deg/s window and we've recently been outside of it
                    // Cutoff time for compare should be maybe 2-3x the cutoff frequency period??
                    //  Note that this will also catch the times that we're just transiting through center quickly... which sucks, but what is the other choice??
                    //  Luckily, the moment we get outside the window we'll go back to full blast of feedforward on the pitch axis, so it may not be very noticeable during fast stick movements.

                    // Apply the elevator filter offset to help gradually slow us down.
                    // There will be a slightly discontinuity in the pitch feedforward as we hit the elevator filter window.
                    // Should be relative to the amount of feedforward gain.  More Kf -> More elevator offset.
                    // elevator_filter_gain = 100 will give 1:1 offset between the LPF and setPoint.
                    //   As we enter the window eleOffset will start high and slowly decay down to zero over time.
                    eleOffset = (elevatorSetpointLPF - currentPidSetpoint) * feedforwardGain * 90.0f * transition * pidProfile->elevator_filter_gain / 100.0f;
                }
                
                // If it's been a while since we've been outside of our cutoff window then don't do anything different.
                //   Otherwise we will slow down fast stick movements, and we don't want that!  We only want to change the stop characteristics around center stick.
            }
            
            // HF3D:  Direct stick feedforward for roll and pitch.  Stick delta feedforward for yaw.
            // Let's do direct stick feedforward for roll & pitch, and let's AMP IT UP A LOT.
            // 0.013754 * 90 * 1 * 60 deg/s = 74 output for 100 feedForward gain
            float feedForward = feedforwardGain * 90.0f * transition * currentPidSetpoint;
            if (axis == FD_PITCH) {
                feedForward += eleOffset;
                // Store the elevator filter offset value into the AC_CORRECTION debug channel (absolute control) since it isn't used otherwise
                DEBUG_SET(DEBUG_AC_CORRECTION, 3, lrintf(eleOffset));
            } else if (axis == FD_YAW) {
                // Stick delta feedforward for the yaw axis.
                feedForward = feedforwardGain * transition * pidSetpointDelta * pidFrequency;    //  Kf * 1 * 20 deg/s * 8000
            }

#ifdef USE_INTERPOLATED_SP
            // HF3D:  Only apply feedforward interpolation limits to the Yaw axis since we're using direct feedforward on the roll and pitch axes.
            if (axis == FD_YAW) {
                pidData[axis].F = shouldApplyFfLimits(axis) ?
                    applyFfLimit(axis, feedForward, pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
            } else {
               pidData[axis].F = feedForward;
            }               
#else
            pidData[axis].F = feedForward;
#endif
        } else {
            pidData[axis].F = 0;
        }
        
         // HF3D:  Calculate tail feedforward precompensation and add it to the pidSum on the Yaw channel
        if (axis == FD_YAW) {
            
            // Calculate absolute value of the percentage of collective stick throw
            if ((rcCommand[COLLECTIVE] >= 500) || (rcCommand[COLLECTIVE] <= -500)) {
                collectiveStickPercent = 100.0f;
            } else {
                if (rcCommand[COLLECTIVE] >= 0) {
                    collectiveStickPercent = (rcCommand[COLLECTIVE] * 100.0f) / (PWM_RANGE_MAX - rxConfig()->midrc);
                } else if (rcCommand[COLLECTIVE] < 0) {
                    collectiveStickPercent = (rcCommand[COLLECTIVE] * -100.0f) / (rxConfig()->midrc - PWM_RANGE_MIN);
                }
            }
            
            // Collective pitch impulse feed-forward for the main motor
            // Run our collectiveStickPercent through a low pass filter
            collectiveStickLPF = collectiveStickLPF + collectivePulseFilterGain * (collectiveStickPercent - collectiveStickLPF);
            // Subtract LPF from the original value to get a high pass filter
            // HPF value will be <60% or so of the collectiveStickPercent, and will be smaller the slower the stick movement is.
            //  Cutoff frequency determines this action.
            collectiveStickHPF = collectiveStickPercent - collectiveStickLPF;

            // HF3D TODO:  Negative because of clockwise main rotor spin direction -> CCW body torque on helicopter
            //   Implement a configuration parameter for rotor rotation direction
            float tailCollectiveFF = -1.0f * collectiveStickPercent * pidProfile->yawColKf / 100.0f;
            float tailCollectivePulseFF = -1.0f * collectiveStickHPF * pidProfile->yawColPulseKf / 100.0f;
            float tailBaseThrust = -1.0f * pidProfile->yawBaseThrust / 10.0f;
            
            // Calculate absolute value of the percentage of cyclic stick throw (both combined... but swash ring is the real issue).
			float tailCyclicFF = -1.0f * servosGetSwashRingValue() * 100.0f * pidProfile->yawCycKf / 100.0f;
            
            // Main motor torque increase from the ESC is proportional to the absolute change in average voltage (NOT percent change in average voltage)
            //     and it is linear with the amount of change.
            // Main motor torque will always cause the tail to rotate in in the same direction, so we just need to apply this offset in the correct direction to counter-act that torque.
            // For CW rotation of the main rotor, the torque will turn the body of the helicopter CCW
            //   This means the leading edge of the tail blade needs to tip towards the left side of the helicopter to counteract it.
            //   Yaw stick right = clockwise rotation = tip of tail blade to left = POSITIVE servo values observed on the X3
            //      NOTE:  Servo values required to obtain a certain direction of change in the tail depends on which side the servo is mounted and if it's a leading or trailing link!!
            //      It is very important to set the servo reversal configuration up correctly so the servo moves in the same direction as the Preview model!
            //   Yaw stick right = positive control channel PWM values from the TX also.
            //   Smix on my rudder channel is setup for -100... so it will take NEGATIVE values in the pidSum to create positive servo movement!

            //   So my collective comp also needs to make positive tail servo values, which means that I need a NEGATIVE adder to the pidSum due to the channel rate reversal.
            //     But, this should always work for all helis that are also setup correctly, regardless of servo reversal...
            //     ... as long as it yaws the correct direction to match the Preview model.
            //     ... and as long as the main motor rotation direction is CW!  CCW rotation will require reversing this and generating POSITIVE pidSum additions!
            // HF3D TODO:  Add a "main rotor rotation direction" configuration parameter.
            
            // HF3D TODO:  Consider adding a delay in here to allow time for other things to occur...
            //  Tail servo can probably add pitch faster than the swash servos can add collective pitch
            //   and it's also probably faster than the ESC can add torque to the main motor (for gov impulse)
            //  But for motor-driven tails the delay may not be needed... depending on how fast the ESC+motor
            //   can spin up the tail blades relative to the other pieces of the puzzle.
            
            // Add our collective feedforward terms into the yaw axis pidSum as long as we don't have a motor driven tail
            // Only if we're armed and throttle > 15 use the tail feedforwards.  If we're auto-rotating then there's no use for all this stuff.  It will just screw up our tail position since there's no main motor torque!!
            // HF3D TODO:  Add a configurable override for this check in case someone wants to run an external governor without passing the throttle signal through the flight controller?
            if ((calculateThrottlePercentAbs() > 15) || (!ARMING_FLAG(ARMED))) {
                // if disarmed, show the user what they will get regardless of throttle value
                pidData[FD_YAW].F += tailCollectiveFF + tailCollectivePulseFF + tailBaseThrust + tailCyclicFF;
            } 
            // HF3D TODO:  Do some integration of the motor driven tail code here for motorCount == 2...
            //   But have to be careful, because if main motor throttle goes near zero then we'll never get the tail back if we're
            //   adding feedforward that doesn't need to be there.  We can't create negative thrust with a motor driven fixed pitch tail.
                        
        }       

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin 
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

        // calculating the PID sum
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && useIntegratedYaw) {
            pidData[axis].Sum += pidSum * dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * integratedYawRelax / 100000.0f * dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
        
    }

/*     // HF3D:  PID Delay Compensation
    //  Allows for higher PID gains since the previous control actions the helicopter hasn't responded to yet are taken into account in the PID controller output
    // Update the PID sums by subtracting the previous unresponded control inputs from the current PID controller outputs
    pidData[FD_ROLL].Sum -= delayCompAlpha * delayCompSum[FD_ROLL];
    pidData[FD_PITCH].Sum -= delayCompAlpha * delayCompSum[FD_PITCH];
    
    // Update the delay array values & sums
    // Subtract the oldest value from the array since it will have caused a response by now
    delayCompSum[FD_ROLL] -= delayArrayRoll[delayArrayPos];
    delayCompSum[FD_PITCH] -= delayArrayPitch[delayArrayPos];
    // Replace the oldest value with the newest value
    delayArrayRoll[delayArrayPos] = (int16_t) constrainf(pidData[FD_ROLL].Sum, -pidProfile->pidSumLimit, pidProfile->pidSumLimit);
    delayArrayPitch[delayArrayPos] = (int16_t) constrainf(pidData[FD_PITCH].Sum, -pidProfile->pidSumLimit, pidProfile->pidSumLimit);
    // Add the newest outputs to the sums
    delayCompSum[FD_ROLL] += delayArrayRoll[delayArrayPos];
    delayCompSum[FD_PITCH] += delayArrayPitch[delayArrayPos];
    // Increment the delay array position pointer
    delayArrayPos++;
    if (delayArrayPos >= DELAYLENGTH) {
        delayArrayPos = 0;    // Reset pointer into array to the beginning
    } */

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very inefficient, but it is done on purpose to always show real CPU usage as in flight
    // HF3D TODO:  We're not really using this right now since we're decaying accumulated error in the main PID loop to handle initial spool-up
    if (!pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
/*         delayCompSum[FD_ROLL] = 0;
        delayCompSum[FD_PITCH] = 0; */
    } else if (zeroThrottleItermReset) {
        pidResetIterm();
    }
    
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

/* void pidSetAntiGravityState(bool newState)
{
    if (newState != antiGravityEnabled) {
        // reset the accelerator on state changes
        itermAccelerator = 1.0f;
    }
    antiGravityEnabled = newState;
} */

/* bool pidAntiGravityEnabled(void)
{
    return antiGravityEnabled;
} */

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (dynLpfFilter != DYN_LPF_NONE) {
        const unsigned int cutoffFreq = fmax(dynThrottle(throttle) * dynLpfMax, dynLpfMin);

         if (dynLpfFilter == DYN_LPF_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, dT));
            }
        } else if (dynLpfFilter == DYN_LPF_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
        }
    }
}
#endif

void pidSetItermReset(bool enabled)
{
    zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}

float pidGetDT()
{
    return dT;
}

float pidGetPidFrequency()
{
    return pidFrequency;
}

float pidGetCollectiveStickPercent()
{
    return collectiveStickPercent;
}

float pidGetCollectiveStickHPF()
{
    return collectiveStickHPF;
}

uint16_t pidGetRescueCollectiveSetting()
{
    return rescueCollective;
}

float pidGetCollectivePulseFilterGain(void)
{
    return collectivePulseFilterGain;
}

bool pidGetAutoflipInProgress(void)
{
    return autoflipInProgress;
}

timeDelta_t pidGetAutoflipFlipTime(void)
{
    return autoflipFlipTime;
}

timeUs_t pidGetAutoflipEngagedTime(void)
{
    return autoflipEngagedTime;
}

float pidGetAutoflipCollectiveMultiplier(void)
{
    return autoflipCollectiveMultiplier;
}
