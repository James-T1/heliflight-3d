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

#include "config/feature.h"

#include "pg/motor.h"
#include "pg/rx.h"

#include "drivers/dshot.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/core.h"
#include "fc/rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

#define DYN_LPF_THROTTLE_STEPS           100
#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 5000 // minimum of 5ms between updates

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .mixerMode = DEFAULT_MIXER,
    .yaw_motors_reversed = false,
    .crashflip_motor_percent = 0,
    .gov_max_headspeed = 7000,
    .gov_gear_ratio = 100,
    .gov_p_gain = 0,
    .gov_i_gain = 0,
    .gov_cyclic_ff_gain = 0,
    .gov_collective_ff_gain = 0,
    .gov_collective_ff_impulse_gain = 0,
    .gov_collective_ff_impulse_freq = 100,
    .spoolup_time = 5
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

#define PWM_RANGE_MID 1500

static FAST_RAM_ZERO_INIT uint8_t motorCount;

float FAST_RAM_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

static FAST_RAM_ZERO_INIT int throttleAngleCorrection;

// HF3D TODO:  Remove all the useless mixers and replace them with CCPM mixes for helicopters
//     May require updating configurator?
static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
#ifndef USE_QUAD_MIXER_ONLY
static const motorMixer_t mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};
#else
#define mixerBicopter NULL
#endif

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};


#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};
static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};
#else
#define mixerHex6H NULL
#define mixerHex6P NULL
#define mixerY6 NULL
#endif // USE_UNCOMMON_MIXERS
#else
#define mixerHex6X NULL
#endif // MAX_SUPPORTED_MOTORS >= 6

#if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
    { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
    { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
    { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
    { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
};
#else
#define mixerOctoX8 NULL
#define mixerOctoFlatP NULL
#define mixerOctoFlatX NULL
#endif

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
    { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },          // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },          // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },          // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },          // FRONT_L
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};
#else
#define mixerDualcopter NULL
#endif

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const motorMixer_t mixerQuadX1234[] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
};

// Keep synced with mixerMode_e
// Some of these entries are bogus when servos (USE_SERVOS) are not configured,
// but left untouched to keep ordinals synced with mixerMode_e (and configurator).
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL },                // entry 0
    { 3, true,  mixerTricopter },      // MIXER_TRI
    { 4, false, mixerQuadP },          // MIXER_QUADP
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 2, true,  mixerBicopter },       // MIXER_BICOPTER
    { 0, true,  NULL },                // * MIXER_GIMBAL
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
    { 6, false, mixerHex6X },          // MIXER_HEX6X
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 1, true,  mixerSingleProp },     // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
    { 6, false, mixerHex6H },          // MIXER_HEX6H
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
    { 4, false, mixerQuadX1234 },
};
#endif // !USE_QUAD_MIXER_ONLY

FAST_RAM_ZERO_INIT float motorOutputHigh, motorOutputLow;

static FAST_RAM_ZERO_INIT float disarmMotorOutput, deadbandMotor3dHigh, deadbandMotor3dLow;
static FAST_RAM_ZERO_INIT float rcCommandThrottleRange;
#ifdef USE_DYN_IDLE
static FAST_RAM_ZERO_INIT float idleMaxIncrease;
static FAST_RAM_ZERO_INIT float idleThrottleOffset;
static FAST_RAM_ZERO_INIT float idleMinMotorRps;
static FAST_RAM_ZERO_INIT float idleP;
#endif
// Governor & Spool-up
static FAST_RAM_ZERO_INIT float rampRate;  // = 0.0002f     // Used for spool-up logic.  Default is 5 seconds at 1kHz.
static FAST_RAM_ZERO_INIT uint16_t govMaxHeadspeed;
static FAST_RAM_ZERO_INIT float govGearRatio;
static FAST_RAM_ZERO_INIT float govKp;
static FAST_RAM_ZERO_INIT float govKi;
static FAST_RAM_ZERO_INIT float govCycKf;
static FAST_RAM_ZERO_INIT float govColKf;
static FAST_RAM_ZERO_INIT float govColPulseKf;
static FAST_RAM_ZERO_INIT float govColPulseFc;
static FAST_RAM_ZERO_INIT float govColPulseFilterGain;   
static FAST_RAM_ZERO_INIT float govBaseThrottleFilterGain;   // Used for governor failure

uint8_t getMotorCount(void)
{
    return motorCount;
}

float getMotorMixRange(void)
{
    //return motorMixRange;     // HF3D TODO:  Remove motorMixRange completely
    return 0.0f;
}

bool areMotorsRunning(void)
{
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;

                break;
            }
        }
    }

    return motorsRunning;
}

#ifdef USE_SERVOS
bool mixerIsTricopter(void)
{
    return (currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI);
}
#endif

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void)
{
    float motorOutputLimit = 1.0f;
    if (currentPidProfile->motor_output_limit < 100) {
        motorOutputLimit = currentPidProfile->motor_output_limit / 100.0f;
    }

    motorInitEndpoints(motorOutputLimit, &motorOutputLow, &motorOutputHigh, &disarmMotorOutput, &deadbandMotor3dHigh, &deadbandMotor3dLow);

    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
}

// called from init at FC startup.
void mixerInit(mixerMode_e mixerMode)
{
    currentMixerMode = mixerMode;

    initEscEndpoints();
#ifdef USE_SERVOS
    if (mixerIsTricopter()) {
        mixerTricopterInit();
    }
#endif
#ifdef USE_DYN_IDLE
    idleMinMotorRps = currentPidProfile->idle_min_rpm * 100.0f / 60.0f;
    idleMaxIncrease = currentPidProfile->idle_max_increase * 0.001f;
    idleThrottleOffset = motorConfig()->digitalIdleOffsetValue * 0.0001f;
    idleP = currentPidProfile->idle_p * 0.0001f;
#endif
    
    // Initialize governor settings
    // Determine rampRate that will increase throttle on each loop to go from 0% throttle to 100% throttle in rampTime seconds
    // We're stealing the mmix throttle% section on motor 0 to be our ramp-up time instead. 
    // rampTime is how many seconds to go from 0rpm to 100% pwm output
    // mmix 1 throttle = 5.0 ==> 5 seconds to spool up from 0% to 100% throttle
    //   activemixer.throttle = floating point value between 0-1.00
    // targetPidLooptime is in units of microseconds...  (8kHz => targetPidLooptime = 125)
    //   5 seconds = @ 8kHz ==> 40,000 loops for full spool-up from 0% to 100%
    //rampRate = targetPidLooptime / (currentMixer[0].throttle * 1e6f);    
    //rampRate = 0.000025f;    // HF3D TODO:  Figure out why the line of code above doesn't work.
        // targetPidLooptime should be (uint32_t) 125
        // currentMixer[0].throttle should be  (float) 5.0
        // 1e6f should be 1000000
        // SO WHY DOESN'T IT WORK?????  Result should be 0.000025f, but instead the stupid heli just won't spool up.
    
    rampRate = 0.000025f;
    //rampRate = pidGetDT() / (float)mixerConfig()->spoolup_time;
    // HF3D TODO:  lol... rampRate STILL not working.  Wtf.
    govMaxHeadspeed = mixerConfig()->gov_max_headspeed;     // stays uint16_t
    govGearRatio = (float)mixerConfig()->gov_gear_ratio / 100.0f;
    govKp = (float)mixerConfig()->gov_p_gain / 10.0f;
    govKi = (float)mixerConfig()->gov_i_gain / 10.0f;
    govCycKf = (float)mixerConfig()->gov_cyclic_ff_gain / 10.0f;
    govColKf = (float)mixerConfig()->gov_collective_ff_gain / 10000.0f;
    govColPulseKf = (float)mixerConfig()->gov_collective_ff_impulse_gain / 10000.0f;
    govColPulseFc = (float)mixerConfig()->gov_collective_ff_impulse_freq / 100.0f;     // Setting is frequency in Hz / 100
    
    // Setup filter for governor base throttle & Collective Impulse Feedforward
    // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
    //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
    //   k = dT / (RC + dT);                  ==>  k = 0.0000393 for 8kHz
    govBaseThrottleFilterGain = pidGetDT() / (pidGetDT() + (1 / ( 2 * 3.14159f * 0.05f)));
    govColPulseFilterGain = pidGetDT() / (pidGetDT() + (1 / ( 2 * 3.14159f * govColPulseFc)));
}


#ifndef USE_QUAD_MIXER_ONLY
// called from init at FC startup.
void mixerConfigureOutput(void)
{
    motorCount = 0;

    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // Check if done by seeing if this motor has any mixing
            // HF3D:  Also allow for tail motors by checking if motor is assigned to Yaw channel OR Throttle channel
            if ((customMotorMixer(i)->throttle == 0.0f) && (customMotorMixer(i)->yaw == 0.0f)) {
                break;
            }
            currentMixer[i] = *customMotorMixer(i);
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (motorCount > MAX_SUPPORTED_MOTORS) {
            motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
    mixerResetDisarmedMotors();
    
}

void mixerLoadMix(int index, motorMixer_t *customMixers)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i].throttle = 0.0f;
    }
    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (int i = 0; i < mixers[index].motorCount; i++) {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}
#else
void mixerConfigureOutput(void)
{
    motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetDisarmedMotors();
}
#endif // USE_QUAD_MIXER_ONLY

void mixerResetDisarmedMotors(void)
{
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_RAM_ZERO_INIT float throttle = 0;
static FAST_RAM_ZERO_INIT float mixerThrottle = 0;
static FAST_RAM_ZERO_INIT float motorOutputMin;
static FAST_RAM_ZERO_INIT float motorRangeMin;
static FAST_RAM_ZERO_INIT float motorRangeMax;
static FAST_RAM_ZERO_INIT float motorOutputRange;
static FAST_RAM_ZERO_INIT int8_t motorOutputMixSign;


static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;
#ifdef USE_DYN_IDLE
    static float oldMinRps;
#endif
    float currentThrottleInputRange = 0;

    if (featureIsEnabled(FEATURE_3D)) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
            // INVERTED
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else
#endif
            {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;

#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else
#endif
            {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }

            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;

            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < 250000) {
            // keep iterm zero for 250ms after motor reversal
            pidResetIterm();
        }
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
#ifdef USE_DYN_IDLE
        if (idleMinMotorRps > 0.0f) {
            motorOutputLow = DSHOT_MIN_THROTTLE;
            const float maxIncrease = isAirmodeActivated() ? idleMaxIncrease : 0.04f;
            const float minRps = rpmMinMotorFrequency();
            const float targetRpsChangeRate = (idleMinMotorRps - minRps) * currentPidProfile->idle_adjustment_speed;
            const float error = targetRpsChangeRate - (minRps - oldMinRps) * pidGetPidFrequency();
            const float pidSum = constrainf(idleP * error, -currentPidProfile->idle_pid_limit, currentPidProfile->idle_pid_limit);
            motorRangeMinIncrease = constrainf(motorRangeMinIncrease + pidSum * pidGetDT(), 0.0f, maxIncrease);
            oldMinRps = minRps;
            throttle += idleThrottleOffset * rcCommandThrottleRange;

            DEBUG_SET(DEBUG_DYN_IDLE, 0, motorRangeMinIncrease * 1000);
            DEBUG_SET(DEBUG_DYN_IDLE, 1, targetRpsChangeRate);
            DEBUG_SET(DEBUG_DYN_IDLE, 2, error);
            DEBUG_SET(DEBUG_DYN_IDLE, 3, minRps);
            
        }
#endif
        // if not in 3D mode:    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
        currentThrottleInputRange = rcCommandThrottleRange;
        // motorOutputLow includes the increase from digitalIdleOffset!
        // if DynIdle not used:  motorRangeMinIncrease = 0
        motorRangeMin = motorOutputLow + motorRangeMinIncrease * (motorOutputHigh - motorOutputLow);
        motorRangeMax = motorOutputHigh;
        motorOutputMin = motorRangeMin;
        // if DynIdle not used:  motorOutputRange = motorOutputHigh - motorOutputLow
        motorOutputRange = motorOutputHigh - motorOutputMin;
        motorOutputMixSign = 1;
    }

    // if not in 3D Mode and DynIdle not used:  throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
    //    so, without throttleAngleCorrection:  throttle = rcCommand[THROTTLE]
    // Throttle Angle Correction is probably not a great idea for helicopters.
    //    To disable Throttle Angle Correction, set:  throttleCorrectionConfig()->throttle_correction_value = 0
    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
    //    and without any extra features:       throttle = rcCommand[THROTTLE] / (PWM_RANGE_MAX - PWM_RANGE_MIN)
    //    Essentially just converting throttle from a value in microseconds to a float between 0.0 and 1.0
}

// HF3D TODO:  Move governor logic to separate source file
static FAST_RAM_ZERO_INIT float lastSpoolThrottle = 0;
static FAST_RAM_ZERO_INIT uint8_t spooledUp = 0;
static FAST_RAM_ZERO_INIT float governorSetpoint;
static FAST_RAM_ZERO_INIT float governorSetpointLimited = 0.0f;
static FAST_RAM_ZERO_INIT float govBaseThrottle;
static FAST_RAM_ZERO_INIT float govPidSum = 0;
static FAST_RAM_ZERO_INIT float govI = 0;
static FAST_RAM_ZERO_INIT float govCollectiveFF = 0;
static FAST_RAM_ZERO_INIT float govCollectivePulseFF = 0;
static FAST_RAM_ZERO_INIT timeMs_t lastSpoolEndTime = 0;

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS], motorMixer_t *activeMixer)
{
    // HF3D: Re-wrote this section for main and optional tail motor use.  No longer valid for multirotors.
    //   Main motor must be motor[0] (Motor 0)
    //     * Use resource assignment to reassign output pin on board for main motor ESC to Motor 1
    //     * mmix 0 uses the Throttle% to determine ramp rate
    //   Tail motor is optional and must be motor[1] (Motor 1)
    //     * If used, tail motor should have 100% yaw mixing
    //     * mmix 1 (tail motor) uses the Throttle% to determine tailMotorBaseThrustGain
    //     *    1.0 would be way too high.  Full tail thrust when head is at 100% rpm.
    float mainMotorThrottle = 0.0f;         // Used by the tail code to set the base tail motor output as a fraction of main motor output

    // Handle MAIN motor (motor[0]) throttle output & spool-up
    if (motorCount > 0) {

        // Calculate headspeed
        float mainMotorRPM = rpmGetFilteredMotorRPM(0);    // Get main motor rpm  --- what about calcESCrpm if DSHOT not available, but normal ESC telemetry is?
        float headspeed = mainMotorRPM / govGearRatio;
                
        // Some logic to help us come back from a stage 1 failsafe / glitch / RPM loss / accidental throttle hold quickly
        // We're going to use this time to lock in our spooledUp state for a few seconds after throttle = 0 when we were just spooledUp on the last pass through
        // Also gives us a few second window if we lose headspeed signal... in that case we'll fall back to the commanded throttle value
        if (spooledUp && (throttle == 0.0f || headspeed < 1000.0f) && cmp32(millis(), lastSpoolEndTime) > 5500) {
            // Time check above must be set just a little longer than any of the lastSpoolEndTime checks below.
            // HF3D TODO:  Maybe change the throttle check above to something >0.0f or add some more logic to allow for a faster ramp for autorotation bailout of some kind?
            lastSpoolEndTime = millis();
        }
                
        // Determine governor setpoint (use governor if throttle setting is >50%)
        // HF3D TODO:  Check !isDshotMotorTelemetryActive(i)
        if (throttle > 0.50) {

            // Set the user requested governor headspeed setting
            governorSetpoint = throttle * (float)govMaxHeadspeed;
            
            // If we don't have a non-zero rate limited setpoint yet, set it to the headspeed
            if (governorSetpointLimited <= 0) {
                governorSetpointLimited = headspeed;
            }
            
            // Increment or decrement the rate limited governor setpoint if needed
            // If ramp is set to 5s then this will allow 20% change in 1 second, or 10% headspeed in 0.5 seconds.
            float govRampRate = rampRate * (float)govMaxHeadspeed;
            // Check to see if we've been spooledUp recently .  If so, increase our govRampRate greatly to help recover from temporary loss of throttle signal.
            // HF3D TODO:  If someone immediately plugged in their heli, armed, and took off throttle hold we could accidentally hit this fast governor ramp.  That would be crazy... but possible I guess?
            if ( spooledUp && cmp32(millis(), lastSpoolEndTime) < 5000 ) {
                govRampRate *= 7.0f;
            }
            if ((governorSetpoint - governorSetpointLimited) > govRampRate) {
                // Setpoint is higher than the rate limited setpoint, so increment limited setpoint higher
                governorSetpointLimited = constrainf(governorSetpointLimited + govRampRate, governorSetpointLimited, governorSetpoint);                
            
            } else if ((governorSetpointLimited - governorSetpoint) > govRampRate)  {
                // Setpoint is lower than the rate limited setpoint, so decrement limited setpoint lower
                governorSetpointLimited = constrainf(governorSetpointLimited - govRampRate, governorSetpoint, governorSetpointLimited);
            }
        } else {
            // Throttle is less than 50%, don't use governor.
            governorSetpoint = 0;
            governorSetpointLimited = 0;
        }

        if (headspeed == 0) {
            // Disable governor if main motor RPM is not available
            governorSetpoint = 0;
            governorSetpointLimited = 0;
        }
    
        //----------------- Spoolup Logic ----------------
        // MVP:  If Main Motor RPM <1000, then reset spooledUp flag and use spooledUp logic
        // HF3D TODO:  Add auto-rotation mode that ignores this check, or at least decreases the ramp time significantly.
        // HF3D TODO:  Clean-up spool-up logic that allows for 2 modes:  Spool on throttle % only (no RPM telemetry), and Spool on Governor (Headspeed)
        //    May be easier to just have 2 sets of code... 1 for spooling on governor and one for spooling without governor??
        // HF3D TODO:
        //  Idea for RPM-based spool-up logic when governor is active...
        //  Bring throttle up to ~25%.  If no RPM detected, then disarm.
        //  Once RPM is detected, start tracking PWM vs. RPM.  
        //  If there becomes a large discrepency between PWM ramp and RPM ramp, then stop ramping PWM until ratio comes back in line.
        //  This will prevent the "lag" that sometimes occurs where we end up ramping PWM into oblivion and then the ESC "catches up" later on.
        //  See 4/9/20 log #7

        // Spool-up using governorSetpoint (not the rate limited GovernorSetpoint)
        // Determine spoolup status
        if (headspeed < 1000.0f && cmp32(millis(), lastSpoolEndTime) > 3000) {
            // Require heli to spin up slowly if it's been more than 3 seconds since we last had a headspeed below 1000 rpm
            spooledUp = 0;

        } else if (!governorSetpoint && throttle <= lastSpoolThrottle) {
            // Governor is disabled, running on throttle % only.
            // If user spools up above 1000rpm, then lowers throttle below the last spool target, allow the heli to be considered spooled up
            spooledUp = 1;
            lastSpoolThrottle = throttle;        // Allow spool target to reduce with throttle freely even if already spooledUp.
            // HF3D TODO:  There's a bug with this block....
            //   If you spool up above 1000 and then come back below that throttle setting (normal mode)
            //   lastSpoolThrottle will ONLY go down since that's all it's allowed to do here.
            //   Then when the governor turns on it gets this really low lastSpoolThrottle value (maybe even zero)
            //      and then throttle drops to zero and the i-term has to wind up the entire amount.
            
        } else if (!spooledUp && governorSetpoint && (headspeed > governorSetpoint*0.97)) {
            // Governor is enabled, running on headspeed
            // If headspeed is within 3% of governorSetpoint, consider the heli to be spooled up
            spooledUp = 1;
            // Set the governor's base throttle % to our last spooled throttle value
            govBaseThrottle = lastSpoolThrottle;
            // Jump the rate limited Setpoint up to the setpoint.
            governorSetpointLimited = governorSetpoint*0.97f;
        
        } else if (!spooledUp && governorSetpoint && (lastSpoolThrottle > 0.90f)) {
            // Governor is enabled, and we've hit 90% throttle trying to get within 97% the requested headspeed.
            // HF3D TODO:  Flag and alert user in the logs and/or with beep tones after flight that govMaxHeadspeed is set too high.
            spooledUp = 1;
            govBaseThrottle = lastSpoolThrottle;
            governorSetpointLimited = headspeed;
        }

        // Handle ramping of throttle
        // HF3D TODO:  Eventually add a "govEnabled" user setting flag.  If the user doesn't have the governor enabled then we need to not use headspeed to perform our spooledUp check.
        //   Right now we have governor enabled all of of the time, but eventually the code needs to support running with or without an RPM signal (gov or no gov)
        // Skip spooling logic if no throttle signal or if we're disarmed
        if ((throttle == 0.0f) || !ARMING_FLAG(ARMED)) {
            // Don't reset spooledUp flag because we want throttle to respond quickly if user drops throttle in normal mode or re-arms while blades are still spinning > 1000rpm
            //   spooledUp flag will reset anyway if RPM < 1000
            lastSpoolThrottle = 0.0f;         // Require re-spooling to start from zero if RPM<1000 and throttle=0 or disarmed
        
        // If not spooled up and throttle is higher than our last spooled-up output, spool some more
        } else if (!governorSetpoint && !spooledUp && (lastSpoolThrottle < throttle)) {
            // Governor is disabled, running on throttle % only.
            throttle = lastSpoolThrottle + rampRate;    // rampRate defined in mixerConfigureOutput up above
            lastSpoolThrottle = throttle;

        // If not spooled up and headspeed is lower than our governorSetpoint, spool some more
        } else if (governorSetpoint && !spooledUp && (headspeed < governorSetpoint)) {
            // Governor is enabled, running on headspeed.
            throttle = lastSpoolThrottle + rampRate;    // rampRate defined in mixerConfigureOutput up above
            lastSpoolThrottle = throttle;
        
        }
        // --------------- End of Spoolup Logic --------------
        
        // --------------- Feedforward Calculations ----------
        
        // Quick and dirty collective pitch linear feed-forward for the main motor
        // Calculate linear feedforward vs. collective stick position (always positive adder)
        //   Reasonable value would be 0.15 throttle addition for 12-degree collective throw..
        //   So gains in the 0.0015 - 0.0032 range depending on where max collective pitch is on the heli
        //   HF3D TODO:  Set this up so works off of a calibrated pitch value for the heli taken during setup
        govCollectiveFF = govColKf * pidGetCollectiveStickPercent();
        
        // Collective pitch impulse feed-forward for the main motor
        govCollectivePulseFF = govColPulseKf * pidGetCollectiveStickHPF();

        // HF3D TODO:  Add a cyclic stick feedforward to the governor - linear gain should be fine.
        // Additional torque is required from the motor when adding cyclic pitch, just like collective (although less)
        // Maybe use this?:  float transition = feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * feedForwardTransition) : 1;
        
        // --------------- End of Feedforward Calculations ---

        // --------------- Governor Logic --------------------
        if (spooledUp && governorSetpointLimited) {
                
            // Calculate error as a percentage of the max headspeed, since 100% throttle should be close to max headspeed
            // HF3D TODO:  Do we really want the governor to respond the same even if setpoint is only 60% of max?
            //   100 rpm error on 60% of max would "feel" a lot different than 100 rpm error on 90% of max headspeed.
            //   But would it really require any torque differences for the same response??  Maybe, since less inertia in head?
            float govError = (governorSetpointLimited - headspeed) / (float)govMaxHeadspeed;
            
            // if gov_p_gain = 10 (govKp = 1), we will get 1% change in throttle for 1% error in headspeed
            float govP = govKp * govError;
            // if gov_i_gain = 10 (govKi = 1), we will get 1% change in throttle for 1% error in headspeed after 1 second
            govI = constrainf(govI + govKi * govError * pidGetDT(), -50.0f, 50.0f);
            govPidSum = govP + govI;
            // float govPidSum = govP + govI;
            
            // HF3D TODO:  Scale the sums based on the average battery voltage?
            //  Note:  This should NOT apply to the tail feedforward compensations that go into the PID controller!
            //         Those compensations are related to the amount of TORQUE only... and this comp would be trying
            //            to keep torque equal, so those shouldn't have to change.
            
            // Generate our new governed throttle signal
            throttle = govBaseThrottle + govCollectiveFF + govCollectivePulseFF + govPidSum;
            // Reset any wind-up due to excess control signal
            if (throttle > 1.0f) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                if (govError > 0.0f) {
                    govI = govI - govKi * govError * pidGetDT();
                }
                throttle = 1.0f;
           
            } else if (throttle < 0.0f) {
                // Remove last addition to I-term to prevent further wind-up if it was moving us towards this over-control
                // HF3D TODO:  What if I-term was at contraints before we did this?
                if (govError < 0.0f) {
                    govI = govI - govKi * govError * pidGetDT();
                }

                throttle = 0.0f;
            }

            // pidGetdT() = targetPidLooptime * 1e-6f;  // 0.00125 for 8kHz
            // Calculate similar to pt1FilterGain with cutoff frequency of 0.05Hz (20s)
            //   RC = 1 / ( 2 * M_PI_FLOAT * f_cut);  ==> RC = 3.183
            //   k = dT / (RC + dT);                  ==>  k = 0.0000393
            // Slowly adapt our govBaseThrottle over time (LPF) as a fallback in case we lose RPM data.
            // HF3D TODO:  If always flying at high collective pitch, that govCollectiveFF will end up adding into the govBaseThrottle
            //    This means the base throttle will be ramped way up if then go back towards a lower average pitch... is that a problem?
            float govBaseThrottleChange = (throttle - govBaseThrottle) * govBaseThrottleFilterGain;
            govBaseThrottle += govBaseThrottleChange;
            // Adjust the I-term to account for the base throttle adjustment we just made
            // HF3D TODO:  What if I-term was at contraints before we did this?
            govI -= govBaseThrottleChange;
        
            // Set lastSpoolThrottle to track the governor throttle signal when the governor is active
            lastSpoolThrottle = throttle;

            DEBUG_SET(DEBUG_SMARTAUDIO, 2, govPidSum*1000.0f);             // Max pidsum will be around 1, so increase by 1000x
        }
        
        mainMotorThrottle = throttle;        // Used by the tail motor code to set the base tail motor output as a fraction of main motor output

        // HF3D TODO:  Rename debug_smartaudio entry eventually
        DEBUG_SET(DEBUG_SMARTAUDIO, 0, governorSetpointLimited);
        DEBUG_SET(DEBUG_SMARTAUDIO, 1, headspeed);
        //DEBUG_SET(DEBUG_SMARTAUDIO, 2, govPidSum*1000.0f);             // Max pidsum will be around 1, so increase by 1000x
        DEBUG_SET(DEBUG_SMARTAUDIO, 3, rpmGetFilteredMotorRPM(1));  // Tail motor RPM

        // HF3D:  Modified original code to ignore any idle offset value when scaling main motor output -- we should always ensure that the main motor will be 100% stopped at zero throttle.
        //   motorOutputMin = motorRangeMin = motorOutputLow = DSHOT_MIN_THROTTLE
#ifdef USE_DSHOT
        float motorOutput = DSHOT_MIN_THROTTLE + (motorOutputHigh - DSHOT_MIN_THROTTLE) * throttle;
#else
        //   for analog PWM, change motorOutputMin to disarmMotorOutput to be safe
        float motorOutput = disarmMotorOutput + (motorOutputHigh - disarmMotorOutput) * throttle;
#endif

        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < DSHOT_MIN_THROTTLE) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);        // motorRangeMax = motorOutputHigh   for uni-directional motor rotation
        } else {
            // HF3D:  Prevent main motor from running or twitching when dshot_idle_value is used.
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                // Use DSHOT_MIN_THROTTLE to allow the main motor to come to a complete stop at any time
                // Also prevents the main motor from "twitching" if the dshot_idle_value is setup to prevent the tail motor from stopping
                motorOutput = constrain(motorOutput, DSHOT_MIN_THROTTLE, motorRangeMax);
            } else
#endif
            {
                // HF3D:  For analog PWM control of main motor ESC, motorRangeMin includes motorConfig()->minthrottle parameter.  
                //   This is NOT good.  Any idle minthrottle setting will cause the ESC to twitch when at zero throttle.  Set to disarmMotorOutput for now just to be safe.
                motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
            }
        }
        motor[0] = motorOutput;
        
    } // end of Main Motor handling (motor[0])

    // Handle the TAIL motor mixing & control (motor[1])
    // HF3D TODO:  Eventually need to support motor driven + variable pitch combination tails
    if (motorCount > 1) {

        // motorMix for tail motor should be 100% stabilized yaw channel
        float motorOutput = motorOutputMixSign * motorMix[1];
        
        // HF3D TODO:  Yaw base thrust is now a setting in pid.c
        //  For a tail motor.. we don't really want it spinning like crazy anytime we're armed,
        //   so tone the motorOutput down a bit using the mainMotorThrottle as a gain until 
        //   we're at half our throttle setting or something.
        
        
        // Linearize the tail motor thrust  (pidApplyThrustLinearization)
#ifdef USE_THRUST_LINEARIZATION
        // Scale PID sums and throttle to linearize the system (thrust varies with rpm^2)
        //   https://github.com/betaflight/betaflight/pull/7304
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif



        // Add in base tail motor thrust to compensate for the main shaft torque
        // Base thrust should vary with main motor RPM^2, but our tail motor also has thrust^2, so increase in base thrust will be linear
        //   Divider should be the maximum headspeed the heli can achieve
        // HF3D TODO:  Add divider RPM to the user configuration.... or at least just use the "max governed RPM" value that will be used for a governor.
        float tailMotorBaseThrustGain = activeMixer[1].throttle;            // HF3D TODO:  Move configuration value to a new configuration parameter
        float tailMotorBaseThrust = 0.0f;
        if (mainMotorThrottle < 0.4f) {
            tailMotorBaseThrust = (mainMotorThrottle * tailMotorBaseThrustGain);   // Track the main motor output while spooling up (looks cool for them to both spool up at once)
        } else {
            tailMotorBaseThrust = (0.4f * tailMotorBaseThrustGain);                // Set fixed base thrust since lower headspeeds actually require higher base thrust.  Not sure what to think about doing here.
            //  Base thrust probably needs to be "load" based.... so feed-forward that takes into account the blade pitch, rpms, etc.
            //  Even then, the relative velocity of the heli through the air will change the loading at the same blade pitch.
            //  Maybe if we had instantaneous motor power telemetry available (watts) we could calculate instantaneous motor torque from watts and RPMs.
            //    This would give us a nearly perfect estimator of how much tail thrust is needed to counteract the motor shaft torque?
            //    Which would make one heck of a cool feedforward compensator...
        }
        
        // HF3D:  Quick and dirty collective pitch pre-compensation for the tail motor
        // HF3D TODO:  Just have this piggy-back on the feedforward values going into the governor.
        // Only perform collectivePrecompensation on the tail if the main motor is running
        if (mainMotorThrottle > 0.0f) {
            // Zero collectiveStickPercent is zero collective pitch, so very little main shaft torque
            //    0x normal tail motor base thrust
            // Hover collectiveStickPercent is a few degrees of collective pitch, so 1x multiplier
            //    1x normal tail motor base thrust at hover collective
            // 100% collectiveStickPercent is very high collective pitch, so LOTS of thrust and corresponding main shaft torque
            //    2x normal tail motor base thrust
            float collectivePrecomp = 0.02321083f + 0.05059961f*pidGetCollectiveStickPercent() - 0.0002088975f*pidGetCollectiveStickPercent()*pidGetCollectiveStickPercent();
            //float collectivePrecomp = collectiveStickPercent / 100.0f * 3.0f;
            // Precomp is linear since collective changes give a linear change in thrust, but tail motor thrust is exponential due to rpm response
            //   Linearize the collectivePrecomp value for the tail motor commands
            collectivePrecomp = pidApplyThrustLinearization(collectivePrecomp);
            tailMotorBaseThrust *= collectivePrecomp;
        }
        
        motorOutput += tailMotorBaseThrust;
        
        //motorOutput += (mainMotorThrottle * tailMotorBaseThrustGain);       // Probably something like 0.2 would be a good setting for this?  Just a guess.
        //  ^^ Actually, I think this is a bad idea.  Base tail thrust will actually need to INCREASE for lower headspeeds.  Lower mainshaft rpm with some power input = more torque necessary.
        //motorOutput += (mainMotorRPM/6000.0f)*tailMotorBaseThrustGain;    // Just guessing that 20% thrust on tail will be about right for 100% rpm on head.  Nevermind, see above.  This is wrong.

        // scale tail motor output to full motor output range, including impact of any idle offset.  
        // Note that motorOutput here can still be < 0 if the motorMix is sufficiently negative.  Idle offset will be taken into account again down below.
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
            // HF3D:  Only use dshot_idle_value when main motor is spinning.
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                if (mainMotorThrottle > 0.0) {
                    // Use dshot_idle_value to prevent tail from stopping when main motor is running
                    // motorRangeMin = DSHOT_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
                    motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
                } else {
                    // Use DSHOT_MIN_THROTTLE to allow the tail to come to a complete stop when main motor isn't running
                    motorOutput = constrain(motorOutput, DSHOT_MIN_THROTTLE, motorRangeMax);
                }
            } else
#endif
            {
                // Handle analog PWM tail motor ESC.  Not sure why anyone would do that... but, okay.
                if (mainMotorThrottle > 0.0) {
                    // Use minThrottle value to prevent tail from stopping when main motor is running
                    motorOutput = constrain(motorOutput, motorRangeMin, motorRangeMax);
                } else {
                    motorOutput = constrain(motorOutput, disarmMotorOutput, motorRangeMax);
                }
            }
        }
        motor[1] = motorOutput;     // Set final tail motor output

    }  // end of tail motor handling
     
    // HF3D does not support more than 1 main and 1 tail motor.  Turn any additional motors off.
    for (int i = 2; i < motorCount; i++) {
        motor[i] = disarmMotorOutput;
    }

    // Disarmed mode check
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < motorCount; i++) {
        motor[i] = disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static int dynLpfPreviousQuantizedThrottle = -1;  // to allow an initial zero throttle to set the filter cutoff

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = lrintf(throttle * DYN_LPF_THROTTLE_STEPS); // quantize the throttle reduce the number of filter updates
        if (quantizedThrottle != dynLpfPreviousQuantizedThrottle) {
            // scale the quantized value back to the throttle range so the filter cutoff steps are repeatable
            const float dynLpfThrottle = (float)quantizedThrottle / DYN_LPF_THROTTLE_STEPS;
            dynLpfGyroUpdate(dynLpfThrottle);
            dynLpfDTermUpdate(dynLpfThrottle);
            dynLpfPreviousQuantizedThrottle = quantizedThrottle;
            lastDynLpfUpdateUs = currentTimeUs;
        }
    }
}
#endif

// mixTable is called from the main loop calculate the motor and servo mixing
FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs, uint8_t vbatPidCompensation)
{
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    // if (isFlipOverAfterCrashActive()) {
        // applyFlipOverAfterCrashModeToMotors();

        // return;
    // }

    motorMixer_t * activeMixer = &currentMixer[0];
    
    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    // PID_MIXER_SCALING = 1000.0f
    // Default yawPidSumLimit = 400
    //  So scaledAxisPidYaw maxxes out at +/- 0.4 if yawPidSumLimit is left at 400... probably not good for a motor driven tail
    //  because the motor output is actually +/- 1.0f, but values below 0 won't actually do anything since the tail motor will be stopped.
    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Calculate voltage compensation
    const float vbatCompensationFactor = vbatPidCompensation ? calculateVbatPidCompensation() : 1.0f;

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    const bool airmodeEnabled = airmodeIsEnabled();

#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeEnabled) {
        throttle = 0.5f;   // 
    }
#endif // USE_YAW_SPIN_RECOVERY

    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    for (int i = 0; i < motorCount; i++) {

        float mix =
            scaledAxisPidRoll  * activeMixer[i].roll +
            scaledAxisPidPitch * activeMixer[i].pitch +
            scaledAxisPidYaw   * activeMixer[i].yaw;

        // HF3D TODO:  VBatt compensation might actually be valid for a motor driven tail?
        //   Other option is to use a governor to drive the tail motor to an exact RPM target instead of using a throttle setpoint
        mix *= vbatCompensationFactor;  // Add voltage compensation

        motorMix[i] = mix;
    }

    pidUpdateAntiGravityThrottleFilter(throttle);

#ifdef USE_DYN_LPF
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

// HF3D:  Re-used throttleBoost settings for flat piro compensation
// #if defined(USE_THROTTLE_BOOST)
    // if (throttleBoost > 0.0f) {
        // const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        // throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    // }
// #endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    mixerThrottle = throttle;

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !featureIsEnabled(FEATURE_3D)
        && !airmodeEnabled
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // Stop all motors by setting them to disarmMotorOutput value.
        // HF3D:  It's very important that this be done so that tail motor will stop as well.  Tail motor may still have yaw Pidsum mix even if throttle = 0!
        applyMotorStop();
        // HF3D TODO:  Call governor function to tell it we are in a stopped state
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix, activeMixer);
    }
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}

mixerMode_e getMixerMode(void)
{
    return currentMixerMode;
}


bool isFixedWing(void)
{
    switch (currentMixerMode) {
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_CUSTOM_AIRPLANE:
        return true;

        break;
    default:
        return false;

        break;
    }
}

// HF3D TODO:  Move this to governor/spoolup source file eventually
// Return the status of whether the heli is spooled up
// Very critical that this status is correct, because core.c checks it to force 
//     the pid controller to reset it's I term on each pass if this is set.
uint8_t isHeliSpooledUp(void)
{
    // Spooled Up is reset to 0 if (mainMotorRPM < 1000.0f)
    // spooledUp will set to "1" the first time if it's a "clean" spoolup where you arm and spool to a single throttle setpoint
    //   Jacking around with the throttle during spoolup could cause it to set the spooledUp flag instantly if you're already above 1000rpm
    //   If user spools up above 1000rpm, then lowers throttle below the last spool target reached, the heli will be considered spooled up
    return spooledUp;
}

float mixerGetGovGearRatio(void)
{
    return govGearRatio;
}

float mixerGetGovCollectivePulseFilterGain(void)
{
    return govColPulseFilterGain;
}