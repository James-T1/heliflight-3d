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

#ifdef USE_PWM_OUTPUT

#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

FAST_RAM_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    // HAL driver used for timers on F7 and H7 mcus
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) return;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(Handle, &TIM_OCInitStructure, channel);
#else
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
#endif
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
#if defined(USE_HAL_DRIVER)
    TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
    if (Handle == NULL) return;
#endif

    // Configure the time-base unit for this timer.  The TBU is shared by all channels on this timer.
    //  This means that all output channels on this timer are sharing a common PWM output frequency.
    configTimeBase(timerHardware->tim, period, hz);

    // Configure and initialize this timer output compare channel for PWM and enable preload.
    pwmOCConfig(timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
        );

    // Enable the PWM output on this Timer
#if defined(USE_HAL_DRIVER)
    // F7 and H7 devices use HAL driver for access to some peripherals like timers - see MCU_COMMON_SRC differences in the mcu makefiles
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL)
        HAL_TIMEx_PWMN_Start(Handle, timerHardware->channel);
    else
        HAL_TIM_PWM_Start(Handle, timerHardware->channel);
    HAL_TIM_Base_Start(Handle);
#else
    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);
#endif

    // save the address of this channel's condition code register so we can update it later
    // CCR can now be updated with our desired duty cycle percentage on the output
    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    // As long as the channel is configured in output mode, the content of the TIMx_CCRy channel register is compared to the content of the timer counter.  This drives the channel output.
    //   https://www.st.com/resource/en/application_note/dm00236305-generalpurpose-timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf
    // Set CCR value to zero to get 0% duty cycle on the PWM output we just configured
    *channel->ccr = 0;
}

static FAST_RAM_ZERO_INIT motorDevice_t motorPwmDevice;

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    *motors[index].channel.ccr = lrintf((value * motors[index].pulseScale) + motors[index].pulseOffset);
}

void pwmShutdownPulsesForAllMotors(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors();
}

static motorVTable_t motorPwmVTable;
bool pwmEnableMotors(void)
{
    /* check motors can be enabled */
    return (motorPwmVTable.write != &pwmWriteUnused);
}

bool pwmIsMotorEnabled(uint8_t index)
{
    return motors[index].enabled;
}

static void pwmCompleteOneshotMotorUpdate(void)
{
    for (int index = 0; index < motorPwmDevice.count; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

static float pwmConvertFromExternal(uint16_t externalValue)
{
    return (float)externalValue;
}

static uint16_t pwmConvertToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

static motorVTable_t motorPwmVTable = {
    .postInit = motorPostInitNull,
    .enable = pwmEnableMotors,
    .disable = pwmDisableMotors,
    .isMotorEnabled = pwmIsMotorEnabled,
    .shutdown = pwmShutdownPulsesForAllMotors,
    .convertExternalToMotor = pwmConvertFromExternal,
    .convertMotorToExternal = pwmConvertToExternal,
};

motorDevice_t *motorPwmDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    motorPwmDevice.vTable = motorPwmVTable;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol) {
    default:
    case PWM_TYPE_ONESHOT125:
        sMin = 125e-6f;
        sLen = 125e-6f;
        break;
    case PWM_TYPE_ONESHOT42:
        sMin = 42e-6f;
        sLen = 42e-6f;
        break;
    case PWM_TYPE_MULTISHOT:
        sMin = 5e-6f;
        sLen = 20e-6f;
        break;
    case PWM_TYPE_BRUSHED:
        sMin = 0;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    case PWM_TYPE_STANDARD:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    }

    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.updateStart = motorUpdateStartNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm ? motorUpdateCompleteNull : pwmCompleteOneshotMotorUpdate;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

#if defined(STM32F1)
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);
#endif

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = ((motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, motorConfig->motorPwmInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    return &motorPwmDevice;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

#ifdef USE_SERVOS
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].channel.ccr) {
        *servos[index].channel.ccr = lrintf(value);
    }
}

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag) {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        const timerHardware_t *timer = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        if (timer == NULL) {
            // TODO:  flag failure and disable ability to arm?
            break;
        }

#if defined(STM32F1)
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(servos[servoIndex].io, IOCFG_AF_PP, timer->alternateFunction);
#endif

        // HF3D:  Initialize with zero output to support servos with different center pulse widths (removed servoConfig->servoCenterPulse from next to last parameter)
        pwmOutConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / servoConfig->servoPwmRate, 0, 0);
        servos[servoIndex].enabled = true;
    }
}
#endif // USE_SERVOS

// Castle Link Live protocol
timerCCHandlerRec_t edgeCb;
timerOvrHandlerRec_t updateCb;

// Note:  Since CCR is 16-bit, we're limited to 65535uS, or 16Hz as our minimum PWM frequency.
static volatile uint16_t ccrForLastPulse = 0;               // Set this to the CCR used when a new PWM pulse is generated
static volatile uint16_t firstValidTelemetryTime = 0;       // Reset this to zero each time a PWM pulse is generated
static volatile bool ccDataValid = false;                   // Let's us flag if a telemetry data set had any invalid timing pulses
static volatile uint8_t ccTelemIndex = 0;                   // Keeps track of which telemetry value we are expecting next from the ESC
static volatile uint16_t rawTelemetryTimes[11];             // Store the raw telemetry times (may include invalid data: check ccDataValid flag)
const uint16_t ccMaxPulseLengths[11] = { 1000, 5500, 5500, 5500, 3000, 4500, 5500, 5500, 5500, 5500, 4500 };
volatile uint16_t ccTelemetryTimes[12];                     // Last array value is the validity field (0=invalid, 1=valid)

// Process any falling edge on the castle ESC signal wire
//   First falling edge will be start of PWM output from FC, second pulse should be telemetry data from the ESC
static void castleFallingEdgeInputCallback(timerCCHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    // Exit if we've already stored a "valid" telemetry edge time, if there's no valid PWM output, or if this edge is way too early to be telemetry data.
    if (firstValidTelemetryTime || !ccrForLastPulse || capture < 1000 || capture < ccrForLastPulse+300) {
        return;
    }
    
    // Do a quick sanity check to ensure this edge is within window of 300uS to 5700uS of the end of our last PWM pulse before storing it
    const uint16_t telemetryTime = lrintf((capture - ccrForLastPulse) / motors[0].pulseScale);
    if (telemetryTime > 300 && telemetryTime < 5700) {
        firstValidTelemetryTime = telemetryTime;
    }
}

// Do some data processing at the beginning of each PWM pulse that we create
static void castleUpdateCallback(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    // capture just stores the value of the ARR register
    UNUSED(capture);
    
    // Check to see if we have a valid telemetry value
    if (firstValidTelemetryTime) {
        // Validate each telemetry pulse is within a window of +/- 200uS of the expected range, otherwise throw the data out.
        // First pulse should be 1ms +/- the timing jitter
        if (ccTelemIndex == 0 && firstValidTelemetryTime>800 && firstValidTelemetryTime<1200) {
            rawTelemetryTimes[ccTelemIndex] = firstValidTelemetryTime;
        // The other pulses should be 0.5ms minimum to their maximum pulse time per the Castle Link Live specification
        } else if (ccTelemIndex>0 && ccTelemIndex<11 && firstValidTelemetryTime>300 && firstValidTelemetryTime<(ccMaxPulseLengths[ccTelemIndex]+200)) {
            rawTelemetryTimes[ccTelemIndex] = firstValidTelemetryTime;
        // If a non-zero pulse time does not meet the specification then ignore it and set our telemetry data as invalid.
        } else {
            ccDataValid = false;
        }
        
        ccTelemIndex++;
        
    } else {
        // No pulse was detected during the last PWM period.  Check if ccTelemIndex = 11 and the ccDataValid flag to see if we received a full set of valid telemetry data.
        // If data is all valid, then set the "valid" flag in the ccTelemetryData array.  This flag allows for keeping state between this and the esc_sensor code.
        //    Then when we read it from the esc_Sensor code we can just set the invalid flag once we're done with it...
        //      ^^^ This!!  Then we'll read it once when it's valid, set the flag to invalid, and wait until we see a "valid" flag set again before we read the data.
        //      And if we just see "invalid" over and over for too long (1 second or whatever), then we'll consider all of the data invalid.
        if (ccTelemIndex == 11 && ccDataValid) {
            // Copy raw telemetry times array to the castle ESC telemetry times array and set the validity flag to true.
            for (int i=0; i<= 10; i++) {
                ccTelemetryTimes[i] = rawTelemetryTimes[i];
            }
            // Set final value of array (validity flag) to true.
            ccTelemetryTimes[11] = 1;
        }
        
        // Reset our telemetry position index and data validity since no pulse was detected
        ccTelemIndex = 0;
        ccDataValid = true;
    }

    // Reset the firstValidTelemetry time after each read
    firstValidTelemetryTime = 0;
    
    // Store the initial CCR value so that the input callback knows the exact time of the PWM pulse end even if the CCR preload value is changed mid-pulse
    ccrForLastPulse = *motors[0].channel.ccr;
}

motorDevice_t *motorPwmCastleLLDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm)
{
    motorPwmDevice.vTable = motorPwmVTable;

    //Castle is standard PWM and only case PWM_TYPE_STANDARD:
    float sMin = 1e-3f;
    float sLen = 1e-3f;
    useUnsyncedPwm = true;
    idlePulse = 0;

    motorPwmDevice.vTable.write = pwmWriteStandard;
    motorPwmDevice.vTable.updateStart = motorUpdateStartNull;
    motorPwmDevice.vTable.updateComplete = useUnsyncedPwm ? motorUpdateCompleteNull : pwmCompleteOneshotMotorUpdate;

    // Configure only Motor 1 for Castle Link Live Telemetry
    if (0 < motorCount) {
        int motorIndex = 0;
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

#if defined(STM32F1)
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_OD);
#else
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_OD, timerHardware->alternateFunction);
#endif

        /* standard PWM output */
        const unsigned pwmRateHz = motorConfig->motorPwmRate;

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = hz / pwmRateHz;

        motors[motorIndex].pulseScale = (sLen * hz) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);
        
        // Castle ESC in LinkLive mode requires inverted PWM (low = active part of PWM pulse)
        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, true);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
        
        // Setup the callbacks and enable IRQs for PWM generator and input capture on indirect channel
        // Use the timerHardware->channel value to get the paired adjacent channel (channels 1/2 and 3/4).
        //   0x00 is channel 1, 0x10 is channel 3
        //   0x01 is channel 2, 0x11 is channel 4
        //   So the "adjacent channel" is always timerHardware->channel XOR 0x01  (channel 2)
        timerNVICConfigure(timerInputIrq(timerHardware->tim));
        timerChCCHandlerInit(&edgeCb, castleFallingEdgeInputCallback);
        timerChOvrHandlerInit(&updateCb, castleUpdateCallback);
        castleTimerChConfigCallbacks(timerHardware, &edgeCb, &updateCb);

        // Initialize the indirect input capture on the adjacent timer channel
#if defined(USE_HAL_DRIVER)
        TIM_HandleTypeDef* Handle = timerFindTimerHandle(timerHardware->tim);
        if (Handle == NULL) return NULL;

        TIM_IC_InitTypeDef TIM_ICInitStructure;
        memset(&TIM_ICInitStructure, 0, sizeof(TIM_ICInitStructure));
        TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_FALLING;
        TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_INDIRECTTI;
        TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(Handle, &TIM_ICInitStructure, timerHardware->channel ^ TIM_CHANNEL_2);
        HAL_TIM_IC_Start_IT(Handle, timerHardware->channel);
#else
        TIM_ICInitTypeDef TIM_ICInitStructure;
        TIM_ICStructInit(&TIM_ICInitStructure);
        TIM_ICInitStructure.TIM_Channel = timerHardware->channel ^ TIM_Channel_2;   // get opposite channel no
        TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
        TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
        TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
        TIM_ICInitStructure.TIM_ICFilter = 0;
        TIM_ICInit(timerHardware->tim, &TIM_ICInitStructure);
#endif

    }
    
    // Initialize the rest of the motors as they would be in pwm_output.c (normal polarity and push/pull configuration with no Castle telemetry)
    for (int motorIndex = 1; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerAllocate(tag, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            motorPwmDevice.vTable.write = &pwmWriteUnused;
            motorPwmDevice.vTable.updateComplete = motorUpdateCompleteNull;
            /* TODO: block arming and add reason system cannot arm */
            return NULL;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

#if defined(STM32F1)
        IOConfigGPIO(motors[motorIndex].io, IOCFG_AF_PP);
#else
        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);
#endif

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex].pulseScale = ((motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, motorConfig->motorPwmInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    return &motorPwmDevice;
}


#endif // USE_PWM_OUTPUT
