/*
   Copyright (C) 2017 Raoul Rubien (github.com/rubienr)

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the program; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
 */

#include "./hal.h"

// system includes
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <string.h>
#include <stdarg.h>

// 3rd party includes

// local library includes
#include "./pendant.h"

using std::endl;

namespace XhcWhb04b6 {

// ----------------------------------------------------------------------

WhbHalMemory::WhbHalMemory() :
    in(),
    out()
{
}

// ----------------------------------------------------------------------

WhbHalMemory::~WhbHalMemory()
{
}

// ----------------------------------------------------------------------


WhbHalMemory::In::In() :
    feedOverrideValue(nullptr),
    spindleIsOn(nullptr),
    spindleOverrideValue(nullptr),
    spindleRps(nullptr),
    jogMaxVelocity(nullptr),
    stepsizeUp(nullptr),
    isProgramRunning(nullptr),
    isProgramPaused(nullptr),
    isProgramIdle(nullptr),
    isModeAuto(nullptr),
    isModeJoint(nullptr),
    isModeManual(nullptr),
    isModeMdi(nullptr),
    isEmergencyStop(nullptr),
    isMachineOn(nullptr)
{
}

// ----------------------------------------------------------------------

WhbHalMemory::Out::Out() :
    button_pin{0},
    jogCount(nullptr),
    jogCountNeg(nullptr),
    jogVelocity(nullptr),
    feedOverrideDecrease(nullptr),
    feedOverrideIncrease(nullptr),
    spindleDoDecrease(nullptr),
    spindleDoIncrease(nullptr),
    spindleOverrideDoDecrease(nullptr),
    spindleOverrideDoIncrease(nullptr),
    jogSpeedValue(nullptr),
    homeAll(nullptr),
    jogIncrement(nullptr),
    jogIncrementPlus(nullptr),
    jogIncrementMinus(nullptr),
    jogPlus(nullptr),
    jogMinus(nullptr),
    jointXSelect(nullptr),
    jointYSelect(nullptr),
    jointZSelect(nullptr),
    jointASelect(nullptr),
    jointBSelect(nullptr),
    jointCSelect(nullptr),
    stepsize(nullptr),
    isPendantSleeping(nullptr),
    isPendantConnected(nullptr),
    isPendantRequired(nullptr),
    doRunProgram(nullptr),
    doPauseProgram(nullptr),
    doResumeProgram(nullptr),
    doStopProgram(nullptr),
    doModeAuto(nullptr),
    doModeJoint(nullptr),
    doModeManual(nullptr),
    doModeMdi(nullptr),
    doEmergencyStop(nullptr),
    resetEmergencyStop(nullptr)
{
}

// ----------------------------------------------------------------------

void WhbHal::freeSimulatedPin(void** pin)
{
    if (*pin != nullptr)
    {
        free(*pin);
        pin = nullptr;
    }
}

// ----------------------------------------------------------------------

WhbHal::WhbHal() :
    memory(nullptr),
    mButtonNameToIdx(),
    velocityComputation(),
    mIsSimulationMode(true),
    mName("xhc-whb04b-6"),
    mComponentPrefix("whb"),
    mHalCompId(-1),
    mDevNull(nullptr),
    mHalCout(&mDevNull),
    mStepMode(HandwheelStepmodes::Mode::STEP)
{
}

// ----------------------------------------------------------------------

WhbHal::~WhbHal()
{
    if (!mIsSimulationMode)
    {
        memory->~WhbHalMemory();
        // documentation tells us to not free hal pins
        return;
    }

    if (memory == nullptr)
    {
        return;
    }

    freeSimulatedPin((void**)(&memory->in.feedOverrideValue));
    freeSimulatedPin((void**)(&memory->in.spindleIsOn));
    freeSimulatedPin((void**)(&memory->in.spindleOverrideValue));
    freeSimulatedPin((void**)(&memory->in.spindleRps));
    for (size_t idx = 0; idx < (sizeof(memory->out.button_pin) / sizeof(hal_bit_t *)); idx++)
    {
        freeSimulatedPin((void**)(&memory->out.button_pin[idx]));
    }
    freeSimulatedPin((void**)(&memory->out.jogCount));
    freeSimulatedPin((void**)(&memory->out.jogCountNeg));
    freeSimulatedPin((void**)(&memory->out.jogVelocity));
    freeSimulatedPin((void**)(&memory->out.feedOverrideDecrease));
    freeSimulatedPin((void**)(&memory->out.feedOverrideIncrease));
    freeSimulatedPin((void**)(&memory->out.spindleStart));
    freeSimulatedPin((void**)(&memory->out.spindleStop));
    freeSimulatedPin((void**)(&memory->out.spindleDoDecrease));
    freeSimulatedPin((void**)(&memory->out.spindleDoIncrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideDoDecrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideDoIncrease));
    freeSimulatedPin((void**)(&memory->in.jogMaxVelocity));
    freeSimulatedPin((void**)(&memory->out.jogIncrement));
    freeSimulatedPin((void**)(&memory->out.jogIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogPlus));
    freeSimulatedPin((void**)(&memory->out.jogMinus));
    freeSimulatedPin((void**)(&memory->out.jogSpeedValue));
    freeSimulatedPin((void**)(&memory->out.homeAll));
    freeSimulatedPin((void**)(&memory->in.stepsizeUp));
    freeSimulatedPin((void**)(&memory->out.jointXSelect));
    freeSimulatedPin((void**)(&memory->out.jointYSelect));
    freeSimulatedPin((void**)(&memory->out.jointZSelect));
    freeSimulatedPin((void**)(&memory->out.jointASelect));
    freeSimulatedPin((void**)(&memory->out.jointBSelect));
    freeSimulatedPin((void**)(&memory->out.jointCSelect));
    freeSimulatedPin((void**)(&memory->out.stepsize));
    freeSimulatedPin((void**)(&memory->out.isPendantSleeping));
    freeSimulatedPin((void**)(&memory->out.isPendantConnected));
    freeSimulatedPin((void**)(&memory->out.isPendantRequired));
    freeSimulatedPin((void**)(&memory->out.doRunProgram));
    freeSimulatedPin((void**)(&memory->out.doPauseProgram));
    freeSimulatedPin((void**)(&memory->out.doResumeProgram));
    freeSimulatedPin((void**)(&memory->out.doStopProgram));
    freeSimulatedPin((void**)(&memory->out.doModeAuto));
    freeSimulatedPin((void**)(&memory->out.doModeJoint));
    freeSimulatedPin((void**)(&memory->out.doModeManual));
    freeSimulatedPin((void**)(&memory->out.doModeMdi));
    freeSimulatedPin((void**)(&memory->out.doEmergencyStop));
    freeSimulatedPin((void**)(&memory->out.resetEmergencyStop));
    freeSimulatedPin((void**)(&memory->in.isEmergencyStop));
    freeSimulatedPin((void**)(&memory->in.isMachineOn));
    delete memory;
}

// ----------------------------------------------------------------------

int WhbHal::newSimulatedHalPin(char* pin_name, void** ptr, int s)
{
    *ptr = calloc(s, 1);
    assert(*ptr != nullptr);
    memset(*ptr, 0, s);
    return 0;
}

// ----------------------------------------------------------------------

int WhbHal::newHalFloat(hal_pin_dir_t direction, hal_float_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(ptr != nullptr);
    assert(*ptr == nullptr);
    *mHalCout << "hal   float ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_float_t));
    }
    else
    {
        int r = hal_pin_float_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}

// ----------------------------------------------------------------------

int WhbHal::newHalSigned32(hal_pin_dir_t direction, hal_s32_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(ptr != nullptr);
    assert(*ptr == nullptr);
    *mHalCout << "hal   s32   ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;
    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_s32_t));
    }
    else
    {
        int r = hal_pin_s32_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}

// ----------------------------------------------------------------------

int WhbHal::newHalBit(hal_pin_dir_t direction, hal_bit_t** ptr, int componentId, const char* fmt, ...)
{
    char    pin_name[256];
    va_list args;
    va_start(args, fmt);
    vsprintf(pin_name, fmt, args);
    va_end(args);

    assert(ptr != nullptr);
    assert(*ptr == nullptr);
    *mHalCout << "hal   bit   ";
    if (direction == HAL_OUT)
    {
        *mHalCout << "out ";
    }
    else
    {
        *mHalCout << "in  ";
    }
    *mHalCout << pin_name << endl;

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_bit_t));
    }
    else
    {
        int r = hal_pin_bit_new(pin_name, direction, ptr, componentId);
        assert(r == 0);
        return r;
    }
}

// ----------------------------------------------------------------------

bool WhbHal::isSimulationModeEnabled() const
{
    return mIsSimulationMode;
}

// ----------------------------------------------------------------------

void WhbHal::setSimulationMode(bool isSimulationMode)
{
    this->mIsSimulationMode = isSimulationMode;
}

// ----------------------------------------------------------------------

int WhbHal::getHalComponentId() const
{
    return mHalCompId;
}

// ----------------------------------------------------------------------

const char* WhbHal::getHalComponentName() const
{
    return mName;
}

// ----------------------------------------------------------------------

void WhbHal::init(const WhbSoftwareButton* softwareButtons, const WhbKeyCodes& mKeyCodes)
{
    if (!mIsSimulationMode)
    {
        *mHalCout << "hal   initialize HAL component in HAL mode " << mName << " ... ";
        mHalCompId = hal_init(mName);
        if (mHalCompId <= 0)
        {
            std::cerr << endl << "failed to initialize HAL component " << mName << endl;
            exit(EXIT_FAILURE);
        }
        *mHalCout << "ok" << endl;

        *mHalCout << "hal   initialize shared HAL memory for component id  " << mHalCompId << " ... ";
        memory = reinterpret_cast<WhbHalMemory*>(hal_malloc(sizeof(WhbHalMemory)));
        memory = new(memory) WhbHalMemory();
    }
    else
    {
        *mHalCout << "hal   initialize simulated HAL memory " << " ... ";
        memory = new WhbHalMemory();
    }

    if (memory == nullptr)
    {
        std::cerr << "failed to allocate HAL memory" << endl;
        exit(EXIT_FAILURE);
    }
    *mHalCout << "ok" << endl;

    // register all known xhc-whb04b-6 buttons
    for (size_t idx = 0; !((softwareButtons[idx].key.code == mKeyCodes.buttons.undefined.code) &&
                           (softwareButtons[idx].modifier.code == mKeyCodes.buttons.undefined.code)); idx++)
    {
        const char* buttonName = nullptr;
        if (&softwareButtons[idx].modifier == &mKeyCodes.buttons.undefined)
        {
            buttonName = softwareButtons[idx].key.text;
        }
        else
        {
            buttonName = softwareButtons[idx].key.altText;
        }

        mButtonNameToIdx[std::string(mComponentPrefix) + ".button." + std::string(buttonName)] = idx;
        newHalBit(HAL_OUT, &((memory->out.button_pin)[idx]), mHalCompId, "%s.button.%s", mComponentPrefix,
                  buttonName);
    }

    newHalBit(HAL_OUT, &(memory->out.isPendantSleeping), mHalCompId, "%s.pendant.is-sleeping", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.isPendantConnected), mHalCompId, "%s.pendant.is-connected", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.isPendantRequired), mHalCompId, "%s.pendant.is-required", mComponentPrefix);

    newHalFloat(HAL_IN, &(memory->in.feedOverrideValue), mHalCompId, "%s.halui.feed-override.value", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideDecrease), mHalCompId, "%s.halui.feed-override.decrease", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideIncrease), mHalCompId, "%s.halui.feed-override.increase",
              mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.stepsize), mHalCompId, "%s.stepsize", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.stepsizeUp), mHalCompId, "%s.stepsize-up", mComponentPrefix);

    newHalFloat(HAL_IN, &(memory->in.spindleRps), mHalCompId, "%s.spindle-rps", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.spindleOverrideValue), mHalCompId, "%s.halui.spindle-override.value",
                mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoIncrease), mHalCompId, "%s.halui.spindle.increase",
              mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleDoDecrease), mHalCompId, "%s.halui.spindle.decrease",
              mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideDoIncrease), mHalCompId, "%s.halui.spindle-override.increase",
              mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideDoDecrease), mHalCompId, "%s.halui.spindle-override.decrease",
              mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.spindleIsOn), mHalCompId, "%s.halui.spindle.is-on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleStart), mHalCompId, "%s.halui.spindle.start", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.spindleStop), mHalCompId, "%s.halui.spindle.stop", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.doEmergencyStop), mHalCompId, "%s.halui.estop.activate", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isEmergencyStop), mHalCompId, "%s.halui.estop.is-activated", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.resetEmergencyStop), mHalCompId, "%s.halui.estop.reset", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isMachineOn), mHalCompId, "%s.halui.machine.is-on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doMachineOn), mHalCompId, "%s.halui.machine.on", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doMachineOff), mHalCompId, "%s.halui.machine.off", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isProgramIdle), mHalCompId, "%s.halui.program.is-idle", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isProgramPaused), mHalCompId, "%s.halui.program.is-paused", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isProgramRunning), mHalCompId, "%s.halui.program.is-running", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doResumeProgram), mHalCompId, "%s.halui.program.resume", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doPauseProgram), mHalCompId, "%s.halui.program.pause", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doRunProgram), mHalCompId, "%s.halui.program.run", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doStopProgram), mHalCompId, "%s.halui.program.stop", mComponentPrefix);

    newHalBit(HAL_IN, &(memory->in.isModeAuto), mHalCompId, "%s.halui.mode.is-auto", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeJoint), mHalCompId, "%s.halui.mode.is-joint", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeManual), mHalCompId, "%s.halui.mode.is-manual", mComponentPrefix);
    newHalBit(HAL_IN, &(memory->in.isModeMdi), mHalCompId, "%s.halui.mode.is-mdi", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeAuto), mHalCompId, "%s.halui.mode.auto", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeJoint), mHalCompId, "%s.halui.mode.joint", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeManual), mHalCompId, "%s.halui.mode.manual", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.doModeMdi), mHalCompId, "%s.halui.mode.mdi", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.jointXSelect), mHalCompId, "%s.halui.joint.x.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jointYSelect), mHalCompId, "%s.halui.joint.y.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jointZSelect), mHalCompId, "%s.halui.joint.z.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jointASelect), mHalCompId, "%s.halui.joint.a.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jointBSelect), mHalCompId, "%s.halui.joint.b.select", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jointCSelect), mHalCompId, "%s.halui.joint.c.select", mComponentPrefix);

    newHalFloat(HAL_OUT, &(memory->out.jogSpeedValue), mHalCompId, "%s.halui.jog-speed", mComponentPrefix);

    newHalSigned32(HAL_OUT, &(memory->out.jogCount), mHalCompId, "%s.jog.counts", mComponentPrefix);
    newHalSigned32(HAL_OUT, &(memory->out.jogCountNeg), mHalCompId, "%s.jog.counts-neg", mComponentPrefix);

    newHalFloat(HAL_OUT, &(memory->out.jogVelocity), mHalCompId, "%s.jog.velocity", mComponentPrefix);
    newHalFloat(HAL_IN, &(memory->in.jogMaxVelocity), mHalCompId, "%s.halui.max-velocity.value", mComponentPrefix);
    newHalFloat(HAL_OUT, &(memory->out.jogIncrement), mHalCompId, "%s.halui.jog.selected.increment", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jogIncrementPlus), mHalCompId, "%s.halui.jog.selected.increment-plus", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jogIncrementMinus), mHalCompId, "%s.halui.jog.selected.increment-minus", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jogPlus), mHalCompId, "%s.halui.jog.selected.plus", mComponentPrefix);
    newHalBit(HAL_OUT, &(memory->out.jogMinus), mHalCompId, "%s.halui.jog.selected.minus", mComponentPrefix);

    newHalBit(HAL_OUT, &(memory->out.homeAll), mHalCompId, "%s.halui.home-all", mComponentPrefix);
}

// ----------------------------------------------------------------------

void WhbHal::setEnableVerbose(bool enable)
{
    if (enable)
    {
        mHalCout = &std::cout;
    }
    else
    {
        mHalCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbHal::setNoAxisActive(bool enabled)
{
    *mHalCout << "hal   OFF no axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisXActive(bool enabled)
{
    *memory->out.jointXSelect = enabled;
    *mHalCout << "hal   X axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisYActive(bool enabled)
{
    *memory->out.jointYSelect = enabled;
    *mHalCout << "hal   Y axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisZActive(bool enabled)
{
    *memory->out.jointZSelect = enabled;
    *mHalCout << "hal   Z axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisAActive(bool enabled)
{
    *memory->out.jointASelect = enabled;
    *mHalCout << "hal   A axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisBActive(bool enabled)
{
    *memory->out.jointBSelect = enabled;
    *mHalCout << "hal   B axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setAxisCActive(bool enabled)
{
    *memory->out.jointCSelect = enabled;
    *mHalCout << "hal   C axis active" << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setStepSize(const hal_float_t& stepSize)
{
    if (mStepMode == HandwheelStepmodes::Mode::STEP)
    {
        *memory->out.jogIncrement  = stepSize;
        *memory->out.jogSpeedValue = 0;
    }
    else // CONTINUOUS (speed)
    {
        *memory->out.jogIncrement  = 0;
        *memory->out.jogSpeedValue = stepSize;
    }
    *mHalCout << "hal   step size " << stepSize << endl;
}

// ----------------------------------------------------------------------

void WhbHal::setLead()
{
    std::ios init(NULL);
    init.copyfmt(*mHalCout);
    *mHalCout << "hal   feed rate Lead" << endl;
    mHalCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbHal::setReset(bool enabled)
{
    if (*memory->in.isMachineOn) { // disable machine
        clearStartResumeProgramStates();
        *memory->out.doMachineOff = true;
    }
    else { // enable machine
        *memory->out.doMachineOn = true;
    }

    if (!enabled) {
        *memory->out.doMachineOff = false;
        *memory->out.doMachineOn = false;
    }
    setPin(enabled, KeyCodes::Buttons.reset.text);
}

// ----------------------------------------------------------------------

hal_bit_t* WhbHal::getButtonHalBit(size_t pinNumber)
{
    assert(memory->out.button_pin[pinNumber] != nullptr);
    return memory->out.button_pin[pinNumber];
}

// ----------------------------------------------------------------------

void WhbHal::setStop(bool enabled)
{
    clearStartResumeProgramStates();
    *memory->out.doStopProgram = enabled;
    setPin(enabled, KeyCodes::Buttons.stop.text);
}

// ----------------------------------------------------------------------

void WhbHal::setStart(bool enabled)
{
    if (!enabled) {
        // clear auto mode
        *memory->out.doModeAuto = false;
    }
    else {
        // request auto mode
        *memory->out.doModeAuto = true;
        toggleStartResumeProgram();
    }
    setPin(enabled, KeyCodes::Buttons.start.text);
}

// ----------------------------------------------------------------------

void WhbHal::clearStartResumeProgramStates()
{
    *memory->out.doModeAuto      = false;
    *memory->out.doPauseProgram  = false;
    *memory->out.doRunProgram    = false;
    *memory->out.doResumeProgram = false;
}

// ----------------------------------------------------------------------

void WhbHal::toggleStartResumeProgram()
{
    if (*memory->in.isProgramPaused)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = true;
    }
    if (*memory->in.isProgramRunning)
    {
        *memory->out.doPauseProgram  = true;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = false;
    }
    if (*memory->in.isProgramIdle)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = true;
        *memory->out.doResumeProgram = false;
    }
}

// ----------------------------------------------------------------------

void WhbHal::setFeedPlus(bool enabled)
{
    *memory->out.feedOverrideIncrease = enabled;
    setPin(enabled, KeyCodes::Buttons.feed_plus.text);
}

// ----------------------------------------------------------------------

void WhbHal::setFeedMinus(bool enabled)
{
    *memory->out.feedOverrideDecrease = enabled;
    setPin(enabled, KeyCodes::Buttons.feed_minus.text);
}

// ----------------------------------------------------------------------

void WhbHal::setSpindlePlus(bool enabled)
{
    if (enabled)
    {
        *memory->out.spindleOverrideDoIncrease = true;
    }
    else
    {
        *memory->out.spindleOverrideDoIncrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_plus.text);
}

// ----------------------------------------------------------------------

void WhbHal::setSpindleMinus(bool enabled)
{
    if (enabled)
    {
        *memory->out.spindleOverrideDoDecrease = true;
    }
    else
    {
        *memory->out.spindleOverrideDoDecrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_minus.text);
}

// ----------------------------------------------------------------------

void WhbHal::setMachineHome(bool enabled)
{
    enableMdiMode(enabled);
    *memory->out.homeAll = enabled;
    setPin(enabled, KeyCodes::Buttons.machine_home.text);
}

// ----------------------------------------------------------------------

void WhbHal::setSafeZ(bool enabled)
{
    enableMdiMode(enabled);
    setPin(enabled, KeyCodes::Buttons.safe_z.text);
}

// ----------------------------------------------------------------------

void WhbHal::setWorkpieceHome(bool enabled)
{
    enableMdiMode(enabled);
    setPin(enabled, KeyCodes::Buttons.workpiece_home.text);
}

// ----------------------------------------------------------------------

void WhbHal::setSpindleOn(bool enabled)
{
    if (enabled)
    {
        // stop spindle
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleStop = enabled;
        }
        else // start spindle
        {
            *memory->out.spindleStart = enabled;
        }
    }
    else
    {
        *memory->out.spindleStop  = false;
        *memory->out.spindleStart = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_on_off.text);
}

// ----------------------------------------------------------------------

void WhbHal::setProbeZ(bool enabled)
{
    enableMdiMode(enabled);
    setPin(enabled, KeyCodes::Buttons.probe_z.text);
}

// ----------------------------------------------------------------------

void WhbHal::setContinuousMode(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.manual_pulse_generator.text);
}

// ----------------------------------------------------------------------

void WhbHal::setStepMode(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.step_continuous.text);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro1(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.feed_plus.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro2(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.feed_minus.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro3(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleDoIncrease = true;
        }
    }
    else
    {
        *memory->out.spindleDoIncrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_plus.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro4(bool enabled)
{
    if (enabled)
    {
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleDoDecrease = true;
        }
    }
    else
    {
        *memory->out.spindleDoDecrease = false;
    }
    setPin(enabled, KeyCodes::Buttons.spindle_minus.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro5(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.machine_home.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro6(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.safe_z.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro7(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.workpiece_home.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro8(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.spindle_on_off.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro9(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.probe_z.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro10(bool enabled)
{
    enableManualMode(enabled);
    setPin(enabled, KeyCodes::Buttons.macro10.text);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro11(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.reset.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro12(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.stop.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro13(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.start.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro14(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.macro10.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro15(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.manual_pulse_generator.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setMacro16(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.step_continuous.altText);
}

// ----------------------------------------------------------------------

void WhbHal::setPin(bool enabled, size_t pinNumber, const char* pinName)
{
    *mHalCout << "hal   " << pinName << ((enabled) ? " enabled" : " disabled") << " (pin # " << pinNumber << ")"
              << endl;
    *(memory->out.button_pin[pinNumber]) = enabled;
}

// ----------------------------------------------------------------------

void WhbHal::setPin(bool enabled, const char* pinName)
{
    std::string fullyQualifiedPinName = std::string(mComponentPrefix) + ".button." + pinName;
    assert(mButtonNameToIdx.find(fullyQualifiedPinName) != mButtonNameToIdx.end());
    size_t pinNumber = mButtonNameToIdx[fullyQualifiedPinName];
    setPin(enabled, pinNumber, pinName);
}
// ----------------------------------------------------------------------

void WhbHal::newJogDialDelta(int8_t delta)
{
    *memory->out.jogCount += delta;

    std::ios init(NULL);
    init.copyfmt(*mHalCout);
    *mHalCout << std::setfill(' ')
              << "hal   new jog dial delta " << std::setw(3) << static_cast<signed short>(delta) << endl;
    mHalCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbHal::setJogWheelStepMode(HandwheelStepmodes::Mode stepMode)
{
    mStepMode = stepMode;

    if (stepMode == HandwheelStepmodes::Mode::STEP)
    {
        *mHalCout << "hal   step mode is step" << endl;
    }
    else
    {
        *mHalCout << "hal   step mode is continuous" << endl;
    }
}

// ----------------------------------------------------------------------

void WhbHal::setFunction(bool enabled)
{
    setPin(enabled, KeyCodes::Buttons.function.text);
}

// ----------------------------------------------------------------------

void WhbHal::enableManualMode(bool isRisingEdge)
{
    if (isRisingEdge)
    {
        if (*memory->in.isModeManual || *memory->in.isModeMdi)
        {
            *memory->out.doModeManual = true;
        }
    }
    else
    {
        *memory->out.doModeManual = false;
    }
}

// ----------------------------------------------------------------------

void WhbHal::enableMdiMode(bool isRisingEdge)
{
    if (isRisingEdge)
    {
        if (*memory->in.isModeManual || *memory->in.isModeMdi)
        {
            *memory->out.doModeMdi = true;
        }
    }
    else
    {
        *memory->out.doModeMdi = false;
    }
}

// ----------------------------------------------------------------------

void WhbHal::computeVelocity()
{
    /*
    timeval now, delta_tv;
    gettimeofday(&now, nullptr);

    if (velocityComputation.last_tv.tv_sec == 0)
    {
        velocityComputation.last_tv = now;
    }
    timersub(&now, &velocityComputation.last_tv, &delta_tv);
    float elapsed = delta_tv.tv_sec + 1e-6f * delta_tv.tv_usec;
    if (elapsed <= 0)
        return;

    float delta_pos = (*(memory->out.jogCount) - velocityComputation.last_jog_counts) * *(memory->out.jogScale);
    float velocity  = *(memory->in.jogMaxVelocity) * 60.0f * *(memory->out.jogScale);
    float k         = 0.05f;

    if (delta_pos)
    {
        *(memory->out.jogVelocity)  = (1 - k) * *(memory->out.jogVelocity) + k * velocity;
        *(memory->out.jogIncrement) = rtapi_fabs(delta_pos);

        *(memory->out.jogXIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointXSelect);
        *(memory->out.jogXIncrementMinus) = (delta_pos < 0) && *(memory->out.jointXSelect);
        *(memory->out.jogYIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointYSelect);
        *(memory->out.jogYIncrementMinus) = (delta_pos < 0) && *(memory->out.jointYSelect);
        *(memory->out.jogZIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointZSelect);
        *(memory->out.jogZIncrementMinus) = (delta_pos < 0) && *(memory->out.jointZSelect);
        *(memory->out.jogAIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointASelect);
        *(memory->out.jogAIncrementMinus) = (delta_pos < 0) && *(memory->out.jointASelect);
        *(memory->out.jogBIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointBSelect);
        *(memory->out.jogBIncrementMinus) = (delta_pos < 0) && *(memory->out.jointBSelect);
        *(memory->out.jogCIncrementPlus)  = (delta_pos > 0) && *(memory->out.jointCSelect);
        *(memory->out.jogCIncrementMinus) = (delta_pos < 0) && *(memory->out.jointCSelect);
        velocityComputation.last_jog_counts = *(memory->out.jogCount);
        velocityComputation.last_tv         = now;
    }
    else
    {
        *(memory->out.jogVelocity) = (1 - k) * *(memory->out.jogVelocity);
        if (elapsed > 0.25)
        {
            *(memory->out.jogVelocity)        = 0;
            *(memory->out.jogXIncrementPlus)  = 0;
            *(memory->out.jogXIncrementMinus) = 0;
            *(memory->out.jogYIncrementPlus)  = 0;
            *(memory->out.jogYIncrementMinus) = 0;
            *(memory->out.jogZIncrementPlus)  = 0;
            *(memory->out.jogZIncrementMinus) = 0;
            *(memory->out.jogAIncrementPlus)  = 0;
            *(memory->out.jogAIncrementMinus) = 0;
            *(memory->out.jogBIncrementPlus)  = 0;
            *(memory->out.jogBIncrementMinus) = 0;
            *(memory->out.jogCIncrementPlus)  = 0;
            *(memory->out.jogCIncrementMinus) = 0;
        }
    }*/
}
// ----------------------------------------------------------------------

WhbVelocityComputation::WhbVelocityComputation() :
    last_jog_counts(-1),
    last_tv()
{
}
}
