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
    xWorkpieceCoordinate(nullptr),
    yWorkpieceCoordinate(nullptr),
    zWorkpieceCoordinate(nullptr),
    aWorkpieceCoordinate(nullptr),
    bWorkpieceCoordinate(nullptr),
    cWorkpieceCoordinate(nullptr),
    xMachineCoordinate(nullptr),
    yMachineCoordinate(nullptr),
    zMachineCoordinate(nullptr),
    aMachineCoordinate(nullptr),
    bMachineCoordinate(nullptr),
    cMachineCoordinate(nullptr),
    feedrateOverride(nullptr),
    feedrate(nullptr),
    spindleIsOn(nullptr),
    spindleOverrideValue(nullptr),
    spindleRps(nullptr),
    jogMaxVelocity(nullptr),
    stepsizeUp(nullptr),
    isProgramRunning(nullptr),
    isProgramPaused(nullptr),
    isProgramIdle(nullptr)
{
}

// ----------------------------------------------------------------------

WhbHalMemory::Out::Out() :
    button_pin{0},
    //jogEnableOff(nullptr),
    /*jogEnableX(nullptr),
    jogEnableY(nullptr),
    jogEnableZ(nullptr),
    jogEnableA(nullptr),
    jogEnableB(nullptr),
    jogEnableC(nullptr),*/
    jogCount(nullptr),
    jogCountNeg(nullptr),
    jogVelocity(nullptr),
    feedOverrideDecrease(nullptr),
    feedOverrideIncrease(nullptr),
    spindleOverrideDecrease(nullptr),
    spindleOverrideIncrease(nullptr),
    jogXIncrementValue(nullptr),
    jogXIncrementPlus(nullptr),
    jogXIncrementMinus(nullptr),
    jogYIncrementValue(nullptr),
    jogYIncrementPlus(nullptr),
    jogYIncrementMinus(nullptr),
    jogZIncrementValue(nullptr),
    jogZIncrementPlus(nullptr),
    jogZIncrementMinus(nullptr),
    jogAIncrementValue(nullptr),
    jogAIncrementPlus(nullptr),
    jogAIncrementMinus(nullptr),
    jogBIncrementValue(nullptr),
    jogBIncrementPlus(nullptr),
    jogBIncrementMinus(nullptr),
    jogCIncrementValue(nullptr),
    jogCIncrementPlus(nullptr),
    jogCIncrementMinus(nullptr),
    jogSpeedValue(nullptr),
    jogXSpeedPlus(nullptr),
    jogXSpeedMinus(nullptr),
    jogYSpeedPlus(nullptr),
    jogYSpeedMinus(nullptr),
    jogZSpeedPlus(nullptr),
    jogZSpeedMinus(nullptr),
    jogASpeedPlus(nullptr),
    jogASpeedMinus(nullptr),
    jogBSpeedPlus(nullptr),
    jogBSpeedMinus(nullptr),
    jogCSpeedPlus(nullptr),
    jogCSpeedMinus(nullptr),
    homeAll(nullptr),
    jogIncrement(nullptr),
    /*jogPlusX(nullptr),
    jogPlusY(nullptr),
    jogPlusZ(nullptr),
    jogPlusA(nullptr),
    jogPlusB(nullptr),
    jogPlusC(nullptr),
    jogMinusX(nullptr),
    jogMinusY(nullptr),
    jogMinusZ(nullptr),
    jogMinusA(nullptr),
    jogMinusB(nullptr),
    jogMinusC(nullptr),*/
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
    doEmergencyStop(nullptr)
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
    velocityComputation(),
    mIsSimulationMode(true),
    mName("xhc-whb04b-6"),
    mHalCompId(-1),
    mDevNull(nullptr),
    mHalCout(&mDevNull),
    mStepMode(JogWheelStepMode::STEP)
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

    freeSimulatedPin((void**)(&memory->in.xWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.yWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.zWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.aWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.bWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.cWorkpieceCoordinate));
    freeSimulatedPin((void**)(&memory->in.xMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.yMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.zMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.aMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.bMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.cMachineCoordinate));
    freeSimulatedPin((void**)(&memory->in.feedrateOverride));
    freeSimulatedPin((void**)(&memory->in.feedrate));
    freeSimulatedPin((void**)(&memory->in.spindleIsOn));
    freeSimulatedPin((void**)(&memory->in.spindleOverrideValue));
    freeSimulatedPin((void**)(&memory->in.spindleRps));
    for (size_t idx = 0; idx < (sizeof(memory->out.button_pin) / sizeof(hal_bit_t * )); idx++)
    {
        freeSimulatedPin((void**)(&memory->out.button_pin[idx]));
    }
    //freeSimulatedPin((void**)(&memory->out.jogEnableOff));
    /*freeSimulatedPin((void**)(&memory->out.jogEnableX));
    freeSimulatedPin((void**)(&memory->out.jogEnableY));
    freeSimulatedPin((void**)(&memory->out.jogEnableZ));
    freeSimulatedPin((void**)(&memory->out.jogEnableA));
    freeSimulatedPin((void**)(&memory->out.jogEnableB));
    freeSimulatedPin((void**)(&memory->out.jogEnableC));*/
    freeSimulatedPin((void**)(&memory->out.jogCount));
    freeSimulatedPin((void**)(&memory->out.jogCountNeg));
    freeSimulatedPin((void**)(&memory->out.jogVelocity));
    freeSimulatedPin((void**)(&memory->out.feedOverrideDecrease));
    freeSimulatedPin((void**)(&memory->out.feedOverrideIncrease));
    freeSimulatedPin((void**)(&memory->out.spindleStart));
    freeSimulatedPin((void**)(&memory->out.spindleStop));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideDecrease));
    freeSimulatedPin((void**)(&memory->out.spindleOverrideIncrease));
    freeSimulatedPin((void**)(&memory->in.jogMaxVelocity));
    freeSimulatedPin((void**)(&memory->out.jogIncrement));
    freeSimulatedPin((void**)(&memory->out.jogXIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogXIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogXIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogYIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogYIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogYIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogZIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogZIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogZIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogAIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogAIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogAIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogBIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogBIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogBIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogCIncrementValue));
    freeSimulatedPin((void**)(&memory->out.jogCIncrementPlus));
    freeSimulatedPin((void**)(&memory->out.jogCIncrementMinus));
    freeSimulatedPin((void**)(&memory->out.jogSpeedValue));
    freeSimulatedPin((void**)(&memory->out.jogXSpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogXSpeedMinus));
    freeSimulatedPin((void**)(&memory->out.jogYSpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogYSpeedMinus));
    freeSimulatedPin((void**)(&memory->out.jogZSpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogZSpeedMinus));
    freeSimulatedPin((void**)(&memory->out.jogASpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogASpeedMinus));
    freeSimulatedPin((void**)(&memory->out.jogBSpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogBSpeedMinus));
    freeSimulatedPin((void**)(&memory->out.jogCSpeedPlus));
    freeSimulatedPin((void**)(&memory->out.jogCSpeedMinus));
    freeSimulatedPin((void**)(&memory->out.homeAll));
    /* freeSimulatedPin((void**)(&memory->out.jogPlusX));
     freeSimulatedPin((void**)(&memory->out.jogPlusY));
     freeSimulatedPin((void**)(&memory->out.jogPlusZ));
     freeSimulatedPin((void**)(&memory->out.jogPlusA));
     freeSimulatedPin((void**)(&memory->out.jogPlusB));
     freeSimulatedPin((void**)(&memory->out.jogPlusC));
     freeSimulatedPin((void**)(&memory->out.jogMinusX));
     freeSimulatedPin((void**)(&memory->out.jogMinusY));
     freeSimulatedPin((void**)(&memory->out.jogMinusZ));
     freeSimulatedPin((void**)(&memory->out.jogMinusA));
     freeSimulatedPin((void**)(&memory->out.jogMinusB));
     freeSimulatedPin((void**)(&memory->out.jogMinusC));*/
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
    freeSimulatedPin((void**)(&memory->out.doEmergencyStop));

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
        *mHalCout << " pin" << endl;
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

        newHalBit(HAL_OUT, &((memory->out.button_pin)[idx]), mHalCompId, "%s.out.button.%s", mName,
                  buttonName);
    }

    newHalBit(HAL_OUT, &(memory->out.isPendantSleeping), mHalCompId, "%s.out.pendant.is-sleeping", mName);
    newHalBit(HAL_OUT, &(memory->out.isPendantConnected), mHalCompId, "%s.out.pendant.is-connected", mName);
    newHalBit(HAL_OUT, &(memory->out.isPendantRequired), mHalCompId, "%s.out.pendant.is-required", mName);

    newHalFloat(HAL_IN, &(memory->in.xMachineCoordinate), mHalCompId, "%s.in.halui.axis.x.pos-commanded",
                mName);
    newHalFloat(HAL_IN, &(memory->in.yMachineCoordinate), mHalCompId, "%s.in.halui.axis.y.pos-commanded",
                mName);
    newHalFloat(HAL_IN, &(memory->in.zMachineCoordinate), mHalCompId, "%s.in.halui.axis.z.pos-commanded",
                mName);
    newHalFloat(HAL_IN, &(memory->in.aMachineCoordinate), mHalCompId, "%s.in.halui.axis.a.pos-commanded",
                mName);
    newHalFloat(HAL_IN, &(memory->in.bMachineCoordinate), mHalCompId, "%s.in.halui.axis.b.pos-commanded",
                mName);
    newHalFloat(HAL_IN, &(memory->in.cMachineCoordinate), mHalCompId, "%s.in.halui.axis.c.pos-commanded",
                mName);

    newHalFloat(HAL_IN, &(memory->in.yWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.y.pos-relative",
                mName);
    newHalFloat(HAL_IN, &(memory->in.xWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.x.pos-relative",
                mName);
    newHalFloat(HAL_IN, &(memory->in.zWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.z.pos-relative",
                mName);
    newHalFloat(HAL_IN, &(memory->in.aWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.a.pos-relative",
                mName);
    newHalFloat(HAL_IN, &(memory->in.bWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.b.pos-relative",
                mName);
    newHalFloat(HAL_IN, &(memory->in.cWorkpieceCoordinate), mHalCompId, "%s.in.halui.axis.c.pos-relative",
                mName);

    newHalFloat(HAL_IN, &(memory->in.feedrate), mHalCompId, "%s.in.feed", mName);
    newHalFloat(HAL_IN, &(memory->in.feedrateOverride), mHalCompId, "%s.in.halui.feed-override.value", mName);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideIncrease), mHalCompId, "%s.out.halui.feed-override.increase",
              mName);
    newHalBit(HAL_OUT, &(memory->out.feedOverrideDecrease), mHalCompId, "%s.out.halui.feed-override.decrease",
              mName);

    newHalSigned32(HAL_OUT, &(memory->out.stepsize), mHalCompId, "%s.out.stepsize", mName);
    newHalBit(HAL_IN, &(memory->in.stepsizeUp), mHalCompId, "%s.in.stepsize-up", mName);

    newHalFloat(HAL_IN, &(memory->in.spindleRps), mHalCompId, "%s.in.spindle-rps", mName);
    newHalBit(HAL_IN, &(memory->in.spindleIsOn), mHalCompId, "%s.in.halui.spindle.is-on", mName);
    newHalFloat(HAL_IN, &(memory->in.spindleOverrideValue), mHalCompId, "%s.in.halui.spindle-override.value",
                mName);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideIncrease), mHalCompId, "%s.out.halui.spindle.increase",
              mName);
    newHalBit(HAL_OUT, &(memory->out.spindleOverrideDecrease), mHalCompId, "%s.out.halui.spindle.decrease",
              mName);

    newHalBit(HAL_OUT, &(memory->out.spindleStart), mHalCompId, "%s.out.halui.spindle.start", mName);
    newHalBit(HAL_OUT, &(memory->out.spindleStop), mHalCompId, "%s.out.halui.spindle.stop", mName);

    newHalBit(HAL_OUT, &(memory->out.doEmergencyStop), mHalCompId, "%s.out.halui.estop.activate", mName);
    newHalBit(HAL_IN, &(memory->in.isProgramIdle), mHalCompId, "%s.in.halui.program.is-idle", mName);
    newHalBit(HAL_IN, &(memory->in.isProgramPaused), mHalCompId, "%s.in.halui.program.is-paused", mName);
    newHalBit(HAL_IN, &(memory->in.isProgramRunning), mHalCompId, "%s.in.halui.program.is-running", mName);
    newHalBit(HAL_OUT, &(memory->out.doResumeProgram), mHalCompId, "%s.out.halui.program.resume", mName);
    newHalBit(HAL_OUT, &(memory->out.doPauseProgram), mHalCompId, "%s.out.halui.program.pause", mName);
    newHalBit(HAL_OUT, &(memory->out.doRunProgram), mHalCompId, "%s.out.halui.program.run", mName);
    newHalBit(HAL_OUT, &(memory->out.doStopProgram), mHalCompId, "%s.out.halui.program.stop", mName);

    /*r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableOff), mHalCompId, "%s.out.jog.enable-off", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableX), mHalCompId, "%s.out.jog.enable-x", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableY), mHalCompId, "%s.out.jog.enable-y", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableZ), mHalCompId, "%s.out.jog.enable-z", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableA), mHalCompId, "%s.out.jog.enable-a", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogEnableB), mHalCompId, "%s.out.jog.enable-b", mName);
    r |= newHalBit(HAL_OUT, &(memory->out.jogEnableC), mHalCompId, "%s.out.jog.enable-c", mName);*/

    newHalBit(HAL_OUT, &(memory->out.jointXSelect), mHalCompId, "%s.out.halui.joint.x.select", mName);
    newHalBit(HAL_OUT, &(memory->out.jointYSelect), mHalCompId, "%s.out.halui.joint.y.select", mName);
    newHalBit(HAL_OUT, &(memory->out.jointZSelect), mHalCompId, "%s.out.halui.joint.z.select", mName);
    newHalBit(HAL_OUT, &(memory->out.jointASelect), mHalCompId, "%s.out.halui.joint.a.select", mName);
    newHalBit(HAL_OUT, &(memory->out.jointBSelect), mHalCompId, "%s.out.halui.joint.b.select", mName);
    newHalBit(HAL_OUT, &(memory->out.jointCSelect), mHalCompId, "%s.out.halui.joint.c.select", mName);

    newHalFloat(HAL_OUT, &(memory->out.jogXIncrementValue), mHalCompId, "%s.out.halui.jog.x.increment", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogYIncrementValue), mHalCompId, "%s.out.halui.jog.y.increment", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogZIncrementValue), mHalCompId, "%s.out.halui.jog.z.increment", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogAIncrementValue), mHalCompId, "%s.out.halui.jog.a.increment", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogBIncrementValue), mHalCompId, "%s.out.halui.jog.b.increment", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogCIncrementValue), mHalCompId, "%s.out.halui.jog.c.increment", mName);

    newHalBit(HAL_OUT, &(memory->out.jogXIncrementPlus), mHalCompId, "%s.out.halui.jog.x.increment-plus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogYIncrementPlus), mHalCompId, "%s.out.halui.jog.y.increment-plus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogZIncrementPlus), mHalCompId, "%s.out.halui.jog.z.increment-plus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogAIncrementPlus), mHalCompId, "%s.out.halui.jog.a.increment-plus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogBIncrementPlus), mHalCompId, "%s.out.halui.jog.b.increment-plus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogCIncrementPlus), mHalCompId, "%s.out.halui.jog.c.increment-plus",
              mName);

    newHalBit(HAL_OUT, &(memory->out.jogXIncrementMinus), mHalCompId, "%s.out.halui.jog.x.increment-minus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogYIncrementMinus), mHalCompId, "%s.out.halui.jog.y.increment-minus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogZIncrementMinus), mHalCompId, "%s.out.halui.jog.z.increment-minus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogAIncrementMinus), mHalCompId, "%s.out.halui.jog.a.increment-minus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogBIncrementMinus), mHalCompId, "%s.out.halui.jog.b.increment-minus",
              mName);
    newHalBit(HAL_OUT, &(memory->out.jogCIncrementMinus), mHalCompId, "%s.out.halui.jog.c.increment-minus",
              mName);

    newHalFloat(HAL_OUT, &(memory->out.jogSpeedValue), mHalCompId, "%s.out.halui.jog-speed", mName);

    newHalBit(HAL_OUT, &(memory->out.jogXSpeedPlus), mHalCompId, "%s.out.halui.jog.x.speed-plus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogYSpeedPlus), mHalCompId, "%s.out.halui.jog.y.speed-plus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogZSpeedPlus), mHalCompId, "%s.out.halui.jog.z.speed-plus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogASpeedPlus), mHalCompId, "%s.out.halui.jog.a.speed-plus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogBSpeedPlus), mHalCompId, "%s.out.halui.jog.b.speed-plus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogCSpeedPlus), mHalCompId, "%s.out.halui.jog.c.speed-plus", mName);

    newHalBit(HAL_OUT, &(memory->out.jogXSpeedMinus), mHalCompId, "%s.out.halui.jog.x.speed-minus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogYSpeedMinus), mHalCompId, "%s.out.halui.jog.y.speed-minus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogZSpeedMinus), mHalCompId, "%s.out.halui.jog.z.speed-minus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogASpeedMinus), mHalCompId, "%s.out.halui.jog.a.speed-minus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogBSpeedMinus), mHalCompId, "%s.out.halui.jog.b.speed-minus", mName);
    newHalBit(HAL_OUT, &(memory->out.jogCSpeedMinus), mHalCompId, "%s.out.halui.jog.c.speed-minus", mName);

    newHalSigned32(HAL_OUT, &(memory->out.jogCount), mHalCompId, "%s.out.jog.counts", mName);
    newHalSigned32(HAL_OUT, &(memory->out.jogCountNeg), mHalCompId, "%s.out.jog.counts-neg", mName);

    newHalFloat(HAL_OUT, &(memory->out.jogVelocity), mHalCompId, "%s.out.jog.velocity", mName);
    newHalFloat(HAL_IN, &(memory->in.jogMaxVelocity), mHalCompId, "%s.in.halui.max-velocity.value", mName);
    newHalFloat(HAL_OUT, &(memory->out.jogIncrement), mHalCompId, "%s.out.jog.increment", mName);

    /*
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusX), mHalCompId, "%s.out.jog.plus-x", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusY), mHalCompId, "%s.out.jog.plus-y", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusZ), mHalCompId, "%s.out.jog.plus-z", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusA), mHalCompId, "%s.out.jog.plus-a", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusB), mHalCompId, "%s.out.jog.plus-b", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogPlusC), mHalCompId, "%s.out.jog.plus-c", mName);

    r |= newHalBit(HAL_OUT, &(memory->out.jogMinusX), mHalCompId, "%s.out.jog.minus-x", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogMinusY), mHalCompId, "%s.out.jog.minus-y", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogMinusZ), mHalCompId, "%s.out.jog.minus-z", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogMinusA), mHalCompId, "%s.out.jog.minus-a", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogMinusB), mHalCompId, "%s.out.jog.minus-b", mName);
    r |= newBitHalPin(HAL_OUT, &(memory->out.jogMinusC), mHalCompId, "%s.out.jog.minus-c", mName);*/

    newHalBit(HAL_OUT, &(memory->out.homeAll), mHalCompId, "%s.out.halui.home-all", mName);
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
    if (mStepMode == JogWheelStepMode::STEP)
    {
        *memory->out.jogIncrement  = stepSize;
        *memory->out.jogSpeedValue = 0;
    }
    else
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


void WhbHal::setReset(bool enabled, size_t pinNumber)
{
    *memory->out.doEmergencyStop = enabled;
    setPin(enabled, pinNumber, "reset");
}

// ----------------------------------------------------------------------

hal_bit_t* WhbHal::getButtonHalBit(size_t pinNumber)
{
    assert(memory->out.button_pin[pinNumber] != nullptr);
    return memory->out.button_pin[pinNumber];
}

// ----------------------------------------------------------------------

void WhbHal::setStop(bool enabled, size_t pinNumber)
{
    *memory->out.doStopProgram = enabled;
    setPin(enabled, pinNumber, "stop");
}

// ----------------------------------------------------------------------

void WhbHal::setStart(bool enabled, size_t pinNumber)
{
    if (memory->in.isProgramPaused)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = true;
    }
    if (memory->in.isProgramRunning)
    {
        *memory->out.doPauseProgram  = true;
        *memory->out.doRunProgram    = false;
        *memory->out.doResumeProgram = false;
    }
    if (memory->in.isProgramIdle)
    {
        *memory->out.doPauseProgram  = false;
        *memory->out.doRunProgram    = true;
        *memory->out.doResumeProgram = false;
    }
    setPin(enabled, pinNumber, "start/stop");
}

// ----------------------------------------------------------------------

void WhbHal::setFeedPlus(bool enabled, size_t pinNumber)
{
    *memory->out.feedOverrideIncrease = enabled;
    setPin(enabled, pinNumber, "feed-plus");
}

// ----------------------------------------------------------------------

void WhbHal::setFeedMinus(bool enabled, size_t pinNumber)
{
    *memory->out.feedOverrideDecrease = enabled;
    setPin(enabled, pinNumber, "feed-minus");
}

// ----------------------------------------------------------------------

void WhbHal::setSpindlePlus(bool enabled, size_t pinNumber)
{
    *memory->out.spindleOverrideIncrease = enabled;
    setPin(enabled, pinNumber, "spindle-plus");
}

// ----------------------------------------------------------------------

void WhbHal::setSpindleMinus(bool enabled, size_t pinNumber)
{
    *memory->out.spindleOverrideDecrease = enabled;
    setPin(enabled, pinNumber, "spindle-minus");
}

// ----------------------------------------------------------------------

void WhbHal::setMachineHome(bool enabled, size_t pinNumber)
{
    *memory->out.homeAll = enabled;
    setPin(enabled, pinNumber, "machine-home");
}

// ----------------------------------------------------------------------

void WhbHal::setSafeZ(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "safe-z");
}

// ----------------------------------------------------------------------

void WhbHal::setWorkpieceHome(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "workpiece-home");
}

// ----------------------------------------------------------------------

void WhbHal::setSpindleOn(bool enabled, size_t pinNumber)
{
    // on button pressed
    if (enabled)
    {
        // generate rising edge on respective pins
        if (*memory->in.spindleIsOn)
        {
            *memory->out.spindleStop = enabled;
        }
        else
        {
            *memory->out.spindleStart = enabled;
        }
    }
        // on button released
    else
    {
        // pull pins to low
        *memory->out.spindleStop  = false;
        *memory->out.spindleStart = false;
    }
    setPin(enabled, pinNumber, "spindle-on/off");
}

// ----------------------------------------------------------------------

void WhbHal::setProbeZ(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "probe-z");
}

// ----------------------------------------------------------------------

void WhbHal::setContinuousMode(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "continuous-mode");
}

// ----------------------------------------------------------------------

void WhbHal::setStepMode(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "step-mode");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro1(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-1");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro2(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-2");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro3(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-3");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro4(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-4");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro5(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-5");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro6(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-6");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro7(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-7");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro8(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-8");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro9(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-9");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro10(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-10");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro11(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-11");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro12(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-12");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro13(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-13");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro14(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-14");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro15(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-15");
}

// ----------------------------------------------------------------------

void WhbHal::setMacro16(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "macro-16");
}

// ----------------------------------------------------------------------

void WhbHal::setPin(bool enabled, size_t pinNumber, const char* pinName)
{
    *mHalCout << "hal   " << pinName << ((enabled) ? " enabled" : " disabled") << " (pin # " << pinNumber << ")"
              << endl;
    *(memory->out.button_pin[pinNumber]) = enabled;
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

void WhbHal::setJogWheelStepMode(JogWheelStepMode stepMode)
{
    mStepMode = stepMode;

    if (stepMode == JogWheelStepMode::STEP)
    {
        *mHalCout << "hal   step mode is step" << endl;
    }
    else
    {
        *mHalCout << "hal   step mode is continuous" << endl;
    }
}

// ----------------------------------------------------------------------

void WhbHal::setFunction(bool enabled, size_t pinNumber)
{
    setPin(enabled, pinNumber, "fn");
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
