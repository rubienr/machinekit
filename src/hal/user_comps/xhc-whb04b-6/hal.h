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

#pragma once

// system includes
#include <stdint.h>
#include <ostream>

// 3rd party includes

// local library includes
#include <hal.h>

// forward declarations

namespace XhcWhb04b6 {

// forward declarations
class WhbSoftwareButton;
class WhbKeyCodes;
enum class JogWheelStepMode : uint8_t;

// ----------------------------------------------------------------------

//! HAL memory pointers. Each pointer represents an i/o hal pin.

class WhbHalMemory
{
public:
    struct In
    {
    public:
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* xWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* yWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* zWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* aWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* bWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-relative
        hal_float_t* cWorkpieceCoordinate;
        //! to be connected to \ref halui.axis.N.pos-commanded
        hal_float_t* xMachineCoordinate;
        //! to be connected to \ref halui.axis.N.pos-commanded
        hal_float_t* yMachineCoordinate;
        //! to be connected to \ref halui.axis.N.pos-commanded
        hal_float_t* zMachineCoordinate;
        //! to be connected to \ref halui.axis.N.pos-commanded
        hal_float_t* aMachineCoordinate;
        //! to be connected to \ref halui.axis.N.pos-commanded
        hal_float_t* bMachineCoordinate;
        //! to be connected to \ref halui.axis.C.pos-commanded
        hal_float_t* cMachineCoordinate;
        //! to be connected to \ref halui.feed-override.value
        hal_float_t* feedrateOverride;
        // TODO: where should it be connected to
        hal_float_t* feedrate;
        //! to be connected to \ref halui.spindle.is-on
        hal_bit_t  * spindleIsOn;
        //! to be connected to \ref halui.spindle-override.value
        hal_float_t* spindleOverrideValue;
        // TODO: where should it be connected to
        hal_float_t* spindleRps;
        //! to be connected to \ref halui.max-velocity.value
        hal_float_t* jogMaxVelocity;
        // TODO: where should it be connected to
        hal_bit_t  * stepsizeUp;
        //! to be connected to \ref halui.program.is-running// system includes
        hal_bit_t  * isProgramRunning;
        //! to be connected to \ref halui.program.is-paused
        hal_bit_t  * isProgramPaused;
        //! to be connected to \ref halui.program.is-idle
        hal_bit_t  * isProgramIdle;

        In();
    };

    struct Out
    {
    public:
        hal_bit_t* button_pin[64];
        //hal_bit_t  * jogEnableOff;
        /*hal_bit_t  * jogEnableX;
        hal_bit_t  * jogEnableY;// system includes
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <iomanip>
#include <sstream>
#include <assert.h>
#include <signal.h>
#include <libusb.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <libgen.h>

// 3rd party includes
#include <google/protobuf/stubs/common.h>

// local library includes
#include "rtapi_math.h"
#include "hal.h"
#include "inifile.hh"
#include "config.h"
        hal_bit_t  * jogEnableZ;
        hal_bit_t  * jogEnableA;
        hal_bit_t  * jogEnableB;
        hal_bit_t  * jogEnableC;*/
        //! Incremental jog counter. Counts will be consumed by icomp.
        hal_s32_t* jogCount;

        hal_s32_t* jogCountNeg;

        hal_float_t* jogVelocity;

        //! to be connected to \ref halui.feed-override.decrease
        hal_bit_t* feedOverrideDecrease;
        //! to be connected to \ref halui.feed-override.increase
        hal_bit_t* feedOverrideIncrease;

        //! to be connected to \ref halui.spindle.start
        hal_bit_t* spindleStart;
        //! to be connected to \ref halui.spindle.stop
        hal_bit_t* spindleStop;

        //! to be connected to halui.spindle.decrease
        hal_bit_t* spindleOverrideDecrease;
        //! to be connected to halui.spindle.increase
        hal_bit_t* spindleOverrideIncrease;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogXIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogXIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogXIncrementMinus;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogYIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogYIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogYIncrementMinus;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogZIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogZIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogZIncrementMinus;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogAIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogAIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogAIncrementMinus;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogBIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogBIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogBIncrementMinus;

        //! to be connected to \ref halui.jog.N.increment
        hal_float_t* jogCIncrementValue;
        //! to be connected to \ref halui.jog.N.increment-plus
        hal_bit_t  * jogCIncrementPlus;
        //! to be connected to \ref halui.jog.N.increment-minus
        hal_bit_t  * jogCIncrementMinus;

        //! to be connected to \ref halui.jog-speed
        hal_float_t* jogSpeedValue;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogXSpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogXSpeedMinus;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogYSpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogYSpeedMinus;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogZSpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogZSpeedMinus;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogASpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogASpeedMinus;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogBSpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogBSpeedMinus;

        //! to be connected to \ref halui.jog.N.plus
        hal_bit_t* jogCSpeedPlus;
        //! to be connected to \ref halui.jog.N.minus
        hal_bit_t* jogCSpeedMinus;

        //! to be connected to \ref halui.home-all
        hal_bit_t* homeAll;

        hal_float_t* jogIncrement;
        /*
          hal_bit_t  * jogPlusX;
          hal_bit_t  *         // TODO: begin: to be removed
;
          hal_bit_t  * jogPlusZ;
          hal_bit_t  * jogPlusA;
          hal_bit_t  * jogPlusB;
          hal_bit_t  * jogPlusC;
          hal_bit_t  * jogMinusX;
          hal_bit_t  * jogMinusY;
          hal_bit_t  * jogMinusZ;
          hal_bit_t  * jogMinusA;
          hal_bit_t  * jogMinusB;
          hal_bit_t  * jogMinusC;
  */
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointXSelect;
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointYSelect;
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointZSelect;
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointASelect;
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointBSelect;
        //!to be connected to \ref halui.joint.N.select
        hal_bit_t  * jointCSelect;

        // TODO: where should it be connected to
        hal_s32_t* stepsize;
        //! reflects the pendandt's idle state
        hal_bit_t* isPendantSleeping;
        //! reflects pendant's connectivity
        hal_bit_t* isPendantConnected;
        // TODO: remove since this is not needed to be exposed
        hal_bit_t* isPendantRequired;

        //! to be connected to \ref halui.program.run
        hal_bit_t* doRunProgram;
        //! to be connected to \ref halui.program.pause
        hal_bit_t* doPauseProgram;
        //! to be connected to \ref halui.program.resume
        hal_bit_t* doResumeProgram;
        //! to be connected to \ref halui.program.stop
        hal_bit_t* doStopProgram;

        //! to be connected to \ref halui.estop.activate
        hal_bit_t* doEmergencyStop;

        Out();
    };

    In  in;
    Out out;

    WhbHalMemory();
    ~WhbHalMemory();
};

// ----------------------------------------------------------------------

class WhbVelocityComputation
{
public:
    hal_s32_t      last_jog_counts;
    struct timeval last_tv;
    WhbVelocityComputation();
};

// ----------------------------------------------------------------------

//! HAL and related parameters
class WhbHal
{
public:
    WhbHalMemory* memory;
    WhbVelocityComputation velocityComputation;

    WhbHal();
    ~WhbHal();
    //! Initializes HAL memory and pins according to simulation mode. Must not be called more than once.
    //! If \ref mIsSimulationMode is true heap memory will be used, shared HAL memory otherwise.
    //! \ref setIsSimulationMode() must be set before accordingly
    void init(const WhbSoftwareButton* softwareButtons, const WhbKeyCodes& codes);
    bool isSimulationModeEnabled() const;
    //! indicates the program has been invoked in hal mode or normal
    void setSimulationMode(bool isSimulationMode);
    int getHalComponentId() const;
    hal_bit_t* getButtonHalBit(size_t pinNumber);
    const char* getHalComponentName() const;
    //! Enables verbose hal output.
    //! \param enable true to enable hal messages, disable otherwise
    void setEnableVerbose(bool enable);
    //! If set indicates that no other axis is active.
    //! \param enabled true if no axis is active, false otherwise
    void setNoAxisActive(bool enabled);
    //! Sets the A axis to active or disables the same.
    //! \param enabled true if axis should be the one and only active
    void setAxisXActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisYActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisZActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisAActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisBActive(bool enabled);
    //! \sa setAxisXActive(bool)
    void setAxisCActive(bool enabled);

    //! Set the new jog wheel step mode. The mode affects the pins chosen to generate jog movements.
    void setJogWheelStepMode(JogWheelStepMode stepMode);

    //! Sets the new feed rate. The step mode must be set accordingly.
    //! \param feedRate the new feed rate independent of step mode
    void setStepSize(const hal_float_t& feedRate);
    //! If lead is active.
    void setLead();
    //! Sets the hal state of the reset pin. Usually called in case the reset
    //! button is pressed or released. The pin should be connected to \ref halui.estop.activate.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory as registered in
    //! \ref WhbHal::halInit(const WhbSoftwareButton* , size_t , const WhbKeyCodes&)
    //! \sa doEmergencyStop
    void setReset(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setStop(bool enabled, size_t pinNumber);
    //! Toggles the start/pause/resume states.
    //! \param enabled true if start/resume is requested, false otherwise
    //! \param pinNumber \sa setReset(bool, size_t)
    void setStart(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setFeedPlus(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setFeedMinus(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setSpindlePlus(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setSpindleMinus(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setFunction(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setMachineHome(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setSafeZ(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setWorkpieceHome(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setSpindleOn(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setProbeZ(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setContinuousMode(bool enabled, size_t pinNumber);
    //! \sa setReset(bool, size_t)
    void setStepMode(bool enabled, size_t pinNumber);
    //! Sets the hal state of the macro pin. Usually called in case the macro
    //! button is pressed or released. A macro button can be any button
    //! when pressed together with the modifier key.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory.
    //! \sa setReset(bool, size_t)
    void setMacro1(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro2(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro3(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro4(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro5(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro6(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro7(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro8(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro9(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro10(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro11(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro12(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro13(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro14(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro15(bool enabled, size_t pinNumber);
    //! \sa setMacro1(bool, size_t)
    void setMacro16(bool enabled, size_t pinNumber);
    //! This method cumulates new jog dial delta. In other words it produces counts
    //! to be consumed by an icomp component.
    //! \param delta new jog dial delta
    void newJogDialDelta(int8_t delta);
    void computeVelocity();

private:
    bool mIsSimulationMode;
    const char* mName;
    int          mHalCompId;
    std::ostream mDevNull;
    std::ostream* mHalCout;
    JogWheelStepMode mStepMode;
    //int16_t                    mPendingStepsContinuousMode;
    //int16_t                    mPendingStepsStepMode;

    //! //! Allocates new hal_bit_t pin according to \ref mIsSimulationMode. If \ref mIsSimulationMode then
    //! mallocs memory, hal_pin_bit_new allocation otherwise.
    //! \param pin_name the pin name when registered to hal
    //! \param ptr will point to the allocated memory
    //! \param s size in bytes
    //! \return != 0 on error, 0 otherwise
    int newSimulatedHalPin(char* pin_name, void** ptr, int s);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalFloat(hal_pin_dir_t direction, hal_float_t** ptr, int componentId, const char* fmt, ...);
    //! \sa  newBitHalPin(hal_pin_dir_t, hal_bit_t**, int, const char*, ...)
    int newHalSigned32(hal_pin_dir_t direction, hal_s32_t** ptr, int componentId, const char* fmt, ...);
    //! \param direction module input or output
    //! \param ptr will point to the allocated memory
    //! \param componentId hal id
    //! \param fmt the pin name when registered to hal
    //! \param ... va args
    //! \return != 0 on error, 0 otherwise
    int newHalBit(hal_pin_dir_t direction, hal_bit_t** ptr, int componentId, const char* fmt, ...);
    //! allocates new hal pin according to \ref mIsSimulationMode
    //! \param pin pointer reference to the memory to be fred
    //! \post pin == nullptr
    void freeSimulatedPin(void** pin);
    //! \param enabled the new pin value
    //! \param pinNumber the pin number to set the value
    //! \param pinName arbitrary name for logging
    void setPin(bool enabled, size_t pinNumber, const char* pinName);
};
}
