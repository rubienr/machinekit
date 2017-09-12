/*
   XHC-WHB04B-6 Wireless MPG pendant LinuxCNC HAL module for LinuxCNC.
   Based on XHC-HB04. The implementation supports no configuration file,
   nor other variations as XHC-WHB04B-4.

   Copyright (C) 2017 Raoul Rubien (github.com/rubienr), based on
   xhc-hb04.cc

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

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <bitset>
#include <iomanip>
#include <assert.h>
#include <signal.h>
#include <libusb.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <libgen.h>
#include <google/protobuf/stubs/common.h>
#include "rtapi_math.h"
#include "hal.h"
#include "inifile.hh"
#include "config.h"

using std::cerr;
using std::endl;
using std::setfill;
using std::setw;
using std::hex;
using std::dec;
using std::ios;

//! function for libusb's incoming data callback function
void usbInputResponseCallback(struct libusb_transfer* transfer);
//! registers signal handler
void registerSignalHandler();
//! called on program termination requested
static void quit(int signal);
namespace XhcWhb04b6 {

class WhbHalMemory;
class WhbHal;
class WhbSleepDetect;
class WhbKeyCode;
class WhbSoftwareButton;
class WhbAxisRotaryButtonCodes;
class WhbFeedRotaryButtonCodes;
class WhbButtonsCode;
class WhbKeyCodes;
class WhbVelocityComputation;
class WhbHandwheelStepModeStepSize;
class WhbHandwheelContinuousModeStepSize;
class WhbStepModeStepSize;
class WhbStepHandler;
class WhbButtonsState;
class WhbUsbOutPackageBlockFields;
union WhbUsbOutPackageBlock;
class WhbUsbOutPackageBlocks;
class WhbUsbOutPackageAxisCoordinate;
class DisplayIndicatorBitFields;
union DisplayIndicator;
class WhbUsbOutPackageData;
union WhbUsbOutPackageBuffer;
class WhbUsbInPackage;
union WhbUsbInPackageBuffer;
class WhbUsbEmptyPackage;
class WhbUsbSleepPackage;
class WhbConstantUsbPackages;
class WhbUsb;
class ConstantPackages;
class UsbInputPackageListener;
class WhbContext;

std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageAxisCoordinate& coordinate);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageData& data);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlockFields& block);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlocks& blocks);

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
        //! to be connected to \ref halui.program.is-running
        hal_bit_t  * isProgramRunning;
        //! to be connected to \ref halui.program.is-paused
        hal_bit_t  * isProgramPaused;
        //! to be connected to \ref halui.program.is-idle
        hal_bit_t  * isProgramIdle;

        In() :
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
    };

    struct Out
    {
    public:
        hal_bit_t* button_pin[64];
        //hal_bit_t  * jogEnableOff;
        /*hal_bit_t  * jogEnableX;
        hal_bit_t  * jogEnableY;
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

        Out() :
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
            doStopProgram(nullptr)
        {
        }
    };

    In  in;
    Out out;

    WhbHalMemory() :
        in(),
        out()
    {
    }

    ~WhbHalMemory()
    {
    }
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

class JogWheelStepMode
{
public:
    enum StepMode
    {
        CONTINUOUS,
        STEP
    };
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
    void setJogWheelStepMode(JogWheelStepMode::StepMode stepMode);

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
    //! This method cummulates new jog dial delta. In other words it produces counts
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
    JogWheelStepMode::StepMode mStepMode;
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
    //! \param ... va agrgs
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

// ----------------------------------------------------------------------

//! pendant sleep/idle state parameters
class WhbSleepDetect
{
    friend XhcWhb04b6::WhbUsb;

public:

    WhbSleepDetect();

private:
    bool           mDropNextInPackage;
    struct timeval mLastWakeupTimestamp;
};

// ----------------------------------------------------------------------

//! pendant button key code description
class WhbKeyCode
{
public:
    const uint8_t code;
    //! default button text as written on pendant (if available)
    const char* text;
    //! alternative button text as written on pendant (if available)
    const char* altText;
    bool operator==(const WhbKeyCode& other) const;
    bool operator!=(const WhbKeyCode& other) const;
    WhbKeyCode(uint8_t code, const char* text, const char* altText);
    WhbKeyCode(const WhbKeyCode& other);
} __attribute__((packed));

// ----------------------------------------------------------------------

//! meta-button state which is dependent on the "Fn" modifier button's state
class WhbSoftwareButton
{
public:
    const WhbKeyCode& key;
    const WhbKeyCode& modifier;
    bool containsKeys(const WhbKeyCode& key, const WhbKeyCode& modifier) const;
    WhbSoftwareButton(const WhbKeyCode& key, const WhbKeyCode& modifier);
};

// ----------------------------------------------------------------------

//! rotary axis selection button related parameters
class WhbAxisRotaryButtonCodes
{
public:
    //typedef enum
    //{
    //    Off = 0, X = 1, Y = 2, Z = 3, A = 4, B = 5, C = 6
    //} AxisIndexName;

    const WhbKeyCode off;
    const WhbKeyCode x;
    const WhbKeyCode y;
    const WhbKeyCode z;
    const WhbKeyCode a;
    const WhbKeyCode b;
    const WhbKeyCode c;
    const WhbKeyCode undefined;
    WhbAxisRotaryButtonCodes();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! rotary feed button related parameters
class WhbFeedRotaryButtonCodes
{
public:
    const WhbKeyCode speed_0_001;
    const WhbKeyCode speed_0_01;
    const WhbKeyCode speed_0_1;
    const WhbKeyCode speed_1;
    const WhbKeyCode percent_60;
    const WhbKeyCode percent_100;
    const WhbKeyCode lead;
    const WhbKeyCode undefined;
    WhbFeedRotaryButtonCodes();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! pendant button related parameters
class WhbButtonsCode
{
public:
    const WhbKeyCode reset;
    const WhbKeyCode stop;
    const WhbKeyCode start;
    const WhbKeyCode feed_plus;
    const WhbKeyCode feed_minus;
    const WhbKeyCode spindle_plus;
    const WhbKeyCode spindle_minus;
    const WhbKeyCode machine_home;
    const WhbKeyCode safe_z;
    const WhbKeyCode workpiece_home;
    const WhbKeyCode spindle_on_off;
    const WhbKeyCode function;
    const WhbKeyCode probe_z;
    const WhbKeyCode macro10;
    const WhbKeyCode manual_pulse_generator;
    const WhbKeyCode step_continuous;
    const WhbKeyCode undefined;
    const WhbKeyCode& getKeyCode(uint8_t keyCode) const;
    WhbButtonsCode();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! whb button and rotary button codes
class WhbKeyCodes
{
public:
    const WhbButtonsCode           buttons;
    const WhbAxisRotaryButtonCodes axis;
    const WhbFeedRotaryButtonCodes feed;
    WhbKeyCodes();
};

// ----------------------------------------------------------------------
//! If hand wheel is in Lead mode (activated by the feed rotary button) this speed setting is applied.
class WhbHandwheelLeadModeStepSize
{
public:
    enum PositionNameIndex
    {
        RotaryButtonLeed = 6
    };
};
// ----------------------------------------------------------------------
//! If hand wheel is in step mode (toggled by Step/Continuous" button) this speed setting is applied.
//! In step mode the step is in machine units distance.
class WhbHandwheelStepModeStepSize
{
public:
    enum PositionNameIndex
    {
        RotaryButton0001      = 0,
        RotaryButton0010      = 1,
        RotaryButton0100      = 2,
        RotaryButton100       = 3,
        RotaryButtonUndefined = 4
    };

    //! Translates the button position to step metric units.
    //! \param buttonPosition
    //! \return the step size in units ∈ {0.001, 0.01, 0.1, 1.0, 0,0}
    float getStepSize(PositionNameIndex buttonPosition) const;
    WhbHandwheelStepModeStepSize();

private:
    const float mSequence[5];
};

// ----------------------------------------------------------------------

//! If hand wheel is in continuous mode (toggled by Step/Continuous" button) this speed setting is applied.
//! In continuous mode the step speed is in percent of max-velocity.
class WhbHandwheelContinuousModeStepSize
{
public:

    enum PositionNameIndex
    {
        RotaryButton2percent   = 0,
        RotaryButton5percent   = 1,
        RotaryButton10percent  = 2,
        RotaryButton30percent  = 3,
        RotaryButton60percent  = 4,
        RotaryButton100percent = 5,
        RotaryButtonUndefined  = 6
    };

    //! Translates the button position to step size in %.
    //! \param buttonPosition
    //! \return the step size in percent ∈ [0, 100]
    uint8_t getStepSize(PositionNameIndex buttonPosition) const;
    WhbHandwheelContinuousModeStepSize();

private:
    const uint8_t mSequence[7];
};

// ----------------------------------------------------------------------

class WhbStepModeStepSize
{
    friend XhcWhb04b6::WhbContext;
    friend XhcWhb04b6::WhbButtonsState;

public:
    WhbStepModeStepSize();

private:
    const WhbHandwheelStepModeStepSize       step;
    const WhbHandwheelContinuousModeStepSize continuous;
};

// ----------------------------------------------------------------------

//! step handling related parameters
class WhbStepHandler
{
    friend XhcWhb04b6::WhbContext;

public:
    const WhbStepModeStepSize stepSize;
    WhbStepHandler();

private:
    unsigned char old_inc_step_status;
};

// ----------------------------------------------------------------------

//! Buttons state struct reflecting i.e. current or previous button state,
//! by storing up to two button codes and axis and feed codes. The software
//! button reflects the logical combination of two button codes if the modifier
//! button code "Fn" is involved.
class WhbButtonsState
{
public:
    //! initialized lookup references are needed to resolve software button state and step speed state
    WhbButtonsState(const WhbButtonsCode& buttonCodesLookup,
                    const WhbAxisRotaryButtonCodes& axisRotaryButtonCodesLookup,
                    const WhbFeedRotaryButtonCodes& feedRotaryButtonCodesLookup,
                    const WhbHandwheelStepModeStepSize& stepSizeLookup,
                    const WhbHandwheelContinuousModeStepSize& continuousStepSizeLookup);
    WhbButtonsState& operator=(const WhbButtonsState& other);

    uint8_t getKeyCode() const;
    uint8_t getModifierCode() const;
    uint8_t getAxisRotaryButtonCode() const;
    uint8_t getFeedRotaryButtonCode() const;
    const WhbSoftwareButton& getSoftwareButton() const;
    //! stores the buttons state and updates the software button state and step speed state
    void updateButtonState(uint8_t keyCode, uint8_t modifierCode, uint8_t currentAxisRotaryButtonCode,
                           uint8_t currentFeedRotaryButtonCode);
    void updateButtonState(const WhbSoftwareButton& softwareButton);
    void setAxisCode(const WhbKeyCode& currentAxis);
    const WhbKeyCode& getAxisCode() const;
    void setFeedCode(const WhbKeyCode& currentFeed);
    const WhbKeyCode& getFeedCode() const;
    //! returns the step size value according to the current rotary step button state
    hal_float_t getStepSize();

    void setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::PositionNameIndex stepSize);
    void setCurrentContinuousModeStepSize(WhbHandwheelContinuousModeStepSize::PositionNameIndex stepSize);
    void setCurrentModeStepMode();
    void setCurrentModeContinuousMode();
    JogWheelStepMode::StepMode currentStepMode() const;
    bool isCurrentModeStepMode() const;
    bool isCurrentModeContinuousMode() const;

private:

    friend XhcWhb04b6::WhbContext;
    const WhbButtonsCode                    & mButtonCodesLookup;
    const WhbAxisRotaryButtonCodes          & mAxisRotaryButtonCodesLookup;
    const WhbFeedRotaryButtonCodes          & mFeedRotaryButtonCodesLookup;
    const WhbHandwheelStepModeStepSize      & mStepModeStepSizeLookup;
    const WhbHandwheelContinuousModeStepSize& mContinuousModeStepSizeLookup;
    WhbHandwheelContinuousModeStepSize::PositionNameIndex mCurrentContinuousModeSize;
    WhbHandwheelStepModeStepSize::PositionNameIndex       mCurrentStepModeSize;
    JogWheelStepMode::StepMode                            mCurrentStepMode;
    uint8_t                                               mCurrentButton1Code;
    uint8_t                                               mCurrentButton2Code;
    const WhbSoftwareButton* mSoftwareButton;
    uint8_t mCurrentAxisCode;
    uint8_t mCurrentFeedCode;
    const WhbKeyCode* mCurrentAxisKeyCode;
    const WhbKeyCode* mCurrentFeedKeyCode;
};

// ----------------------------------------------------------------------

//! Convenience structure for initializing a transmission block.
//! Caution: do not reorder fields!
class WhbUsbOutPackageBlockFields
{
public:
    //! constant 0x06
    uint8_t reportId;
    uint8_t __padding0;
    uint8_t __padding1;
    uint8_t __padding2;
    uint8_t __padding3;
    uint8_t __padding4;
    uint8_t __padding5;
    uint8_t __padding6;
    WhbUsbOutPackageBlockFields();
    void init(const void* data);
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for accessing a block as byte buffer.
class WhbUsbOutPackageBlockBuffer
{
public:
    uint8_t asBytes[sizeof(WhbUsbOutPackageBlockFields)];
} __attribute__((packed));


// ----------------------------------------------------------------------

union WhbUsbOutPackageBlock
{
public:
    WhbUsbOutPackageBlockBuffer asBuffer;
    WhbUsbOutPackageBlockFields asBlock;
    WhbUsbOutPackageBlock();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for initializing a transmission package's blocks.
//! Caution: do not reorder fields!
class WhbUsbOutPackageBlocks
{
public:
    WhbUsbOutPackageBlockFields block0;
    WhbUsbOutPackageBlockFields block1;
    WhbUsbOutPackageBlockFields block2;
    WhbUsbOutPackageBlocks();
    void init(const WhbUsbOutPackageData* data);
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Axis coordinate structure as sent via usb.
//! Caution: do not reorder fields!
class WhbUsbOutPackageAxisCoordinate
{
public:
    uint16_t integerValue;
    uint16_t fractionValue  :15;
    uint16_t coordinateSign :1;
    void setCoordinate(const float& coordinate);
    void clear();
} __attribute__((packed));

// ----------------------------------------------------------------------

class DisplayIndicatorStepMode
{
public:
    //! \see DisplayIndicatorBitFields::stepMode
    enum StepMode
    {
        //! displays "CONT <xx>%"
            CONTINUOUS             = 0x00,
        //! displays "STP: <x.xxxx>"
            STEP                   = 0x01,
        //! displays "MPG <xx>%"
            MANUAL_PULSE_GENERATOR = 0x02
    };
};

// ----------------------------------------------------------------------

//! Caution: do not reorder fields!
class DisplayIndicatorBitFields
{
public:
    //! \see DisplayIndicatorStepMode
    uint8_t stepMode            : 2;
    //! unknown flags
    uint8_t unknown             : 4;
    //! if flag set displays "RESET", \ref stepMode otherwise
    uint8_t isReset             : 1;
    //! if flag set axis names are "X1" "X1" ... "C1", "X" "Y" ... "C" otherwise
    uint8_t isMachineCoordinate : 1;
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Caution: do not reorder fields!
union DisplayIndicator
{
public:
    uint8_t                   asByte;
    DisplayIndicatorBitFields asBitFields;
    DisplayIndicator();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for accessing data fields in output package stream.
//! Caution: do not reorder fields!
class WhbUsbOutPackageData
{
public:
    //! constant: 0xfdfe
    uint16_t                       header;
    uint8_t                        seed;
    DisplayIndicator               displayModeFlags;
    WhbUsbOutPackageAxisCoordinate row1Coordinate;
    WhbUsbOutPackageAxisCoordinate row2Coordinate;
    WhbUsbOutPackageAxisCoordinate row3Coordinate;
    //! printed on feed+/- button pressed
    uint16_t                       feedRate;
    //! printed on spindle+/- button pressed
    uint16_t                       spindleSpeed;
    WhbUsbOutPackageData();
    void clear();

private:
    // TODO: investigate if this is still needed. it was needed when copying data chunks to blocks to avoid invalid read
    uint8_t padding;
} __attribute__((packed));


// ----------------------------------------------------------------------

//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union WhbUsbOutPackageBuffer
{
public:
    WhbUsbOutPackageBlock  asBlockArray[sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlock)];
    WhbUsbOutPackageBlocks asBlocks;
    WhbUsbOutPackageBuffer();
}__attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for accessing data in input package stream.
//! Caution: do not reorder fields!
class WhbUsbInPackage
{
public:
    //! constant 0x04
    const uint8_t header;
    const uint8_t randomByte;
    const uint8_t buttonKeyCode1;
    const uint8_t buttonKeyCode2;
    const uint8_t rotaryButtonFeedKeyCode;
    const uint8_t rotaryButtonAxisKeyCode;
    const int8_t  stepCount;
    const uint8_t crc;
    WhbUsbInPackage();
    WhbUsbInPackage(const uint8_t notAvailable1, const uint8_t notAvailable2, const uint8_t buttonKeyCode1,
                    const uint8_t buttonKeyCode2, const uint8_t rotaryButtonFeedKeyCode,
                    const uint8_t rotaryButtonAxisKeyCode, const int8_t stepCount, const uint8_t crc);
}__attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union WhbUsbInPackageBuffer
{
public:
    const WhbUsbInPackage asFields;
    uint8_t               asBuffer[sizeof(WhbUsbInPackage)];
    WhbUsbInPackageBuffer();
}__attribute__((packed));


// ----------------------------------------------------------------------

//! This package is sent as last but one package before xhc-whb04-6 is powered off,
//! and is meant to be used with operator== for comparison.
class WhbUsbEmptyPackage :
    public WhbUsbInPackage
{
public:

    WhbUsbEmptyPackage();
    //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
    bool operator==(const WhbUsbInPackage& other) const;
    //! \see operator==(const WhbUsbInPackage&)
    bool operator!=(const WhbUsbInPackage& other) const;
} __attribute__((packed));

// ----------------------------------------------------------------------

//! This package is sent as last package before xhc-whb04-6 is powered off,
//! and is meant to be used with operator== for comparison.
class WhbUsbSleepPackage :
    public WhbUsbInPackage
{
public:
    WhbUsbSleepPackage();
    //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
    bool operator==(const WhbUsbInPackage& other) const;
    //! \see operator==(const WhbUsbInPackage&)
    bool operator!=(const WhbUsbInPackage& other) const;
} __attribute__((packed));

// ----------------------------------------------------------------------

//! set of constant usb packages
class WhbConstantUsbPackages
{
public:
    const WhbUsbSleepPackage sleepPackage;
    const WhbUsbEmptyPackage emptyPackage;
    WhbConstantUsbPackages();
};

// ----------------------------------------------------------------------

//! USB related parameters
class WhbUsb
{
public:
    static const WhbConstantUsbPackages ConstantPackages;
    //! \param name device string used for printing messages
    //! \param onDataReceivedCallback called when received data is ready
    WhbUsb(const char* name, UsbInputPackageListener& onDataReceivedCallback);
    ~WhbUsb();
    uint16_t getUsbVendorId() const;
    uint16_t getUsbProductId() const;
    const bool isDeviceOpen() const;
    libusb_context** getContextReference();
    libusb_context* getContext();
    void setContext(libusb_context* context);
    libusb_device_handle* getDeviceHandle();
    void setDeviceHandle(libusb_device_handle* deviceHandle);
    bool isWaitForPendantBeforeHalEnabled() const;
    bool getDoReconnect() const;
    void setDoReconnect(bool doReconnect);
    void onUsbDataReceived(struct libusb_transfer* transfer);
    void setSimulationMode(bool isSimulationMode);
    void setIsRunning(bool enableRunning);
    void requestTermination();
    //! Do offer a HAL memory before calling this method.
    //! \sa takeHalMemoryReference(WhbHalMemory *)
    bool setupAsyncTransfer();
    void sendDisplayData();
    void enableVerboseTx(bool enable);
    void enableVerboseRx(bool enable);
    void enableVerboseInit(bool enable);
    bool init();
    void setWaitWithTimeout(uint8_t waitSecs);
    const WhbUsbOutPackageData& getOutputPackageData();
    void takeHalMemoryReference(WhbHalMemory* memory);

private:
    const uint16_t usbVendorId;
    const uint16_t usbProductId;
    libusb_context      * context;
    libusb_device_handle* deviceHandle;
    bool                   do_reconnect;
    bool                   isWaitWithTimeout;
    bool                   mIsSimulationMode;
    WhbSleepDetect         sleepState;
    bool                   mIsRunning;
    WhbUsbInPackageBuffer  inputPackageBuffer;
    WhbUsbOutPackageBuffer outputPackageBuffer;
    WhbUsbOutPackageData   outputPackageData;
    UsbInputPackageListener& mDataHandler;
    WhbHalMemory           * mHalMemory;
    struct libusb_transfer * inTransfer;
    struct libusb_transfer * outTransfer;
    std::ostream devNull;
    std::ostream* verboseTxOut;
    std::ostream* verboseRxOut;
    std::ostream* verboseInitOut;
    const char  * mName;
    uint8_t mWaitSecs;
};

// ----------------------------------------------------------------------

const WhbConstantUsbPackages WhbUsb::ConstantPackages;

// ----------------------------------------------------------------------

class UsbInputPackageListener
{
public:
    virtual void onInputDataReceived(const WhbUsbInPackage& inPackage) = 0;

    virtual ~UsbInputPackageListener()
    {
    }
};

class WhbKeyEventListener
{
public:
    //! Called when button is pressed.
    //! \param softwareButton the button pressed
    //! \return true if a subsequent re-evaluation should be performed.
    //! Example: A button event changes the feed rotary buttons step mode from
    //! step to continuous. The button must be re-evaluated, otherwise the
    //! button state remains untouched until the next button's event.
    virtual bool onButtonPressedEvent(const WhbSoftwareButton& softwareButton) = 0;
    //! Called when button is released.
    //! \param softwareButton the button released
    //! \return true if a subsequent re-evaluation should be performed.
    //! Example: A button event changes the feed rotary buttons step mode from
    //! step to continuous. The button must be re-evaluated, otherwise the
    //! button state remains untouched until the next button's event.
    virtual bool onButtonReleasedEvent(const WhbSoftwareButton& softwareButton) = 0;
    virtual void onAxisActiveEvent(const WhbKeyCode& axis) = 0;
    virtual void onAxisInactiveEvent(const WhbKeyCode& axis) = 0;
    virtual void onFeedActiveEvent(const WhbKeyCode& axis) = 0;
    virtual void onFeedInactiveEvent(const WhbKeyCode& axis) = 0;
    virtual void onJogDialEvent(int8_t delta) = 0;
    virtual ~WhbKeyEventListener();
};

class UsbInputPackageInterpreted
{
public:
    virtual void onDataInterpreted() = 0;
    virtual ~UsbInputPackageInterpreted();
};

// ----------------------------------------------------------------------

//! program context
class WhbContext :
    public UsbInputPackageListener, public WhbKeyEventListener, public UsbInputPackageInterpreted
{
public:
    WhbContext();
    virtual ~WhbContext();
    void process();
    void teardownUsb();
    void setUsbContext(libusb_context* context);
    libusb_device_handle* getUsbDeviceHandle();
    libusb_context* getUsbContext();
    //! \return the name as specified to \ref WhbContext
    const char* getName() const;
    //! \return the name as specified to \ref hal
    const char* getHalName() const;
    //! callback method received by \ref WhbUsb when a \ref libusb_transfer is received
    void onInputDataReceived(const WhbUsbInPackage& inPackage) override;
    size_t getSoftwareButtonIndex(uint8_t keyCode) const;
    void initWhb();
    void initHal();
    void teardownHal();
    void onUsbDataReceivedCallback(struct libusb_transfer* transfer);
    bool enableReceiveAsyncTransfer();
    void sendDisplayData();
    void linuxcncSimulate();
    void requestTermination(int signal = -42);
    bool isRunning() const;
    int run();
    bool isSimulationModeEnabled() const;
    void setSimulationMode(bool enableSimulationMode);
    void setEnableVerboseKeyEvents(bool enable);
    void enableVerboseRx(bool enable);
    void enableVerboseTx(bool enable);
    void enableVerboseHal(bool enable);
    void enableVerboseInit(bool enable);
    void enableCrcDebugging(bool enable);
    void setWaitWithTimeout(uint8_t waitSecs = 3);
    bool onButtonPressedEvent(const WhbSoftwareButton& softwareButton) override;
    bool onButtonReleasedEvent(const WhbSoftwareButton& softwareButton) override;
    void onAxisActiveEvent(const WhbKeyCode& axis) override;
    void onAxisInactiveEvent(const WhbKeyCode& axis) override;
    void onDataInterpreted() override;
    void onFeedActiveEvent(const WhbKeyCode& axis) override;
    void onFeedInactiveEvent(const WhbKeyCode& axis) override;
    void onJogDialEvent(int8_t delta) override;
    void updateAxisRotaryButton(const WhbUsbInPackage& inPackage);
    //! update all buttons' state to hal and detect button pressed/released event
    //! \param inPackage input package to interpret
    //! \param keyCode pressed button
    //! \param modifierCode Optional pressed modifier button. Wsually "Fn", but could be any button.
    //! \return \ref WhbKeyEventListener::onButtonPressedEvent
    bool updateHalButtons(const WhbUsbInPackage& inPackage, uint8_t keyCode, uint8_t modifierCode);
    void updateJogDial(const WhbUsbInPackage& inPackage);
    void updateStepRotaryButton(const WhbUsbInPackage& inPackage, bool forceEvents = false);
    void printCrcDebug(const WhbUsbInPackage& inPackage, const WhbUsbOutPackageData& outPackageBuffer) const;
    size_t getHalPinNumber(const WhbSoftwareButton& button);
    void offerHalMemory();

private:
    const char* mName;
    WhbHal                                 mHal;
    const WhbKeyCodes                      mKeyCodes;
    const WhbStepHandler                   mStepHandler;
    const WhbSoftwareButton                mSoftwareButtons[32];
    WhbUsb                                 mUsb;
    bool                                   mIsRunning;
    bool                                   mIsSimulationMode;
    WhbButtonsState                        mPreviousButtonCodes;
    WhbButtonsState                        mCurrentButtonCodes;
    std::ostream                           mDevNull;
    std::ostream              * mTxCout;
    std::ostream              * mRxCout;
    std::ostream              * mKeyEventCout;
    std::ostream              * mHalInitCout;
    std::ostream              * mInitCout;
    WhbKeyEventListener       & keyEventReceiver;
    UsbInputPackageListener   & packageReceivedEventReceiver;
    UsbInputPackageInterpreted& packageIntepretedEventReceiver;
    bool mIsCrcDebuggingEnabled;
    //! prints human readable output of the push buttons state
    void printPushButtonText(uint8_t keyCode, uint8_t modifierCode, std::ostream& out);
    //! prints human readable output of the push buttons state to \ref verboseRxOut stream
    void printPushButtonText(uint8_t keyCode, uint8_t modifierCode);
    //! prints human readable output of the push buttons state to \p out
    void printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode, std::ostream& out);
    //! prints human readable output of the rotary button text to \ref verboseRxOut stream
    void printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode);
    //! prints human readable output of the rotary button text to \p out
    void printInputData(const WhbUsbInPackage& inPackage, std::ostream& out);
    //! prints human readable output of the input package to \ref verboseRxOut stream
    void printInputData(const WhbUsbInPackage& inPackage);
    //! prints human readable output of the input package to \p out
    void printHexdump(const WhbUsbInPackage& inPackage, std::ostream& out);
    //! prints a hexdump of the input package to \ref verboseRxOut stream
    void printHexdump(const WhbUsbInPackage& inPackage);
    bool dispatchButtonEventToHal(const WhbSoftwareButton& softwareButton, bool isButtonPressed);
    void dispatchAxisEventToHal(const WhbKeyCode& axis, bool isActive);
};

// ----------------------------------------------------------------------

UsbInputPackageInterpreted::~UsbInputPackageInterpreted()
{
}

// ----------------------------------------------------------------------

WhbKeyEventListener::~WhbKeyEventListener()
{
}

// ----------------------------------------------------------------------

WhbUsbOutPackageBlock::WhbUsbOutPackageBlock() :
    asBlock()
{
    assert(sizeof(WhbUsbOutPackageBlockBuffer) == sizeof(WhbUsbOutPackageBlockFields));
}

// ----------------------------------------------------------------------

WhbSleepDetect::WhbSleepDetect() :
    mDropNextInPackage(false),
    mLastWakeupTimestamp()
{
}

// ----------------------------------------------------------------------

WhbKeyCode::WhbKeyCode(uint8_t code, const char* text, const char* altText) :
    code(code),
    text(text),
    altText(altText)
{
}

// ----------------------------------------------------------------------

WhbKeyCode::WhbKeyCode(const WhbKeyCode& other) :
    code(other.code),
    text(other.text),
    altText(other.altText)
{
}

// ----------------------------------------------------------------------

bool WhbKeyCode::operator==(const WhbKeyCode& other) const
{
    return ((code == other.code) && (text == other.text) && (altText == other.altText));
}

// ----------------------------------------------------------------------

bool WhbKeyCode::operator!=(const WhbKeyCode& other) const
{
    return !(*this == other);
}

// ----------------------------------------------------------------------

WhbSoftwareButton::WhbSoftwareButton(const WhbKeyCode& key, const WhbKeyCode& modifier) :
    key(key),
    modifier(modifier)
{
}

bool WhbSoftwareButton::containsKeys(const WhbKeyCode& key, const WhbKeyCode& modifier) const
{
    return ((this->key.code == key.code) && (this->modifier.code == modifier.code));
}

// ----------------------------------------------------------------------

WhbAxisRotaryButtonCodes::WhbAxisRotaryButtonCodes() :
    off(0x06, "OFF", ""),
    x(0x11, "X", ""),
    y(0x12, "Y", ""),
    z(0x13, "Z", ""),
    a(0x14, "A", ""),
    b(0x15, "B", ""),
    c(0x16, "C", ""),
    undefined(0x00, "", "")
{
}

// ----------------------------------------------------------------------

WhbFeedRotaryButtonCodes::WhbFeedRotaryButtonCodes() :
    speed_0_001(0x0d, "0.001", "2%"),
    speed_0_01(0x0e, "0.01", "5%"),
    speed_0_1(0x0f, "0.1", "10%"),
    speed_1(0x10, "1", "30%"),
    percent_60(0x1a, "", "60%"),
    percent_100(0x1b, "", "100%"),
    lead(0x1c, "Lead", ""),
    undefined(0x00, "", "")
{
}

// ----------------------------------------------------------------------

WhbButtonsCode::WhbButtonsCode() :
    reset(0x01, "reset", "macro-11"),
    stop(0x02, "stop", "macro-12"),
    start(0x03, "start-pause", "macro-13"),
    feed_plus(0x04, "feed-plus", "macro-1"),
    feed_minus(0x05, "feed-minus", "macro-2"),
    spindle_plus(0x06, "spindle-plus", "macro-3"),
    spindle_minus(0x07, "spindle-minus", "macro-4"),
    machine_home(0x08, "m-home", "macro-5"),
    safe_z(0x09, "safe-z", "macro-6"),
    workpiece_home(0x0a, "w-home", "macro-7"),
    spindle_on_off(0x0b, "s-on-off", "macro-8"),
    function(0x0c, "fn", "<unused>"),
    probe_z(0x0d, "probe-z", "macro-9"),
    macro10(0x10, "macro-10", "macro-14"),
    manual_pulse_generator(0x0e, "mode-continuous", "macro-15"),
    step_continuous(0x0f, "mode-step", "macro-16"),
    undefined(0x00, "", "")
{
}

// ----------------------------------------------------------------------

const WhbKeyCode& WhbButtonsCode::getKeyCode(uint8_t keyCode) const
{
    const WhbKeyCode* whbKeyCode = reinterpret_cast<const WhbKeyCode*>(this);

    while (whbKeyCode->code != 0)
    {
        if (whbKeyCode->code == keyCode)
        {
            break;
        }
        whbKeyCode++;
    }

    return *whbKeyCode;
}

// ----------------------------------------------------------------------

WhbKeyCodes::WhbKeyCodes() :
    buttons(),
    axis(),
    feed()
{
}

// ----------------------------------------------------------------------

WhbVelocityComputation::WhbVelocityComputation() :
    last_jog_counts(-1),
    last_tv()
{
}

// ----------------------------------------------------------------------

float WhbHandwheelStepModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[buttonPosition];
}

// ----------------------------------------------------------------------

WhbHandwheelStepModeStepSize::WhbHandwheelStepModeStepSize() :
    mSequence{0.001, 0.01, 0.1, 1.0, 0.0}
{
}

// ----------------------------------------------------------------------

uint8_t WhbHandwheelContinuousModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[buttonPosition];
}

// ----------------------------------------------------------------------

WhbHandwheelContinuousModeStepSize::WhbHandwheelContinuousModeStepSize() :
    mSequence{2, 5, 10, 30, 60, 100, 0}
{
}

// ----------------------------------------------------------------------

WhbStepModeStepSize::WhbStepModeStepSize() :
    step(),
    continuous()
{
}

// ----------------------------------------------------------------------

WhbStepHandler::WhbStepHandler() :
    stepSize(),
    old_inc_step_status(0)
{
}

// ----------------------------------------------------------------------

JogWheelStepMode::StepMode WhbButtonsState::currentStepMode() const
{
    return mCurrentStepMode;
}

// ----------------------------------------------------------------------

bool WhbButtonsState::isCurrentModeStepMode() const
{
    return mCurrentStepMode == JogWheelStepMode::STEP;
}

// ----------------------------------------------------------------------

bool WhbButtonsState::isCurrentModeContinuousMode() const
{
    return mCurrentStepMode == JogWheelStepMode::CONTINUOUS;
}

// ----------------------------------------------------------------------

void WhbButtonsState::setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::PositionNameIndex stepSize)
{
    mCurrentStepMode     = JogWheelStepMode::STEP;
    mCurrentStepModeSize = stepSize;
}

// ----------------------------------------------------------------------

void
WhbButtonsState::setCurrentContinuousModeStepSize(WhbHandwheelContinuousModeStepSize::PositionNameIndex stepSize)
{
    mCurrentStepMode           = JogWheelStepMode::CONTINUOUS;
    mCurrentContinuousModeSize = stepSize;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getKeyCode() const
{
    return mCurrentButton1Code;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getModifierCode() const
{
    return mCurrentButton2Code;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getAxisRotaryButtonCode() const
{
    // TODO: must return type WhbKeyCode
    return mCurrentAxisCode;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getFeedRotaryButtonCode() const
{
    // TODO: must return type WhbKeyCode
    return mCurrentFeedCode;
}

// ----------------------------------------------------------------------
/*
const WhbSoftwareButton& WhbButtonsState::getSoftwareButton() const
{
    assert(mSoftwareButton != nullptr);
    return *mSoftwareButton;
}
*/
// ----------------------------------------------------------------------

void WhbButtonsState::updateButtonState(uint8_t keyCode, uint8_t modifierCode, uint8_t currentAxisRotaryButtonCode,
                                        uint8_t currentFeedRotaryButtonCode)
{
    mCurrentButton1Code = keyCode;
    mCurrentButton2Code = modifierCode;
    mCurrentAxisCode    = currentAxisRotaryButtonCode;
    mCurrentFeedCode    = currentFeedRotaryButtonCode;

    // TODO: update step mode a. axis
}

// ----------------------------------------------------------------------

hal_float_t WhbButtonsState::getStepSize()
{
    // TODO: remove hotfix
    float maxVelocity_mm_sec = 40.0;
    float stepsPerUnit_mm    = 200.0 / 5.0;

    if (mCurrentStepMode == JogWheelStepMode::CONTINUOUS)
    {
        // transform step size into percental of maximum-velocity
        return maxVelocity_mm_sec * (0.01 * mContinuousModeStepSizeLookup.getStepSize(mCurrentContinuousModeSize));
    }
    else if (mCurrentStepMode == JogWheelStepMode::STEP)
    {
        // transform step size to metric units
        return stepsPerUnit_mm * mStepModeStepSizeLookup.getStepSize(mCurrentStepModeSize);
    }
    else
    {
        assert(false);
    }
}

// ----------------------------------------------------------------------

WhbButtonsState::WhbButtonsState(const WhbButtonsCode& buttonCodesLookup,
                                 const WhbAxisRotaryButtonCodes& axisRotaryButtonCodesLookup,
                                 const WhbFeedRotaryButtonCodes& feedRotaryButtonCodesLookup,
                                 const WhbHandwheelStepModeStepSize& stepSizeLookup,
                                 const WhbHandwheelContinuousModeStepSize& continuousStepSizeLookup) :
    mButtonCodesLookup(buttonCodesLookup),
    mAxisRotaryButtonCodesLookup(axisRotaryButtonCodesLookup),
    mFeedRotaryButtonCodesLookup(feedRotaryButtonCodesLookup),
    mStepModeStepSizeLookup(stepSizeLookup),
    mContinuousModeStepSizeLookup(continuousStepSizeLookup),
    mCurrentContinuousModeSize(WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButtonUndefined),
    mCurrentStepModeSize(WhbHandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined),
    mCurrentStepMode(JogWheelStepMode::CONTINUOUS),
    mCurrentButton1Code(buttonCodesLookup.undefined.code),
    mCurrentButton2Code(buttonCodesLookup.undefined.code),
    mSoftwareButton(nullptr),
    mCurrentAxisCode(axisRotaryButtonCodesLookup.undefined.code),
    mCurrentFeedCode(feedRotaryButtonCodesLookup.undefined.code),
    mCurrentAxisKeyCode(&mAxisRotaryButtonCodesLookup.undefined),
    mCurrentFeedKeyCode(&mFeedRotaryButtonCodesLookup.undefined)
{
}

// ----------------------------------------------------------------------

WhbButtonsState& WhbButtonsState::operator=(const WhbButtonsState& other)
{
    mCurrentContinuousModeSize = other.mCurrentContinuousModeSize;
    mCurrentStepModeSize       = other.mCurrentStepModeSize;
    mCurrentButton1Code        = other.mCurrentButton1Code;
    mCurrentButton2Code        = other.mCurrentButton2Code;
    mSoftwareButton            = other.mSoftwareButton;
    mCurrentAxisCode           = other.mCurrentAxisCode;
    mCurrentFeedCode           = other.mCurrentFeedCode;
    mCurrentAxisKeyCode        = other.mCurrentAxisKeyCode;
    mCurrentFeedKeyCode        = other.mCurrentFeedKeyCode;

    return *this;
}

// ----------------------------------------------------------------------

void WhbButtonsState::updateButtonState(const WhbSoftwareButton& softwareButton)
{
    mSoftwareButton = &softwareButton;

    if (softwareButton.key.code == mButtonCodesLookup.manual_pulse_generator.code &&
        softwareButton.modifier == mButtonCodesLookup.undefined)
    {
        mCurrentStepMode = JogWheelStepMode::CONTINUOUS;
    }
    else if (softwareButton.key.code == mButtonCodesLookup.step_continuous.code &&
             softwareButton.modifier == mButtonCodesLookup.undefined)
    {
        mCurrentStepMode = JogWheelStepMode::STEP;
    }
}

// ----------------------------------------------------------------------

void WhbButtonsState::setAxisCode(const WhbKeyCode& currentAxis)
{
    mCurrentAxisKeyCode = &currentAxis;
}

// ----------------------------------------------------------------------

void WhbButtonsState::setFeedCode(const WhbKeyCode& currentFeed)
{
    mCurrentFeedKeyCode = &currentFeed;
}

// ----------------------------------------------------------------------

const WhbKeyCode& WhbButtonsState::getAxisCode() const
{
    return *mCurrentAxisKeyCode;
}

// ----------------------------------------------------------------------

const WhbKeyCode& WhbButtonsState::getFeedCode() const
{
    return *mCurrentFeedKeyCode;
}

// ----------------------------------------------------------------------

void WhbButtonsState::setCurrentModeStepMode()
{
    mCurrentStepMode = JogWheelStepMode::STEP;
}

// ----------------------------------------------------------------------

void WhbButtonsState::setCurrentModeContinuousMode()
{
    mCurrentStepMode = JogWheelStepMode::CONTINUOUS;
}

// ----------------------------------------------------------------------

WhbUsbInPackageBuffer::WhbUsbInPackageBuffer() :
    asBuffer{0}
{
    assert(sizeof(asFields) == sizeof(asBuffer));
}

// ----------------------------------------------------------------------

WhbUsbEmptyPackage::WhbUsbEmptyPackage() :
    WhbUsbInPackage(0x04, 0xff, 0, 0, 0, 0, 0, 0xff)
{
}

// ----------------------------------------------------------------------

bool WhbUsbEmptyPackage::operator==(const WhbUsbInPackage& other) const
{
    // equality constraints: 0x4 0x? 0x0 0x0 0x0 0x0 0x0 0x?
    if ((header == other.header)
        // && (notAvailable2 == other.notAvailable2)
        && (buttonKeyCode1 == other.buttonKeyCode1) && (buttonKeyCode2 == other.buttonKeyCode2) &&
        (rotaryButtonFeedKeyCode == other.rotaryButtonFeedKeyCode) &&
        (rotaryButtonAxisKeyCode == other.rotaryButtonAxisKeyCode) && (stepCount == other.stepCount)
        // && (crc == other.crc)
        )
    {
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------

bool WhbUsbEmptyPackage::operator!=(const WhbUsbInPackage& other) const
{
    return !((*this) == other);
}

// ----------------------------------------------------------------------

WhbUsbSleepPackage::WhbUsbSleepPackage() :
    WhbUsbInPackage(0x04, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff)
{
}

// ----------------------------------------------------------------------

//! caution: it is not guaranteed that (this == \p other) == (\p other == this)
bool WhbUsbSleepPackage::operator==(const WhbUsbInPackage& other) const
{
    // equality constraints: 0x4 0x? 0x? 0x? 0x? 0x? 0x? 0x?
    if ((header == other.header)
        // && (notAvailable2 == other.notAvailable2)
        // && (buttonKeyCode1 == other.buttonKeyCode1)
        // && (buttonKeyCode2 == other.buttonKeyCode2)
        // && (rotaryButtonFeedKeyCode == other.rotaryButtonFeedKeyCode)
        // && (rotaryButtonAxisKeyCode == other.rotaryButtonAxisKeyCode)
        // && (stepCount == other.stepCount)
        // && (crc == other.crc)
        )
    {
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------

//! \see operator==(const WhbUsbInPackage&)
bool WhbUsbSleepPackage::operator!=(const WhbUsbInPackage& other) const
{
    return !((*this) == other);
}

// ----------------------------------------------------------------------

WhbConstantUsbPackages::WhbConstantUsbPackages() :
    sleepPackage(),
    emptyPackage()
{
}

// ----------------------------------------------------------------------

uint16_t WhbUsb::getUsbVendorId() const
{
    return usbVendorId;
}

// ----------------------------------------------------------------------

uint16_t WhbUsb::getUsbProductId() const
{
    return usbProductId;
}

// ----------------------------------------------------------------------


const bool WhbUsb::isDeviceOpen() const
{
    return deviceHandle != nullptr;
}

// ----------------------------------------------------------------------

libusb_context** WhbUsb::getContextReference()
{
    return &context;
}

// ----------------------------------------------------------------------

libusb_context* WhbUsb::getContext()
{
    return context;
}

// ----------------------------------------------------------------------

void WhbUsb::setContext(libusb_context* context)
{
    this->context = context;
}

// ----------------------------------------------------------------------

libusb_device_handle* WhbUsb::getDeviceHandle()
{
    return deviceHandle;
}

// ----------------------------------------------------------------------


void WhbUsb::setDeviceHandle(libusb_device_handle* deviceHandle)
{
    this->deviceHandle = deviceHandle;
}

// ----------------------------------------------------------------------

bool WhbUsb::isWaitForPendantBeforeHalEnabled() const
{
    return isWaitWithTimeout;
}

// ----------------------------------------------------------------------

bool WhbUsb::getDoReconnect() const
{
    return do_reconnect;
}

// ----------------------------------------------------------------------

void WhbUsb::setDoReconnect(bool doReconnect)
{
    this->do_reconnect = doReconnect;
}

// ----------------------------------------------------------------------

WhbUsb::WhbUsb(const char* name, UsbInputPackageListener& onDataReceivedCallback) :
// usbVendorId(0x10ce), // xhc-whb04-4
// usbProductId(0xeb70),// xhc-whb04-4
    usbVendorId(0x10ce), // xhc-whb04-6
    usbProductId(0xeb93), // xhc-whb04-6
    context(nullptr),
    deviceHandle(nullptr),
    do_reconnect(false),
    isWaitWithTimeout(false),
    mIsSimulationMode(false),
    sleepState(),
    mIsRunning(false),
    inputPackageBuffer(),
    outputPackageBuffer(),
    mDataHandler(onDataReceivedCallback),
    mHalMemory(nullptr),
    inTransfer(libusb_alloc_transfer(0)),
    outTransfer(libusb_alloc_transfer(0)),
    devNull(nullptr),
    verboseTxOut(&devNull),
    verboseRxOut(&devNull),
    verboseInitOut(&devNull),
    mName(name),
    mWaitSecs(0)
{
    gettimeofday(&sleepState.mLastWakeupTimestamp, nullptr);
}

// ----------------------------------------------------------------------

void WhbUsb::sendDisplayData()
{
    outputPackageBuffer.asBlocks.init(&outputPackageData);

    static uint16_t u = 0;
    outputPackageData.row1Coordinate.setCoordinate(++u);

    if (mIsSimulationMode)
    {
        *verboseTxOut << "out   0x" << outputPackageBuffer.asBlocks << endl <<
                      dec << "out   size " << sizeof(outputPackageBuffer.asBlockArray) << "B " << outputPackageData
                      << endl;
    }

    for (size_t idx = 0; idx < (sizeof(outputPackageBuffer.asBlockArray) / sizeof(WhbUsbOutPackageBlockFields)); idx++)
    {
        WhbUsbOutPackageBlock& block = outputPackageBuffer.asBlockArray[idx];
        size_t blockSize = sizeof(WhbUsbOutPackageBlock);
        // see also
        // http://www.beyondlogic.org/usbnutshell/usb6.shtml
        // http://libusb.sourceforge.net/api-1.0/group__desc.html
        // http://libusb.sourceforge.net/api-1.0/group__misc.html
        int    r         = libusb_control_transfer(deviceHandle,
            // send to hid descriptor: bmRequestType == LIBUSB_DT_HID == 0x21 == (iterface | endpoint)
                                                   LIBUSB_DT_HID,
            // bRequest == LIBUSB_REQUEST_SET_CONFIGURATION == 0x09 == set configuration
                                                   LIBUSB_REQUEST_SET_CONFIGURATION,
            // wValue: if bRequest == LIBUSB_REQUEST_SET_CONFIGURATION the configuration value
                                                   0x0306,
            // wIndex, device interface number
                                                   0x00,
            // data to transmit
                                                   block.asBuffer.asBytes,
            // wLength, data length
                                                   blockSize,
            // transfer timeout[ms]
                                                   0);

        if (r < 0)
        {
            cerr << "transmission failed, try to reconnect ..." << endl;
            setDoReconnect(true);
            return;
        }
    }
}

// ----------------------------------------------------------------------

void WhbContext::initHal()
{
    mHal.init(mSoftwareButtons, mKeyCodes);
}

// ----------------------------------------------------------------------

void WhbContext::teardownHal()
{
    hal_exit(mHal.getHalComponentId());
}

// ----------------------------------------------------------------------

const char* WhbContext::getName() const
{
    return mName;
}

// ----------------------------------------------------------------------

const char* WhbContext::getHalName() const
{
    return mHal.getHalComponentName();
}

// ----------------------------------------------------------------------

void WhbContext::printCrcDebug(const WhbUsbInPackage& inPackage, const WhbUsbOutPackageData& outPackageBuffer) const
{
    ios init(NULL);
    init.copyfmt(*mRxCout);
    *mRxCout << setfill('0') << std::hex;

    // calculate checksum on button released, jog dial or rotary button change
    if (inPackage.buttonKeyCode1 == mKeyCodes.buttons.undefined.code)
    {
        bool isValid = (inPackage.crc == (inPackage.randomByte & outPackageBuffer.seed));

        if (isValid)
        {
            if (mIsCrcDebuggingEnabled)
            {
                *mRxCout << "crc   checksum OK";
            }
        }
        else
        {
            *mRxCout << "warn  checksum error" << endl;
        }
        mRxCout->copyfmt(init);
        return;
    }

    std::bitset<8> random(inPackage.randomByte), buttonKeyCode(inPackage.buttonKeyCode1), crc(inPackage.crc);
    std::bitset<8> delta(0);

    if (inPackage.randomByte > inPackage.crc)
    {
        delta = inPackage.randomByte - inPackage.crc;
    }
    else
    {
        delta = inPackage.crc - inPackage.randomByte;
    }
    delta                        = inPackage.randomByte - inPackage.crc;

    if (mIsCrcDebuggingEnabled)
    {
        *mRxCout << endl;
        *mRxCout << "0x key " << setw(8) << static_cast<unsigned short>(inPackage.buttonKeyCode1)
                 << " random " << setw(8) << static_cast<unsigned short>(inPackage.randomByte)
                 << " crc " << setw(8) << static_cast<unsigned short>(inPackage.crc)
                 << " delta " << setw(8) << static_cast<unsigned short>(delta.to_ulong()) << endl;

        *mRxCout << "0b key " << buttonKeyCode
                 << " random " << random
                 << " crc " << crc
                 << " delta " << delta << endl;
    }

    //! \brief On button pressed checksum calculation.
    //! Experimental: checksum generator not clear yet, but this is a good starting point.
    //! The implementation has several flaws, but works with seed 0xfe and 0xff (which is a bad seed).
    //! \sa WhbUsbOutPackageData::seed
    //! The checksum implementation does not work reliable with other seeds.
    //! TODO: implement me correctly
    std::bitset<8> seed(outPackageBuffer.seed), nonSeed(~seed);
    std::bitset<8> nonSeedAndRandom(nonSeed & random);
    std::bitset<8> keyXorNonSeedAndRandom(buttonKeyCode ^ nonSeedAndRandom);
    uint16_t       expectedCrc   = static_cast<unsigned short>(inPackage.crc);
    uint16_t       calculatedCrc = static_cast<unsigned short>(0x00ff &
                                                               (random.to_ulong() - keyXorNonSeedAndRandom.to_ulong()));
    std::bitset<8> calculatedCrcBitset(calculatedCrc);
    bool           isValid       = (calculatedCrc == expectedCrc);

    if (mIsCrcDebuggingEnabled)
    {
        *mRxCout << endl
                 << "~seed                  " << nonSeed << endl
                 << "random                 " << random << endl
                 << "                       -------- &" << endl
                 << "~seed & random         " << nonSeedAndRandom << endl
                 << "key                    " << buttonKeyCode << endl
                 << "                       -------- ^" << endl
                 << "key ^ (~seed & random) " << keyXorNonSeedAndRandom
                 << " = calculated delta " << setw(2) << static_cast<unsigned short>(keyXorNonSeedAndRandom.to_ulong())
                 << " vs " << setw(2) << static_cast<unsigned short>(delta.to_ulong())
                 << ((keyXorNonSeedAndRandom == delta) ? " OK" : " FAIL") << endl
                 << "calculated crc         " << calculatedCrcBitset << " " << setw(2) << calculatedCrc << " vs "
                 << setw(2)
                 << expectedCrc << ((isValid) ? "                    OK" : "                    FAIL")
                 << " (random - (key ^ (~seed & random))";
    }

    if (!isValid)
    {
        *mRxCout << "warn  checksum error";
    }

    //assert(calculatedCrc == expectedCrc);
    //assert(keyXorNonSeedAndRandom == delta);

    mRxCout->copyfmt(init);
}

// ----------------------------------------------------------------------

//! interprets data packages as delivered by the XHC WHB04B-6 device
void WhbContext::onInputDataReceived(const WhbUsbInPackage& inPackage)
{
    if (mIsSimulationMode)
    {
        *mRxCout << "in    ";
        printHexdump(inPackage);
        if (inPackage.rotaryButtonFeedKeyCode != 0)
        {
            ios init(NULL);
            init.copyfmt(*mRxCout);
            *mRxCout << " delta " << setfill(' ') << setw(2)
                     << static_cast<unsigned short>(inPackage.rotaryButtonFeedKeyCode);
            mRxCout->copyfmt(init);
        }
        else
        {
            *mRxCout << " delta NA";
        }
        *mRxCout << " => ";
        printInputData(inPackage);
        printCrcDebug(inPackage, mUsb.getOutputPackageData());
        *mRxCout << endl;
    }

    uint8_t modifierCode = mKeyCodes.buttons.undefined.code;
    uint8_t keyCode      = mKeyCodes.buttons.undefined.code;

    //! found modifier at key one, key two is the key
    if (inPackage.buttonKeyCode1 == mKeyCodes.buttons.function.code)
    {
        modifierCode = mKeyCodes.buttons.function.code;
        keyCode      = inPackage.buttonKeyCode2;
    }
        //! found modifier at key two, key one is the key
    else if (inPackage.buttonKeyCode2 == mKeyCodes.buttons.function.code)
    {
        modifierCode = mKeyCodes.buttons.function.code;
        keyCode      = inPackage.buttonKeyCode1;
    }
        //! no modifier, key one and key two are defined, fallback to key two which is the lastly one pressed
    else if (inPackage.buttonKeyCode2 != mKeyCodes.buttons.undefined.code)
    {
        keyCode = inPackage.buttonKeyCode2;
    }
        //! fallback to whatever key one is
    else
    {
        keyCode = inPackage.buttonKeyCode1;
    }

    //! update previous and current button state
    mPreviousButtonCodes = mCurrentButtonCodes;
    mCurrentButtonCodes.updateButtonState(keyCode, modifierCode, inPackage.rotaryButtonAxisKeyCode,
                                          inPackage.rotaryButtonFeedKeyCode);

    bool forceUpdate = updateHalButtons(inPackage, keyCode, modifierCode);
    updateAxisRotaryButton(inPackage);
    updateStepRotaryButton(inPackage, forceUpdate);
    updateJogDial(inPackage);
    onDataInterpreted();
}

// ----------------------------------------------------------------------

void WhbContext::initWhb()
{
    //stepHandler.old_inc_step_status = -1;
    //gettimeofday(&sleepState.mLastWakeupTimestamp, nullptr);
    mIsRunning = true;
    mUsb.setIsRunning(true);
}

// ----------------------------------------------------------------------

void WhbContext::requestTermination(int signal)
{
    if (signal >= 0)
    {
        *mInitCout << "termination requested upon signal number " << signal << " ..." << endl;
    }
    else
    {
        *mInitCout << "termination requested ... " << endl;
    }
    mUsb.requestTermination();
    mIsRunning = false;
}

// ----------------------------------------------------------------------

bool WhbContext::isRunning() const
{
    return mIsRunning;
}

// ----------------------------------------------------------------------

WhbContext::WhbContext() :
    mName("XHC-WHB04B-6"),
    mHal(),
    mKeyCodes(),
    mStepHandler(),
    mSoftwareButtons{WhbSoftwareButton(mKeyCodes.buttons.reset, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.reset, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.stop, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.stop, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.start, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.start, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.feed_plus, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.feed_plus, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.feed_minus, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.feed_minus, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_plus, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_plus, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_minus, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_minus, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.machine_home, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.machine_home, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.safe_z, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.safe_z, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.workpiece_home, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.workpiece_home, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_on_off, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.spindle_on_off, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.function, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.probe_z, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.probe_z, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.macro10, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.macro10, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.manual_pulse_generator, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.manual_pulse_generator, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.step_continuous, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.step_continuous, mKeyCodes.buttons.function),
        //! it is expected to terminate this array with the "undefined" software button
                     WhbSoftwareButton(mKeyCodes.buttons.undefined, mKeyCodes.buttons.undefined)
    },
    mUsb(mName, *this),
    mIsRunning(false),
    mIsSimulationMode(false),
    mPreviousButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                         mStepHandler.stepSize.continuous),
    mCurrentButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                        mStepHandler.stepSize.continuous),
    mDevNull(nullptr),
    mTxCout(&mDevNull),
    mRxCout(&mDevNull),
    mKeyEventCout(&mDevNull),
    mHalInitCout(&mDevNull),
    mInitCout(&mDevNull),
    keyEventReceiver(*this),
    packageReceivedEventReceiver(*this),
    packageIntepretedEventReceiver(*this),
    mIsCrcDebuggingEnabled(false)
{
    setSimulationMode(true);
    enableVerboseRx(false);
    enableVerboseTx(false);
    enableVerboseInit(false);
    enableVerboseHal(false);
}

// ----------------------------------------------------------------------

WhbContext::~WhbContext()
{
}

// ----------------------------------------------------------------------

void WhbContext::sendDisplayData()
{
    mUsb.sendDisplayData();
}

// ----------------------------------------------------------------------

void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode, std::ostream& out)
{
    ios init(NULL);
    init.copyfmt(out);
    int indent = 15;
    out << setfill(' ');

    // no key code
    if (keyCode == mKeyCodes.buttons.undefined.code)
    {
        out << setw(indent) << "";
        return;
    }

    const WhbKeyCode& whbKeyCode = mKeyCodes.buttons.getKeyCode(keyCode);

    // print key text
    if (modifierCode == mKeyCodes.buttons.function.code)
    {
        out << setw(indent) << whbKeyCode.altText;
    }
    else
    {
        out << setw(indent) << whbKeyCode.text;
    }
    out.copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode, std::ostream& out)
{
    ios init(NULL);
    init.copyfmt(out);

    // find key code
    const WhbKeyCode* whbKeyCode = keyCodeBase;
    while (whbKeyCode->code != 0)
    {
        if (whbKeyCode->code == keyCode)
        {
            break;
        }
        whbKeyCode++;
    }
    out << setw(5) << whbKeyCode->text << "(" << setw(4) << whbKeyCode->altText << ")";
    out.copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbUsbOutPackageAxisCoordinate::setCoordinate(const float& coordinate)
{
    float coordinateAbs = rtapi_fabs(coordinate);
    if (coordinate == coordinateAbs)
    {
        coordinateSign = 0;
    }
    else
    {
        coordinateSign = 1;
    }

    uint32_t scaledCoordinate = static_cast<uint32_t>(rtapi_rint(coordinateAbs * 10000.0));
    integerValue  = static_cast<uint16_t>(scaledCoordinate / 10000);
    fractionValue = static_cast<uint16_t>(scaledCoordinate % 10000);
}

// ----------------------------------------------------------------------

void WhbUsbOutPackageAxisCoordinate::clear()
{
    integerValue   = 0;
    fractionValue  = 0;
    coordinateSign = 0;
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageAxisCoordinate& coordinate)
{
    ios init(NULL);
    init.copyfmt(os);
    os << ((coordinate.coordinateSign == 1) ? "-" : "+") << setfill('0')
       << setw(4) << static_cast<unsigned short>(coordinate.integerValue) << "."
       << setw(4) << static_cast<unsigned short>(coordinate.fractionValue);

    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

DisplayIndicator::DisplayIndicator() :
    asByte(0)
{
}

// ----------------------------------------------------------------------

WhbUsbOutPackageBlockFields::WhbUsbOutPackageBlockFields() :
    reportId(0x06),
    __padding0(0),
    __padding1(0),
    __padding2(0),
    __padding3(0),
    __padding4(0),
    __padding5(0),
    __padding6(0)
{
}

// ----------------------------------------------------------------------

void WhbUsbOutPackageBlockFields::init(const void* data)
{
    reportId   = 0x06;
    __padding0 = reinterpret_cast<const uint8_t*>(data)[0];
    __padding1 = reinterpret_cast<const uint8_t*>(data)[1];
    __padding2 = reinterpret_cast<const uint8_t*>(data)[2];
    __padding3 = reinterpret_cast<const uint8_t*>(data)[3];
    __padding4 = reinterpret_cast<const uint8_t*>(data)[4];
    __padding5 = reinterpret_cast<const uint8_t*>(data)[5];
    __padding6 = reinterpret_cast<const uint8_t*>(data)[6];
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlockFields& block)
{
    ios init(NULL);
    init.copyfmt(os);

    os << hex << setfill('0') << setw(2) << static_cast<unsigned short>(block.reportId) << setw(2)
       << static_cast<unsigned short>(block.__padding0) << setw(2) << static_cast<unsigned short>(block.__padding1)
       << setw(2) << static_cast<unsigned short>(block.__padding2) << setw(2)
       << static_cast<unsigned short>(block.__padding3) << setw(2) << static_cast<unsigned short>(block.__padding4)
       << setw(2) << static_cast<unsigned short>(block.__padding5) << setw(2)
       << static_cast<unsigned short>(block.__padding6);

    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

WhbUsbOutPackageBlocks::WhbUsbOutPackageBlocks() :
    block0(),
    block1(),
    block2()
{
}

// ----------------------------------------------------------------------

void WhbUsbOutPackageBlocks::init(const WhbUsbOutPackageData* data)
{
    const uint8_t* d = reinterpret_cast<const uint8_t*>(data);
    block0.init(d += 0);
    block1.init(d += 7);
    block2.init(d + 7);
}

// ----------------------------------------------------------------------


std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlocks& blocks)
{
    return os << blocks.block0 << " " << blocks.block1 << " " << blocks.block2;
}

// ----------------------------------------------------------------------

WhbUsbOutPackageData::WhbUsbOutPackageData()
{
    clear();
}

// ----------------------------------------------------------------------

void WhbUsbOutPackageData::clear()
{
    header = 0xfdfe;
    //! \sa WhbContext::printCrcDebug(const WhbUsbInPackage&, const WhbUsbOutPackageData&)
    seed   = 0xfe;
    displayModeFlags.asByte = 0;

    row1Coordinate.clear();
    row2Coordinate.clear();
    row3Coordinate.clear();

    feedRate     = 0;
    spindleSpeed = 0;
    padding      = 0;
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageData& data)
{
    ios init(NULL);
    init.copyfmt(os);

    bool enableMultiline = false;
    if (enableMultiline)
    {
        os << hex << setfill('0') << "header       0x" << setw(2) << data.header << endl << "day of month   0x"
           << setw(2)
           << static_cast<unsigned short>(data.seed) << endl << "status 0x" << setw(2)
           << static_cast<unsigned short>(data.displayModeFlags.asByte) << endl << dec << "coordinate1  "
           << data.row1Coordinate << endl << "coordinate2  " << data.row2Coordinate << endl << "coordinate3  "
           << data.row3Coordinate << endl << "feed rate        " << data.feedRate << endl << "spindle rps      "
           << data.spindleSpeed;
    }
    else
    {
        os << hex << setfill('0') << "hdr 0x" << setw(4) << data.header << " dom 0x" << setw(2)
           << static_cast<unsigned short>(data.seed) << " status 0x" << setw(2)
           << static_cast<unsigned short>(data.displayModeFlags.asByte) << dec << " coord1 "
           << data.row1Coordinate << " coord2 " << data.row2Coordinate << " coord3 "
           << data.row3Coordinate << " feed " << setw(4) << data.feedRate << " spindle rps "
           << setw(5) << data.spindleSpeed;
    }
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

WhbUsbOutPackageBuffer::WhbUsbOutPackageBuffer() :
    asBlocks()
{
    if (false)
    {
        std::cout << "sizeof usb data " << sizeof(WhbUsbOutPackageData) << endl
                  << " blocks count   " << sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlockFields) << endl
                  << " sizeof block   " << sizeof(WhbUsbOutPackageBlockFields) << endl
                  << " sizeof blocks  " << sizeof(WhbUsbOutPackageBlocks) << endl
                  << " sizeof array   " << sizeof(asBlockArray) << endl
                  << " sizeof package " << sizeof(WhbUsbOutPackageData) << endl;
    }
    assert(sizeof(WhbUsbOutPackageBlocks) == sizeof(asBlockArray));
    size_t blocksCount = sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlockFields);
    assert ((sizeof(WhbUsbOutPackageData) + blocksCount) == sizeof(WhbUsbOutPackageBlocks));
}

// ----------------------------------------------------------------------

WhbUsbInPackage::WhbUsbInPackage() :
    header(0),
    randomByte(0),
    buttonKeyCode1(0),
    buttonKeyCode2(0),
    rotaryButtonFeedKeyCode(0),
    rotaryButtonAxisKeyCode(0),
    stepCount(0),
    crc(0)
{
}

// ----------------------------------------------------------------------

WhbUsbInPackage::WhbUsbInPackage(const uint8_t notAvailable1, const uint8_t notAvailable2, const uint8_t buttonKeyCode1,
                                 const uint8_t buttonKeyCode2, const uint8_t rotaryButtonFeedKeyCode,
                                 const uint8_t rotaryButtonAxisKeyCode, const int8_t stepCount, const uint8_t crc) :
    header(notAvailable1),
    randomByte(notAvailable2),
    buttonKeyCode1(buttonKeyCode1),
    buttonKeyCode2(buttonKeyCode2),
    rotaryButtonFeedKeyCode(rotaryButtonFeedKeyCode),
    rotaryButtonAxisKeyCode(rotaryButtonAxisKeyCode),
    stepCount(stepCount),
    crc(crc)
{
}

// ----------------------------------------------------------------------

void WhbContext::printInputData(const WhbUsbInPackage& inPackage, std::ostream& out)
{
    ios init(NULL);
    init.copyfmt(out);

    out << "| " << setfill('0') << hex << setw(2) << static_cast<unsigned short>(inPackage.header) << " | " << setw(2)
        << static_cast<unsigned short>(inPackage.randomByte) << " | ";
    out.copyfmt(init);
    printPushButtonText(inPackage.buttonKeyCode1, inPackage.buttonKeyCode2, out);
    out << " | ";
    printPushButtonText(inPackage.buttonKeyCode2, inPackage.buttonKeyCode1, out);
    out << " | ";
    printRotaryButtonText((WhbKeyCode*)&mKeyCodes.feed, inPackage.rotaryButtonFeedKeyCode, out);
    out << " | ";
    printRotaryButtonText((WhbKeyCode*)&mKeyCodes.axis, inPackage.rotaryButtonAxisKeyCode, out);
    out << " | " << setfill(' ') << setw(3) << static_cast<short>(inPackage.stepCount) << " | " << hex << setfill('0')
        << setw(2) << static_cast<unsigned short>(inPackage.crc);

    out.copyfmt(init);
}


// ----------------------------------------------------------------------

void WhbContext::printHexdump(const WhbUsbInPackage& inPackage, std::ostream& out)
{
    ios init(NULL);
    init.copyfmt(out);

    out << setfill('0') << hex << "0x" << setw(2) << static_cast<unsigned short>(inPackage.header) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.randomByte) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.buttonKeyCode1) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.buttonKeyCode2) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.rotaryButtonFeedKeyCode) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.rotaryButtonAxisKeyCode) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.stepCount & 0xff) << " " << setw(2)
        << static_cast<unsigned short>(inPackage.crc);
    out.copyfmt(init);
}

// ----------------------------------------------------------------------

int WhbContext::run()
{

    if (mHal.isSimulationModeEnabled())
    {
        *mInitCout << "init  starting in simulation mode" << endl;
    }

    bool isHalReady = false;
    initWhb();
    initHal();
    offerHalMemory();

    if (!mUsb.isWaitForPendantBeforeHalEnabled() && !mHal.isSimulationModeEnabled())
    {
        hal_ready(mHal.getHalComponentId());
        isHalReady = true;
    }

    while (isRunning())
    {
        *(mHal.memory->out.isPendantConnected) = 0;
        *(mHal.memory->out.isPendantRequired)  = mUsb.isWaitForPendantBeforeHalEnabled();

        initWhb();
        if (false == mUsb.init())
        {
            return EXIT_FAILURE;
        }

        *(mHal.memory->out.isPendantConnected) = 1;

        if (!isHalReady && !mHal.isSimulationModeEnabled())
        {
            hal_ready(mHal.getHalComponentId());
            isHalReady = true;
        }

        if (mUsb.isDeviceOpen())
        {
            *mInitCout << "init  enabling reception ...";
            if (!enableReceiveAsyncTransfer())
            {
                cerr << endl << "failed to enable reception" << endl;
                return EXIT_FAILURE;
            }
            *mInitCout << " ok" << endl;
            //Whb.sendDisplayData();
        }

        process();
        teardownUsb();
    }
    teardownHal();

    return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------

void WhbContext::linuxcncSimulate()
{
    static int last_jog_counts = 0; // todo: move to class field
    // *(hal->stepsizeUp) = ((xhc->button_step != 0) && (currentButtonCodes.mCurrentButton1Code == xhc->button_step));

    if (*(mHal.memory->out.jogCount) != last_jog_counts)
    {
        /*
        int   delta_int = *(mHal.memory.jogCount) - last_jog_counts;
        float delta     = delta_int * *(mHal.memory.jogScale);
        if (*(mHal.memory.jogEnableX))
        {
            *(mHal.memory.xMachineCoordinate) += delta;
            *(mHal.memory.xWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableY))
        {
            *(mHal.memory.yMachineCoordinate) += delta;
            *(mHal.memory.yWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableZ))
        {
            *(mHal.memory.zMachineCoordinate) += delta;
            *(mHal.memory.zWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableA))
        {
            *(mHal.memory.aMachineCoordinate) += delta;
            *(mHal.memory.aWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableB))
        {
            *(mHal.memory.bMachineCoordinate) += delta;
            *(mHal.memory.bWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableC))
        {
            *(mHal.memory.cMachineCoordinate) += delta;
            *(mHal.memory.cWorkpieceCoordinate) += delta;
        }*/

        //if (*(hal->jog_enable_spindle)) {
        //*(hal->spindle_override) += delta_int * 0.01;
        //if (*(hal->spindle_override) > 1) *(hal->spindle_override) = 1;
        //if (*(hal->spindle_override) < 0) *(hal->spindle_override) = 0;
        //*(hal->spindle_rps) = 25000.0/60.0 * *(hal->spindle_override);
        //}

        //if (*(hal->jog_enable_feedrate)) {
        //*(hal->feedrate_override) += delta_int * 0.01;
        //if (*(hal->feedrate_override) > 1) *(hal->feedrate_override) = 1;
        //if (*(hal->feedrate_override) < 0) *(hal->feedrate_override) = 0;
        //*(hal->feedrate) = 3000.0/60.0 * *(hal->feedrate_override);
        //}

        last_jog_counts = *(mHal.memory->out.jogCount);
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

bool WhbContext::enableReceiveAsyncTransfer()
{
    return mUsb.setupAsyncTransfer();
}

// ----------------------------------------------------------------------

void WhbContext::onUsbDataReceivedCallback(struct libusb_transfer* transfer)
{
    // pass transfer to data parser
    mUsb.onUsbDataReceived(transfer);
}

// ----------------------------------------------------------------------

void WhbContext::setSimulationMode(bool enableSimulationMode)
{
    mIsSimulationMode = enableSimulationMode;
    mHal.setSimulationMode(mIsSimulationMode);
    mUsb.setSimulationMode(mIsSimulationMode);
}

// ----------------------------------------------------------------------


void WhbContext::setUsbContext(libusb_context* context)
{
    mUsb.setContext(context);
}

// ----------------------------------------------------------------------

libusb_device_handle* WhbContext::getUsbDeviceHandle()
{
    return mUsb.getDeviceHandle();
}

// ----------------------------------------------------------------------

libusb_context* WhbContext::getUsbContext()
{
    return mUsb.getContext();
}

// ----------------------------------------------------------------------

void WhbContext::process()
{
    if (mUsb.isDeviceOpen())
    {
        while (isRunning() && !mUsb.getDoReconnect())
        {
            struct timeval tv;
            tv.tv_sec  = 4;
            tv.tv_usec = 0;

            int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
            assert((r == LIBUSB_SUCCESS) || (r == LIBUSB_ERROR_NO_DEVICE) || (r == LIBUSB_ERROR_BUSY) ||
                   (r == LIBUSB_ERROR_TIMEOUT) || (r == LIBUSB_ERROR_INTERRUPTED));
            //mHal.computeVelocity();
            if (mHal.isSimulationModeEnabled())
            {
                linuxcncSimulate();
            }
            sendDisplayData();
        }

        *(mHal.memory->out.isPendantConnected) = 0;
        *mInitCout << "connection lost, cleaning up" << endl;
        struct timeval tv;
        tv.tv_sec  = 1;
        tv.tv_usec = 0;
        int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
        assert(0 == r);
        r = libusb_release_interface(getUsbDeviceHandle(), 0);
        assert((0 == r) || (r == LIBUSB_ERROR_NO_DEVICE));
        libusb_close(getUsbDeviceHandle());
        mUsb.setDeviceHandle(nullptr);
    }
}

// ----------------------------------------------------------------------

void WhbContext::teardownUsb()
{
    libusb_exit(getUsbContext());
    mUsb.setContext(nullptr);
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseRx(bool enable)
{
    mUsb.enableVerboseRx(enable);
    if (enable)
    {
        mRxCout = &std::cout;
    }
    else
    {
        mRxCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseTx(bool enable)
{
    mUsb.enableVerboseTx(enable);
    if (enable)
    {
        mTxCout = &std::cout;
    }
    else
    {
        mTxCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseHal(bool enable)
{
    mHal.setEnableVerbose(enable);

    if (enable)
    {
        mHalInitCout = &std::cout;
    }
    else
    {
        mHalInitCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseInit(bool enable)
{
    mUsb.enableVerboseInit(enable);
    if (enable)
    {
        mInitCout = &std::cout;
    }
    else
    {
        mInitCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode)
{
    printPushButtonText(keyCode, modifierCode, *mRxCout);
}

// ----------------------------------------------------------------------

void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode)
{
    printRotaryButtonText(keyCodeBase, keyCode, *mRxCout);
}

// ----------------------------------------------------------------------

void WhbContext::printInputData(const WhbUsbInPackage& inPackage)
{
    printInputData(inPackage, *mRxCout);
}

// ----------------------------------------------------------------------

void WhbContext::printHexdump(const WhbUsbInPackage& inPackage)
{
    printHexdump(inPackage, *mRxCout);
}

// ----------------------------------------------------------------------

void WhbContext::setWaitWithTimeout(uint8_t waitSecs)
{
    mUsb.setWaitWithTimeout(waitSecs);
}

// ----------------------------------------------------------------------

bool WhbContext::isSimulationModeEnabled() const
{
    return mIsSimulationMode;
}

// ----------------------------------------------------------------------

size_t WhbContext::getSoftwareButtonIndex(uint8_t keyCode) const
{
    int      buttonsCount = sizeof(mSoftwareButtons) / sizeof(WhbSoftwareButton);
    for (int idx          = 0; idx < buttonsCount; idx++)

        if (mSoftwareButtons[idx].key.code == keyCode)
        {
            return idx;
        }
    assert (false);
    return 0;
}

// ----------------------------------------------------------------------

bool WhbContext::dispatchButtonEventToHal(const WhbSoftwareButton& softwareButton, bool isButtonPressed)
{
    const WhbButtonsCode& buttonCodes = mKeyCodes.buttons;

    // reset button
    if (softwareButton.containsKeys(buttonCodes.reset, buttonCodes.undefined))
    {
        mHal.setReset(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // stop button
    else if (softwareButton.containsKeys(buttonCodes.stop, buttonCodes.undefined))
    {
        mHal.setStop(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // start pause button
    else if (softwareButton.containsKeys(buttonCodes.start, buttonCodes.undefined))
    {
        mHal.setStart(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // feed + button
    else if (softwareButton.containsKeys(buttonCodes.feed_plus, buttonCodes.undefined))
    {
        mHal.setFeedPlus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // feed - button
    else if (softwareButton.containsKeys(buttonCodes.feed_minus, buttonCodes.undefined))
    {
        mHal.setFeedMinus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle + button
    else if (softwareButton.containsKeys(buttonCodes.spindle_plus, buttonCodes.undefined))
    {
        mHal.setSpindlePlus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle - button
    else if (softwareButton.containsKeys(buttonCodes.spindle_minus, buttonCodes.undefined))
    {
        mHal.setSpindleMinus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // machine home button
    else if (softwareButton.containsKeys(buttonCodes.machine_home, buttonCodes.undefined))
    {
        mHal.setMachineHome(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // safe-z button
    else if (softwareButton.containsKeys(buttonCodes.safe_z, buttonCodes.undefined))
    {
        mHal.setSafeZ(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // workpiece home button
    else if (softwareButton.containsKeys(buttonCodes.workpiece_home, buttonCodes.undefined))
    {
        mHal.setWorkpieceHome(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle on/off button
    else if (softwareButton.containsKeys(buttonCodes.spindle_on_off, buttonCodes.undefined))
    {
        mHal.setSpindleOn(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // function - button
    else if (softwareButton.containsKeys(buttonCodes.function, buttonCodes.undefined))
    {
        mHal.setFunction(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // probe-z button
    else if (softwareButton.containsKeys(buttonCodes.probe_z, buttonCodes.undefined))
    {
        mHal.setProbeZ(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 10 button
    else if (softwareButton.containsKeys(buttonCodes.macro10, buttonCodes.undefined))
    {
        mHal.setMacro10(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 11 button
    else if (softwareButton.containsKeys(buttonCodes.reset, buttonCodes.function))
    {
        mHal.setMacro11(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 12 button
    else if (softwareButton.containsKeys(buttonCodes.stop, buttonCodes.function))
    {
        mHal.setMacro12(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 13 button
    else if (softwareButton.containsKeys(buttonCodes.start, buttonCodes.function))
    {
        mHal.setMacro13(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 1 button
    else if (softwareButton.containsKeys(buttonCodes.feed_plus, buttonCodes.function))
    {
        mHal.setMacro1(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 2 button
    else if (softwareButton.containsKeys(buttonCodes.feed_minus, buttonCodes.function))
    {
        mHal.setMacro2(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 3 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_plus, buttonCodes.function))
    {
        mHal.setMacro3(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 4 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_minus, buttonCodes.function))
    {
        mHal.setMacro4(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 5 button
    else if (softwareButton.containsKeys(buttonCodes.machine_home, buttonCodes.function))
    {
        mHal.setMacro5(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 6 button
    else if (softwareButton.containsKeys(buttonCodes.safe_z, buttonCodes.function))
    {
        mHal.setMacro6(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 7 button
    else if (softwareButton.containsKeys(buttonCodes.workpiece_home, buttonCodes.function))
    {
        mHal.setMacro7(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 8 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_on_off, buttonCodes.function))
    {
        mHal.setMacro8(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 9 button
    else if (softwareButton.containsKeys(buttonCodes.probe_z, buttonCodes.function))
    {
        mHal.setMacro9(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 14 button
    else if (softwareButton.containsKeys(buttonCodes.macro10, buttonCodes.function))
    {
        mHal.setMacro14(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 15 button
    else if (softwareButton.containsKeys(buttonCodes.manual_pulse_generator, buttonCodes.function))
    {
        mHal.setMacro15(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 16 button
    else if (softwareButton.containsKeys(buttonCodes.step_continuous, buttonCodes.function))
    {
        mHal.setMacro16(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }

    return false;
}
// ----------------------------------------------------------------------


bool WhbContext::onButtonPressedEvent(const WhbSoftwareButton& softwareButton)
{
    bool isUpdateRecommended = false;

    printPushButtonText(softwareButton.key.code, softwareButton.modifier.code, *mKeyEventCout);
    *mKeyEventCout << endl;
    *mKeyEventCout << "event pressed ";
    if (!dispatchButtonEventToHal(softwareButton, true))
    {
        const WhbButtonsCode& buttonCodes = mKeyCodes.buttons;
        // TODO: implement validation if mpg/step can be applied on the current feed rotary button state
        // continuous button (MPG) and valid velocity rotary button position xxx
        if (softwareButton.containsKeys(buttonCodes.manual_pulse_generator, buttonCodes.undefined))
        {
            // obsolete
            //mCurrentButtonCodes.setCurrentModeContinuousMode();
            isUpdateRecommended = true;
        }
            // step button and valid step rotary button position
        else if (softwareButton.containsKeys(buttonCodes.step_continuous, buttonCodes.undefined))
        {
            // obsolete
            //mCurrentButtonCodes.setCurrentModeStepMode();
            isUpdateRecommended = true;
        }
    }

    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

bool WhbContext::onButtonReleasedEvent(const WhbSoftwareButton& softwareButton)
{
    bool isUpdateRecommended = false;
    *mKeyEventCout << "event released ";
    printPushButtonText(softwareButton.key.code, softwareButton.modifier.code, *mKeyEventCout);
    *mKeyEventCout << endl;

    dispatchButtonEventToHal(softwareButton, false);

    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

void WhbContext::onAxisActiveEvent(const WhbKeyCode& axis)
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event axis active   " << setw(5) << axis.text << " (" << setw(4) << axis.altText << ")" << endl;
    mKeyEventCout->copyfmt(init);

    dispatchAxisEventToHal(axis, true);
}

// ----------------------------------------------------------------------

void WhbContext::onAxisInactiveEvent(const WhbKeyCode& axis)
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event axis inactive " << setw(5) << axis.text << " (" << setw(4) << axis.altText << ")" << endl;
    mKeyEventCout->copyfmt(init);

    dispatchAxisEventToHal(axis, false);
}

// ----------------------------------------------------------------------

void WhbContext::onDataInterpreted()
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event data interpreted, display data ready" << endl;
    // mIsDisplayDataReady = true;
    mKeyEventCout->copyfmt(init);
}

// ----------------------------------------------------------------------

//! update axis rotary button's state to hal and detect active/inactive event
void WhbContext::updateAxisRotaryButton(const WhbUsbInPackage& inPackage)
{
    uint8_t newAxisKeyCode = inPackage.rotaryButtonAxisKeyCode;
    const WhbKeyCode& currentAxisCode = mCurrentButtonCodes.getAxisCode();
    if (currentAxisCode.code != newAxisKeyCode)
    {
        const WhbKeyCode* newAxisCode = nullptr;
        keyEventReceiver.onAxisInactiveEvent(currentAxisCode);
        if (newAxisKeyCode == mKeyCodes.axis.off.code)
        {
            newAxisCode = &mKeyCodes.axis.off;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.x.code)
        {
            newAxisCode = &mKeyCodes.axis.x;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.y.code)
        {
            newAxisCode = &mKeyCodes.axis.y;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.z.code)
        {
            newAxisCode = &mKeyCodes.axis.z;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.a.code)
        {
            newAxisCode = &mKeyCodes.axis.a;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.b.code)
        {
            newAxisCode = &mKeyCodes.axis.b;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.c.code)
        {
            newAxisCode = &mKeyCodes.axis.c;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.undefined.code)
        {
            newAxisCode = &mKeyCodes.axis.undefined;
        }
        else
        {
            *mRxCout << "received unexpected axis id " << static_cast<unsigned short>(newAxisKeyCode) << endl;
            assert(false);
        }
        mCurrentButtonCodes.setAxisCode(*newAxisCode);
        keyEventReceiver.onAxisActiveEvent(*newAxisCode);
    }
}

// ----------------------------------------------------------------------

bool WhbContext::updateHalButtons(const WhbUsbInPackage& inPackage, uint8_t keyCode, uint8_t modifierCode)
{
    bool isUpdateRecommended = false;

    for (int idx = 0; !((mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].key.code) &&
                        (mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].modifier.code)); idx++)
    {
        const WhbSoftwareButton& softwareButton = mSoftwareButtons[idx];
        hal_bit_t halButtonState             = *(mHal.memory->out.button_pin[idx]);
        uint8_t   softwareButtonKeyCode      = softwareButton.key.code;
        uint8_t   softwareButtonModifierCode = softwareButton.modifier.code;

        if ((halButtonState == true) && // on last button state was pressed
            !((softwareButtonKeyCode == keyCode) && // and current state is not pressed
              (softwareButtonModifierCode == modifierCode)))
        {
            // on button released event
            *(mHal.memory->out.button_pin[idx]) = false;
            keyEventReceiver.onButtonReleasedEvent(softwareButton);
        }
        else if ((halButtonState == false) &&  // on last button state was unpressed
                 ((softwareButtonKeyCode == keyCode) && // and current state is pressed
                  (softwareButtonModifierCode == modifierCode)))
        {
            // on button pressed event
            *(mHal.memory->out.button_pin[idx]) = true;
            mCurrentButtonCodes.updateButtonState(softwareButton);
            isUpdateRecommended = keyEventReceiver.onButtonPressedEvent(softwareButton);
        }
    }
    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

//! update axis rotary button's state to hal and detect active/inactive event
void WhbContext::updateStepRotaryButton(const WhbUsbInPackage& inPackage, bool forceEvents)
{
    uint8_t newStepKeyCode = inPackage.rotaryButtonFeedKeyCode;
    const WhbKeyCode& currentStepCode = mCurrentButtonCodes.getFeedCode();

    if ((currentStepCode.code != newStepKeyCode) || forceEvents)
    {
        keyEventReceiver.onFeedInactiveEvent(currentStepCode);
        const WhbKeyCode* newFeedCode = nullptr;

        if (newStepKeyCode == mKeyCodes.feed.speed_0_001.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::RotaryButton0001);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton2percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_001;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_0_01.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::RotaryButton0010);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton5percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_01;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_0_1.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::RotaryButton0100);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton10percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_1;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_1.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::RotaryButton100);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton30percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_1;
        }
        else if (newStepKeyCode == mKeyCodes.feed.percent_60.code)
        {
            if (mCurrentButtonCodes.isCurrentModeContinuousMode())
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton60percent);
                newFeedCode = &mKeyCodes.feed.percent_60;
            }
        }
        else if (newStepKeyCode == mKeyCodes.feed.percent_100.code)
        {
            if (mCurrentButtonCodes.isCurrentModeContinuousMode())
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::RotaryButton100percent);
                newFeedCode = &mKeyCodes.feed.percent_100;
            }
        }
        else if (newStepKeyCode == mKeyCodes.feed.lead.code)
        {
            newFeedCode = &mKeyCodes.feed.lead;
            mHal.setLead();
        }
        else
        {
            // \p newFeedCode value may be unexpected i.e. on device power off
        }

        //! feed rotary button position changed but no valid feed step size available for this mode
        if (newFeedCode != nullptr)
        {
            if ((*newFeedCode) != mKeyCodes.feed.lead)
            {
                mHal.setJogWheelStepMode(mCurrentButtonCodes.currentStepMode());
                mHal.setStepSize(mCurrentButtonCodes.getStepSize());
            }
            mCurrentButtonCodes.setFeedCode(*newFeedCode);
            keyEventReceiver.onFeedActiveEvent(*newFeedCode);
        }
    }
}

// ----------------------------------------------------------------------

void WhbContext::onFeedActiveEvent(const WhbKeyCode& axis)
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event feed active  " << setfill(' ') << setw(5) << axis.text << "(" << setw(4) << axis.altText
                   << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbContext::onFeedInactiveEvent(const WhbKeyCode& axis)
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event feed inactive  " << setfill(' ') << setw(5) << axis.text << "(" << setw(4) << axis.altText
                   << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbContext::setEnableVerboseKeyEvents(bool enable)
{
    mUsb.enableVerboseRx(enable);
    if (enable)
    {
        mKeyEventCout = &std::cout;
    }
    else
    {
        mKeyEventCout = &mDevNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::onJogDialEvent(int8_t delta)
{
    ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event jog dial " << setfill(' ') << setw(3) << static_cast<signed short>(delta) << endl;
    mKeyEventCout->copyfmt(init);
    mHal.newJogDialDelta(delta);
}

// ----------------------------------------------------------------------

void WhbContext::updateJogDial(const WhbUsbInPackage& inPackage)
{
    if (inPackage.stepCount != 0)
    {
        onJogDialEvent(inPackage.stepCount);
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableCrcDebugging(bool enable)
{
    mIsCrcDebuggingEnabled = enable;
}

// ----------------------------------------------------------------------

size_t WhbContext::getHalPinNumber(const WhbSoftwareButton& button)
{
    for (size_t idx = 0; !((mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].key.code) &&
                           (mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].modifier.code)); idx++)
    {
        if ((mSoftwareButtons[idx].key.code == button.key.code) &&
            (mSoftwareButtons[idx].modifier.code == button.modifier.code))
        {
            return idx;
        }
    }
    assert(false);
    return 0;
}

// ----------------------------------------------------------------------

void WhbContext::offerHalMemory()
{
    assert(mHal.memory != nullptr);
    mUsb.takeHalMemoryReference(mHal.memory);
}

// ----------------------------------------------------------------------

void WhbContext::dispatchAxisEventToHal(const WhbKeyCode& axis, bool isActive)
{
    if (axis.code == mKeyCodes.axis.off.code)
    {
        mHal.setNoAxisActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.x.code)
    {
        mHal.setAxisXActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.y.code)
    {
        mHal.setAxisYActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.z.code)
    {
        mHal.setAxisZActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.a.code)
    {
        mHal.setAxisAActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.b.code)
    {
        mHal.setAxisBActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.c.code)
    {
        mHal.setAxisCActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.undefined.code)
    {
        mHal.setNoAxisActive(isActive);
    }
}

// ----------------------------------------------------------------------

void WhbUsb::setSimulationMode(bool isSimulationMode)
{
    mIsSimulationMode = isSimulationMode;
}

// ----------------------------------------------------------------------

void WhbUsb::setIsRunning(bool enableRunning)
{
    mIsRunning = enableRunning;
}

// ----------------------------------------------------------------------

void WhbUsb::requestTermination()
{
    mIsRunning = false;
}

// ----------------------------------------------------------------------

bool WhbUsb::setupAsyncTransfer()
{
    assert(inTransfer != nullptr);
    libusb_fill_bulk_transfer(inTransfer, deviceHandle,
        //! TODO: LIBUSB_ENDPOINT_IN
                              (0x1 | LIBUSB_ENDPOINT_IN), inputPackageBuffer.asBuffer,
                              sizeof(inputPackageBuffer.asBuffer), usbInputResponseCallback,
        //! no callback data
                              nullptr,
        //! timeout[ms]
                              750);
    int r = libusb_submit_transfer(inTransfer);
    assert(0 == r);
    return (0 == r);
}

// ----------------------------------------------------------------------

void WhbUsb::onUsbDataReceived(struct libusb_transfer* transfer)
{
    int expectedPckageSize = static_cast<int>(sizeof(WhbUsbInPackage));
    ios init(NULL);
    init.copyfmt(*verboseTxOut);
    switch (transfer->status)
    {
        case (LIBUSB_TRANSFER_COMPLETED):
            // sleep mode was previously detected, drop current package
            if (sleepState.mDropNextInPackage)
            {
                if (WhbUsb::ConstantPackages.sleepPackage != inputPackageBuffer.asFields)
                {
                    *verboseTxOut << "expected sleep package starting with " << hex << setfill('0') << setw(2)
                                  << static_cast<unsigned short>(WhbUsb::ConstantPackages.sleepPackage.header)
                                  << " but got " << hex << setfill('0') << setw(2)
                                  << static_cast<unsigned short>(inputPackageBuffer.asFields.header) << endl;
                    verboseTxOut->copyfmt(init);
                }

                sleepState.mDropNextInPackage = false;
                goto ___TRUNCATE_PACKAGE;
            }

            if (transfer->actual_length == expectedPckageSize)
            {
                //! detect pendant going to sleep:
                //! when powering off pedant sends two packages
                //! 1st: \ref WhbUsbEmptyPackage
                //! 2nd: \ref WhbUsbSleepPackage
                if (WhbUsb::ConstantPackages.emptyPackage == inputPackageBuffer.asFields)
                {
                    sleepState.mDropNextInPackage = true;
                    *(mHalMemory->out.isPendantSleeping) = 1;
                    if (mIsSimulationMode)
                    {
                        struct timeval now;
                        gettimeofday(&now, nullptr);
                        *verboseTxOut << "event going to sleep: device was idle for "
                                      << (now.tv_sec - sleepState.mLastWakeupTimestamp.tv_sec) << " seconds" << endl;
                    }
                }
                    // on regular package
                else
                {
                    if (*(mHalMemory->out.isPendantSleeping) == 1)
                    {
                        *(mHalMemory->out.isPendantSleeping) = 0;
                        if (mIsSimulationMode)
                        {
                            struct timeval now;
                            gettimeofday(&now, nullptr);
                            *verboseTxOut << "woke up: device was sleeping for "
                                          << (now.tv_sec - sleepState.mLastWakeupTimestamp.tv_sec) << " seconds"
                                          << endl;
                        }
                        gettimeofday(&sleepState.mLastWakeupTimestamp, nullptr);
                    }
                }
                // pass structured transfer to usb data handler
                mDataHandler.onInputDataReceived(inputPackageBuffer.asFields);
            }
            else
            {
                cerr << "received unexpected package size: expected=" << (transfer->actual_length) << ", current="
                     << expectedPckageSize << endl;
            }

            if (mIsRunning)
            {
                setupAsyncTransfer();
            }

            break;

        ___TRUNCATE_PACKAGE:
        case (LIBUSB_TRANSFER_TIMED_OUT):
            if (mIsRunning)
            {
                setupAsyncTransfer();
            }
            break;

        case (LIBUSB_TRANSFER_CANCELLED):
            break;

        case (LIBUSB_TRANSFER_STALL):
        case (LIBUSB_TRANSFER_NO_DEVICE):
        case (LIBUSB_TRANSFER_OVERFLOW):
        case (LIBUSB_TRANSFER_ERROR):
            cerr << "transfer error: " << transfer->status << endl;
            requestTermination();
            break;

        default:
            cerr << "unknown transfer status: " << transfer->status << endl;
            requestTermination();
            break;
    }
    //libusb_free_transfer(transfer);
}

// ----------------------------------------------------------------------

WhbUsb::~WhbUsb()
{
}

// ----------------------------------------------------------------------

void WhbUsb::enableVerboseTx(bool enable)
{
    if (enable)
    {
        verboseTxOut = &std::cout;
    }
    else
    {
        verboseTxOut = &devNull;
    }
}

// ----------------------------------------------------------------------

void WhbUsb::enableVerboseRx(bool enable)
{
    if (enable)
    {
        verboseRxOut = &std::cout;
    }
    else
    {
        verboseRxOut = &devNull;
    }
}

// ----------------------------------------------------------------------

void WhbUsb::enableVerboseInit(bool enable)
{
    if (enable)
    {
        verboseInitOut = &std::cout;
    }
    else
    {
        verboseInitOut = &devNull;
    }
}

// ----------------------------------------------------------------------

bool WhbUsb::init()
{
    if (getDoReconnect() == true)
    {
        int pauseSecs = 3;
        *verboseInitOut << "init  pausing " << pauseSecs << "s, waiting for device to be gone ...";
        while ((pauseSecs--) >= 0)
        {
            *verboseInitOut << "." << std::flush;
            sleep(1);
        }
        setDoReconnect(false);
        *verboseInitOut << " done" << endl;
    }

    *verboseInitOut << "init  usb context ...";
    int r = libusb_init(&context);
    if (r != 0)
    {
        cerr << endl << "failed to initialize usb context" << endl;
        return false;
    }
    *verboseInitOut << " ok" << endl;

    libusb_log_level logLevel = LIBUSB_LOG_LEVEL_INFO;
    //logLevel = LIBUSB_LOG_LEVEL_DEBUG;
    libusb_set_debug(context, logLevel);

    ios init(NULL);
    init.copyfmt(*verboseInitOut);
    if (isWaitWithTimeout)
    {
        *verboseInitOut << "init  waiting maximum " << static_cast<unsigned short>(mWaitSecs) << "s for device "
                        << mName << " vendorId=0x" << hex << setfill('0') << setw(2) << usbVendorId
                        << " productId=0x" << setw(2) << usbProductId << " ...";
    }
    else
    {
        *verboseInitOut << "init  not waiting for device " << mName
                        << " vendorId=0x" << hex << setfill('0') << setw(2) << usbVendorId
                        << " productId=0x" << setw(2) << usbProductId << dec
                        << ", will continue in " << static_cast<unsigned short>(mWaitSecs) << "s ...";
    }
    verboseInitOut->copyfmt(init);

    do
    {
        libusb_device** devicesReference;
        ssize_t devicesCount = libusb_get_device_list(context, &devicesReference);
        if (devicesCount < 0)
        {
            cerr << endl << "failed to get device list" << endl;
            return false;
        }

        deviceHandle = libusb_open_device_with_vid_pid(context, usbVendorId, usbProductId);
        libusb_free_device_list(devicesReference, 1);
        *verboseInitOut << "." << std::flush;
        if (isDeviceOpen() == false)
        {
            *verboseInitOut << "." << std::flush;
            if (isWaitWithTimeout)
            {
                *verboseInitOut << "." << std::flush;
                if ((mWaitSecs--) <= 0)
                {
                    cerr << endl << "timeout exceeded, exiting" << endl;
                    return false;
                }
            }
            sleep(1);
        }
    } while ((isDeviceOpen() == false) && mIsRunning);
    *verboseInitOut << " ok" << endl
                    << "init  " << mName << " device found" << endl;

    if (isDeviceOpen())
    {
        *verboseInitOut << "init  detaching active kernel driver ...";
        if (libusb_kernel_driver_active(deviceHandle, 0) == 1)
        {
            int r = libusb_detach_kernel_driver(deviceHandle, 0);
            assert(0 == r);
            *verboseInitOut << " ok" << endl;
        }
        else
        {
            *verboseInitOut << " already detached" << endl;
        }
        *verboseInitOut << "init  claiming interface ...";
        int r = libusb_claim_interface(deviceHandle, 0);
        if (r != 0)
        {
            cerr << endl << "failed to claim interface" << endl;
            return false;
        }
        *verboseInitOut << " ok" << endl;
    }
    return true;
}

// ----------------------------------------------------------------------

void WhbUsb::setWaitWithTimeout(uint8_t waitSecs)
{
    mWaitSecs         = waitSecs;
    if (mWaitSecs > 0)
    {
        isWaitWithTimeout = true;
        return;
    }
    isWaitWithTimeout = false;
}

// ----------------------------------------------------------------------

const WhbUsbOutPackageData& WhbUsb::getOutputPackageData()
{
    return outputPackageData;
}

// ----------------------------------------------------------------------

void WhbUsb::takeHalMemoryReference(WhbHalMemory* memory)
{
    mHalMemory = memory;
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
    if (mIsSimulationMode == false)
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
    *mHalCout << "hal   new pin " << pin_name << endl;
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

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_float_t));
    }
    else
    {
        *mHalCout << "hal   new pin " << pin_name << endl;
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

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_s32_t));
    }
    else
    {
        *mHalCout << "hal   new pin " << pin_name << endl;
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

    if (mIsSimulationMode)
    {
        return newSimulatedHalPin(pin_name, (void**)ptr, sizeof(hal_bit_t));
    }
    else
    {
        assert(ptr != nullptr);
        assert(*ptr == nullptr);
        *mHalCout << "hal   new pin " << pin_name << endl;
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
            cerr << endl << "failed to initialize HAL component " << mName << endl;
            exit(EXIT_FAILURE);
        }
        *mHalCout << "ok" << endl;

        *mHalCout << "hal   initialize shared HAL memory for component id  " << mHalCompId << " ... ";
        memory = reinterpret_cast<WhbHalMemory*>(hal_malloc(sizeof(WhbHalMemory)));
        memory = new(memory) WhbHalMemory();
    }
    else
    {
        *mHalCout << "hal   initialize HAL memory " << " ... ";
        memory = new WhbHalMemory();
    }

    if (memory == nullptr)
    {
        cerr << "failed to allocate HAL memory" << endl;
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
    ios init(NULL);
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

    ios init(NULL);
    init.copyfmt(*mHalCout);
    *mHalCout << setfill(' ')
              << "hal   new jog dial delta " << setw(3) << static_cast<signed short>(delta) << endl;
    mHalCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbHal::setJogWheelStepMode(JogWheelStepMode::StepMode stepMode)
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
} // namespace

// ----------------------------------------------------------------------

XhcWhb04b6::WhbContext Whb;

// ----------------------------------------------------------------------

static int printUsage(const char* programName, const char* deviceName, bool isError = false)
{
    std::ostream* os = &std::cout;
    if (isError)
    {
        os = &cerr;
    }
    *os << programName << " version " << PACKAGE_VERSION << " " << __DATE__ << " " << __TIME__ << endl
        << endl
        << "SYNOPSIS" << endl
        << "    " << programName << " [-h] | [-H] [OPTIONS] " << endl
        << endl
        << "NAME" << endl
        << "    " << programName << " - jog dial HAL module for the " << deviceName << " device" << endl
        << endl
        << "DESCRIPTION" << endl
        << "    " << programName << " is a HAL module that receives events from the " << deviceName << " device "
        << "and exposes them to HAL via HAL pins." << endl
        << endl
        << "OPTIONS" << endl
        << " -h " << endl
        << "    Prints the synonpsis and the most commonly used commands." << endl
        << endl
        << " -H " << endl
        << "    run " << Whb.getName() << " in HAL-mode instead of interactive mode. When in HAL mode "
        << "commands from device will be exposed to HAL's shred memory. Interactive mode is useful for "
        << "testing device connectivity and debugging." << endl
        << endl
        << " -t " << endl
        << "    Wait with timeout for USB device then proceed, exit otherwise. Without -t the timeout is "
        << "ipmlicitely infinite." << endl
        << endl
        << " -u, -U " << endl
        << "    Show received data from device. With -U received and transmitted data will be printed. "
        << "Output is prefixed with \"usb\"." << endl
        << endl
        << " -p " << endl
        << "    Show HAL pins and HAL related messages. Output is prefixed with \"hal\"." << endl
        << endl
        << " -e " << endl
        << "    Show captured events such as button pressed/released, jog dial, axis rotary button, and "
            "feed rotary button event. Output is prefixed with \"event\"."
        << "and in case." << endl
        << endl
        << " -a " << endl
        << "    Enable all logging facilities without explicitly specifying each." << endl
        //! this feature must be removed when checksum check is implemented
        << endl
        << " -c " << endl
        << "    Enable checksum output which is necessary for debugging the checksum generator function. Do not rely "
            "on this featue since it will be removed once the generator is implemented." << endl
        << endl
        << " -s " << endl
        << "    Force being silent and not printing any output except of errors. This will also inhibit messages "
            "prefixed with \"init\"." << endl
        << endl
        << "EXAMPLES" << endl
        << programName << " -ue" << endl
        << "    Prints incoming USB data transfer and generated key pressed/released events." << endl
        << endl
        << programName << " -p" << endl
        << "    Prints hal pin names and events distributed to HAL memory." << endl
        << endl
        << programName << " -Ha" << endl
        << "    Start in HAL mode and avoid output, except of errors." << endl
        << endl
        << "AUTHORS" << endl
        << "    This module was started by Raoul Rubien (github.com/rubienr) based on predecessor "
            "device's module xhc-hb04.cc. https://github.com/machinekit/machinekit/graphs/contributors "
            "gives you a more complete list of contributors."
        << endl;

    if (isError)
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------

void registerSignalHandler()
{
    signal(SIGINT, quit);
    signal(SIGTERM, quit);
}

// ----------------------------------------------------------------------

static void quit(int signal)
{
    Whb.requestTermination(signal);
}

// ----------------------------------------------------------------------

void usbInputResponseCallback(struct libusb_transfer* transfer)
{
    // pass transfer to usb data parser
    Whb.onUsbDataReceivedCallback(transfer);
}

// ----------------------------------------------------------------------

int main(int argc, char** argv)
{
    const char* optargs = "phaseHuctU";
    for (int opt = getopt(argc, argv, optargs); opt != -1; opt = getopt(argc, argv, optargs))
    {
        switch (opt)
        {
            case 'H':
                Whb.setSimulationMode(false);
                break;
            case 't':
                Whb.setWaitWithTimeout(3);
                break;
            case 'e':
                Whb.setEnableVerboseKeyEvents(true);
                break;
            case 'u':
                Whb.enableVerboseInit(true);
                Whb.enableVerboseRx(true);
                break;
            case 'U':
                Whb.enableVerboseInit(true);
                Whb.enableVerboseRx(true);
                Whb.enableVerboseTx(true);
                break;
            case 'p':
                Whb.enableVerboseInit(true);
                Whb.enableVerboseHal(true);
                break;
            case 'a':
                Whb.enableVerboseInit(true);
                Whb.setEnableVerboseKeyEvents(true);
                Whb.enableVerboseRx(true);
                Whb.enableVerboseTx(true);
                Whb.enableVerboseHal(true);
                break;
            case 'c':
                Whb.enableCrcDebugging(true);
                break;
            case 's':
                break;
            case 'h':
                return printUsage(basename(argv[0]), Whb.getName());
                break;
            default:
                return printUsage(basename(argv[0]), Whb.getName(), true);
                break;
        }
    }

    registerSignalHandler();

    Whb.run();

    //! hotfix for https://github.com/machinekit/machinekit/issues/1266
    if (Whb.isSimulationModeEnabled())
    {
        google::protobuf::ShutdownProtobufLibrary();
    }

    return 0;
}
