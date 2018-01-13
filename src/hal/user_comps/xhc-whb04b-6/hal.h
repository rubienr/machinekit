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

// local includes
#include "pendant-types.h"

// system includes
#include <stdint.h>
#include <ostream>
#include <string>
#include <map>

// 3rd party includes

// local library includes
#include <hal.h>

// forward declarations

namespace XhcWhb04b6 {

// forward declarations
class WhbSoftwareButton;
class WhbKeyCodes;


// ----------------------------------------------------------------------

//! HAL memory pointers. Each pointer represents an i/o hal pin.

class WhbHalMemory
{
public:
    struct In
    {
    public:

        //! to be connected to \ref halui.axis.0.pos-feedback
        hal_float_t* axisXPosition;
        //! to be connected to \ref halui.axis.1.pos-feedback
        hal_float_t* axisYPosition;
        //! to be connected to \ref halui.axis.2.pos-feedback
        hal_float_t* axisZPosition;
        //! to be connected to \ref halui.axis.3.pos-feedback
        hal_float_t* axisAPosition;
        //! to be connected to \ref halui.axis.4.pos-feedback
        hal_float_t* axisBPosition;
        //! to be connected to \ref halui.axis.5.pos-feedback
        hal_float_t* axisCPosition;

        //! to be connected to \ref halui.axis.0.pos-relative
        hal_float_t* axisXPositionRelative;
        //! to be connected to \ref halui.axis.1.pos-relative
        hal_float_t* axisYPositionRelative;
        //! to be connected to \ref halui.axis.2.pos-relative
        hal_float_t* axisZPositionRelative;
        //! to be connected to \ref halui.axis.3.pos-relative
        hal_float_t* axisAPositionRelative;
        //! to be connected to \ref halui.axis.4.pos-relative
        hal_float_t* axisBPositionRelative;
        //! to be connected to \ref halui.axis.5.pos-relative
        hal_float_t* axisCPositionRelative;

        //! to be connected to \ref stepge.00.maxvel
        hal_float_t* stepgenXMaxVelocity;
        //! to be connected to \ref stepge.01.maxvel
        hal_float_t* stepgenYMaxVelocity;
        //! to be connected to \ref stepge.02.maxvel
        hal_float_t* stepgenZMaxVelocity;
        //! to be connected to \ref stepge.03.maxvel
        hal_float_t* stepgenAMaxVelocity;
        //! to be connected to \ref stepge.04.maxvel
        hal_float_t* stepgenBMaxVelocity;
        //! to be connected to \ref stepge.05.maxvel
        hal_float_t* stepgenCMaxVelocity;
        //! to be connected to \ref stepgen.00.position-scale
        hal_float_t* stepgenXPositionScale;
        //! to be connected to \ref stepgen.01.position-scale
        hal_float_t* stepgenYPositionScale;
        //! to be connected to \ref stepgen.02.position-scale
        hal_float_t* stepgenZPositionScale;
        //! to be connected to \ref stepgen.03.position-scale
        hal_float_t* stepgenAPositionScale;
        //! to be connected to \ref stepgen.04.position-scale
        hal_float_t* stepgenBPositionScale;
        //! to be connected to \ref stepgen.05.position-scale
        hal_float_t* stepgenCPositionScale;

        //! to be connected to \ref halui.spindle.is-on
        hal_bit_t  * spindleIsOn;
        //! to be connected to \ref halui.spindle-override.value
        hal_float_t* spindleOverrideValue;

        //! to be connected to \ref halui.program.is-running
        hal_bit_t  * isProgramRunning;
        //! to be connected to \ref halui.program.is-paused
        hal_bit_t  * isProgramPaused;
        //! to be connected to \ref halui.program.is-idle
        hal_bit_t  * isProgramIdle;

        //! to be connected to \ref halui.mode.is-auto
        hal_bit_t* isModeAuto;
        //! to be connected to \ref halui.mode.is-joint
        hal_bit_t* isModeJoint;
        //! to be connected to \ref halui.mode.is-manual
        hal_bit_t* isModeManual;
        //! to be connected to \ref halui.mode.is-madi
        hal_bit_t* isModeMdi;


        //! to be connected to \ref halui.estop.is-activated
        hal_bit_t* isEmergencyStop;

        //! to be connected to \ref halui.machine.is-on
        hal_bit_t* isMachineOn;

        In();
    };

    struct Out
    {
    public:
        hal_bit_t* button_pin[64];

        //! to be connected to \ref axis.0.jog-counts
        hal_s32_t* axisXJogCounts;
        //! to be connected to \ref axis.1.jog-counts
        hal_s32_t* axisYJogCounts;
        //! to be connected to \ref axis.2.jog-counts
        hal_s32_t* axisZJogCounts;
        //! to be connected to \ref axis.3.jog-counts
        hal_s32_t* axisAJogCounts;
        //! to be connected to \ref axis.4.jog-counts
        hal_s32_t* axisBJogCounts;
        //! to be connected to \ref axis.5.jog-counts
        hal_s32_t* axisCJogCounts;

        //! to be connected to \ref axis.0.jog-enable
        hal_bit_t* axisXJogEnable;
        //! to be connected to \ref axis.1.jog-enable
        hal_bit_t* axisYJogEnable;
        //! to be connected to \ref axis.2.jog-enable
        hal_bit_t* axisZJogEnable;
        //! to be connected to \ref axis.3.jog-enable
        hal_bit_t* axisAJogEnable;
        //! to be connected to \ref axis.4.jog-enable
        hal_bit_t* axisBJogEnable;
        //! to be connected to \ref axis.5.jog-enable
        hal_bit_t* axisCJogEnable;

        //! to be connected to \ref axis.0.jog-scale
        hal_float_t* axisXJogScale;
        //! to be connected to \ref axis.1.jog-scale
        hal_float_t* axisYJogScale;
        //! to be connected to \ref axis.2.jog-scale
        hal_float_t* axisZJogScale;
        //! to be connected to \ref axis.3.jog-scale
        hal_float_t* axisAJogScale;
        //! to be connected to \ref axis.4.jog-scale
        hal_float_t* axisBJogScale;
        //! to be connected to \ref axis.5.jog-scale
        hal_float_t* axisCJogScale;

        //! to be connected to \ref axis.0.jog-vel-mode
        hal_bit_t* axisXSetVelocityMode;
        //! to be connected to \ref axis.1.jog-vel-mode
        hal_bit_t* axisYSetVelocityMode;
        //! to be connected to \ref axis.2.jog-vel-mode
        hal_bit_t* axisZSetVelocityMode;
        //! to be connected to \ref axis.3.jog-vel-mode
        hal_bit_t* axisASetVelocityMode;
        //! to be connected to \ref axis.4.jog-vel-mode
        hal_bit_t* axisBSetVelocityMode;
        //! to be connected to \ref axis.5.jog-vel-mode
        hal_bit_t* axisCSetVelocityMode;

        //! to be connected to \ref halui.spindle.start
        hal_bit_t *spindleStart;
        //! to be connected to \ref halui.spindle.stop
        hal_bit_t *spindleStop;

        hal_bit_t* feedValueSelected0_001;
        hal_bit_t* feedValueSelected0_01;
        hal_bit_t* feedValueSelected0_1;
        hal_bit_t* feedValueSelected1_0;

        //! to be connected to \ref  \ref halui.feed-override.scale
        hal_float_t* feedOverrideScale;
        //! to be connected to \ref halui.feed-override.direct-value
        hal_bit_t* feedOverrideDirectValue;
        //! to be connected to \ref halui.feed-override.counts
        hal_s32_t* feedOverrideCounts;
        //! to be connected to \ref halui.feed-override.decrease
        hal_bit_t* feedOverrideDecrease;
        //! to be connected to \ref halui.feed-override.increase
        hal_bit_t* feedOverrideIncrease;

        //! to be connected to halui.spindle.decrease
        hal_bit_t* spindleDoDecrease;
        //! to be connected to halui.spindle.increase
        hal_bit_t* spindleDoIncrease;
        //! to be connected to halui.spindle-override.decrease
        hal_bit_t* spindleOverrideDoDecrease;
        //! to be connected to halui.spindle-override.increase
        hal_bit_t* spindleOverrideDoIncrease;

        //! to be connected to \ref halui.jog-speed
        hal_float_t* jogSpeedValue;

        //! to be connected to \ref halui.home-all
        hal_bit_t* homeAll;

        //! to be connected to \ref halui.jog.selected.increment
        //hal_float_t* jogIncrement;
        //! to be connected to \ref halui.jog.selected.increment-plus
        //hal_bit_t* jogIncrementPlus;
        //! to be connected to \ref halui.jog.selected.increment-minus
        //hal_bit_t* jogIncrementMinus;
        //! to be connected to \ref halui.jog.selected.plus
        //hal_bit_t* jogPlus; //jogging with jog-speed
        //! to be connected to \ref halui.jog.selected.minus
        //hal_bit_t* jogMinus; // jogging with jog-speed

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

        //! reflects the pendandt's idle state
        hal_bit_t* isPendantSleeping;
        //! reflects pendant's connectivity
        hal_bit_t* isPendantConnected;

        //! to be connected to \ref halui.program.run
        hal_bit_t* doRunProgram;
        //! to be connected to \ref halui.program.pause
        hal_bit_t* doPauseProgram;
        //! to be connected to \ref halui.program.resume
        hal_bit_t* doResumeProgram;
        //! to be connected to \ref halui.program.stop
        hal_bit_t* doStopProgram;

        //! to be connected to \ref halui.mode.auto
        hal_bit_t* doModeAuto;
        //! to be connected to \ref halui.mode.joint
        hal_bit_t* doModeJoint;
        //! to be connected to \ref halui.mode.manual
        hal_bit_t* doModeManual;
        //! to be connected to \ref halui.mode.mdi
        hal_bit_t* doModeMdi;

        //! to be connected to \ref halui.estop.activate
        hal_bit_t* doEmergencyStop;
        //! to be connected to \ref halui.estop.reset
        hal_bit_t* resetEmergencyStop;

        //! to be connected to \ref halui.machine.on
        hal_bit_t* doMachineOn;
        //! to be connected to \ref halui.machine.off
        hal_bit_t* doMachineOff;

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
    std::map<std::string, size_t> mButtonNameToIdx;
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
    void setJogWheelStepMode(HandwheelStepmodes::Mode stepMode);

    //! Sets the new feed rate. The step mode must be set accordingly.
    //! \param feedRate the new feed rate independent of step mode
    void setStepSize(const hal_float_t& feedRate);
    //! If lead is active.
    void setLead();
    //! Sets the hal state of the respecitive pin (reset). Usually called in case the reset
    //! button is pressed or released. The pin should be connected to \ref halui.estop.activate.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory as registered in
    //! \ref WhbHal::halInit(const WhbSoftwareButton* , size_t , const WhbKeyCodes&)
    //! \sa doEmergencyStop
    void setReset(bool enabled);
    //! \sa setReset(bool, size_t)
    void setStop(bool enabled);
    //! Toggles the start/pause/resume states.
    //! \param enabled true if start/resume is requested, false otherwise
    //! \param pinNumber \sa setReset(bool, size_t)
    void setStart(bool enabled);
    //! \sa setReset(bool, size_t)
    void setFeedPlus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setFeedMinus(bool enabled);

    // \xrefitem HalMemory::Out::feedOverrideCounts setter
    void setFeedOverrideCounts(hal_s32_t counts);
    // \xrefitem HalMemory::Out::feedOverrideScale setter
    void setFeedOverrideScale(hal_float_t scale);
    // \xrefitem HalMemory::Out::feedOverrideDirectValue setter
    void setFeedOverrideDirectValue(bool enabled);

    void setFeedValueSelected0_001(bool selected);
    void setFeedValueSelected0_01(bool selected);
    void setFeedValueSelected0_1(bool selected);
    void setFeedValueSelected1_0(bool selected);

    //! \sa setReset(bool, size_t)
    void setSpindlePlus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setSpindleMinus(bool enabled);
    //! \sa setReset(bool, size_t)
    void setFunction(bool enabled);
    //! \sa setReset(bool, size_t)
    void setMachineHome(bool enabled);
    //! \sa setReset(bool, size_t)
    void setSafeZ(bool enabled);
    //! \sa setReset(bool, size_t)
    void setWorkpieceHome(bool enabled);
    //! \sa setReset(bool, size_t)
    void setSpindleOn(bool enabled);
    //! \sa setReset(bool, size_t)
    void setProbeZ(bool enabled);
    //! \sa setReset(bool, size_t)
    void setContinuousMode(bool enabled);
    //! \sa setReset(bool, size_t)
    void setStepMode(bool enabled);
    //! Sets the hal state of the macro pin. Usually called in case the macro
    //! button is pressed or released. A macro button can be any button
    //! when pressed together with the modifier key.
    //! \param enabled the new pin value, (true if the button was pressed, false otherwise)
    //! \param pinNumber The pin number in \ref WhbHalMemory.
    //! \sa setReset(bool, size_t)
    void setMacro1(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro2(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro3(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro4(bool enabled);
    //! \sa setMacro1(bool)
    void setMacro5(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro6(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro7(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro8(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro9(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro10(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro11(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro12(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro13(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro14(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro15(bool enabled);
    //! \sa setMacro1(bool, size_t)
    void setMacro16(bool enabled);

    /*
    //! This method cumulates new jog dial delta. In other words it produces counts
    //! to be consumed by an icomp component.
    //! \param delta new jog dial delta
    void newJogDialDelta(int8_t delta);
     */

   /* //! Toggles true/false on the corresponding halui.jog.jog{increment|}Plus according to the \xrefitem mStepMode.
    //! \param direction positive if > 0, negative if < 0, jo jog but reset signals to false otherwise
    //! \return true if the step was consumed (on false to true transition)
    bool jogStep(int8_t direction);
*/
    void doJogCounts(int32_t counts);

    //! Returns the axis position.
    //! \param absolute true absolute, false relative
    //! \return the absolute or relative position in machine units
    hal_float_t getAxisXPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisYPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisZPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisAPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisBPosition(bool absolute) const;
    //! \xrefitem getAxisXPosition(bool)
    hal_float_t getAxisCPosition(bool absolute) const;

private:
    bool mIsSimulationMode;
    const char* mName;
    const char* mComponentPrefix;
    int          mHalCompId;
    std::ostream mDevNull;
    std::ostream* mHalCout;
    HandwheelStepmodes::Mode mStepMode;
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
    //! \sa setPin(bool, size_t, const char*)
    void setPin(bool enabled, const char* pinName);
    //! Toggles program states; running, paused, resume.
    //! Should be called each time after setStart(true) (\sa setStart(bool)) to stay in sync.
    void toggleStartResumeProgram();

    void clearStartResumeProgramStates();

    void enableManualMode(bool isRisingEdge);

    void enableMdiMode(bool isRisingEdge);

    //! Jogs (toggles) one of the first two given arguments according to the given direction.
    //! Sets each other argument to false/0.
    //! \param jogPlus first jog candidate
    //! \param jogMinus second jog candidate
    //! \param otherJogPlus other candidate that will be set to false
    //! \param otherJogMinus another candidate that will be set to false
    //! \param direction jog direction plus if > 0, minus if < 0, reset signals but no jog otherwise
    //! \return true on positive transition (false to true), false otherwise
    bool doJogToggleAndReset(hal_bit_t* jogPlus,
                             hal_bit_t* jogMinus,
                             hal_bit_t* otherJogPlus,
                             hal_bit_t* otherJogMinus,
                             int8_t direction);

};
}
