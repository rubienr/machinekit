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

// 3rd party includes

// local library includes
#include <hal_types.h>

// forward declarations

namespace XhcWhb04b6 {

// forward declarations
class WhbContext;
class WhbButtonsState;

// ----------------------------------------------------------------------

enum class JogWheelStepMode : uint8_t
{
    CONTINUOUS,
    STEP
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
    //enum class AxisIndexName : uint8_t
    //{
    //    Off = 0, X = 1, Y = 2, Z = 3, A = 4, B = 5, C = 6
    //};

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
    enum class PositionNameIndex : uint8_t
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
    enum class PositionNameIndex : uint8_t
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

    enum class PositionNameIndex : uint8_t
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
    JogWheelStepMode currentStepMode() const;
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
    JogWheelStepMode                                      mCurrentStepMode;
    uint8_t                                               mCurrentButton1Code;
    uint8_t                                               mCurrentButton2Code;
    const WhbSoftwareButton* mSoftwareButton;
    uint8_t mCurrentAxisCode;
    uint8_t mCurrentFeedCode;
    const WhbKeyCode* mCurrentAxisKeyCode;
    const WhbKeyCode* mCurrentFeedKeyCode;
};
}
