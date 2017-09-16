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

#include "pendant.h"

// system includes
#include <assert.h>

// 3rd party includes

// local library includes

// local includes
#include "./hal.h"
#include "./xhc-whb04b6.h"

namespace XhcWhb04b6 {

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

JogWheelStepMode WhbButtonsState::currentStepMode() const
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
}
