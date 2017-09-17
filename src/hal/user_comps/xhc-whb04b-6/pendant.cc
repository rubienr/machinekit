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
#include <iostream>
#include <iomanip>

// 3rd party includes

// local library includes

// local includes
#include "./hal.h"
#include "./xhc-whb04b6.h"

using std::endl;

namespace XhcWhb04b6 {

// ----------------------------------------------------------------------

const ButtonsCode              KeyCodes::Buttons;
const MetaButtonsCodes         KeyCodes::Meta(Buttons);
const AxisRotaryButtonCodes    KeyCodes::Axis;
const FeedRotaryButtonCodes    KeyCodes::Feed;

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

std::ostream& operator<<(std::ostream& os, const WhbKeyCode& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << std::hex << std::setfill('0') << "code=0x" << std::setw(2) << static_cast<uint16_t>(data.code)
       << " text='" << data.text << "'"
       << " altText='" << data.altText << "'";

    os.copyfmt(init);
    return os;
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
    return mSequence[static_cast<int8_t>(buttonPosition)];
}

// ----------------------------------------------------------------------

WhbHandwheelStepModeStepSize::WhbHandwheelStepModeStepSize() :
    mSequence{0.001, 0.01, 0.1, 1.0, 0, 0, 0, 0}
{
}

// ----------------------------------------------------------------------

uint8_t WhbHandwheelContinuousModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<int8_t>(buttonPosition)];
}

// ----------------------------------------------------------------------

WhbHandwheelContinuousModeStepSize::WhbHandwheelContinuousModeStepSize() :
    mSequence{2, 5, 10, 30, 60, 100, 0, 0}
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

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------

KeyEventListener::~KeyEventListener()
{
}

// ----------------------------------------------------------------------

HandwheelStepModeStepSize::HandwheelStepModeStepSize() :
    mSequence{0.001, 0.01, 0.1, 1, 0, 0, 0}
{
}

// ----------------------------------------------------------------------

HandwheelStepModeStepSize::~HandwheelStepModeStepSize()
{
}

// ----------------------------------------------------------------------

float HandwheelStepModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}

// ----------------------------------------------------------------------

bool HandwheelStepModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}

// ----------------------------------------------------------------------

HandwheelContinuousModeStepSize::HandwheelContinuousModeStepSize() :
    mSequence{2, 5, 10, 30, 60, 100, 0}
{
}

// ----------------------------------------------------------------------

HandwheelContinuousModeStepSize::~HandwheelContinuousModeStepSize()
{
}

// ----------------------------------------------------------------------

float HandwheelContinuousModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}

// ----------------------------------------------------------------------

bool HandwheelContinuousModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}

// ----------------------------------------------------------------------

HandwheelLeadModeStepSize::HandwheelLeadModeStepSize() :
    mSequence{0, 0, 0, 0, 0, 0, 1, 0}
{
}

// ----------------------------------------------------------------------

HandwheelLeadModeStepSize::~HandwheelLeadModeStepSize()
{
}

// ----------------------------------------------------------------------

float HandwheelLeadModeStepSize::getStepSize(PositionNameIndex buttonPosition) const
{
    return mSequence[static_cast<uint8_t>(buttonPosition)];
}

// ----------------------------------------------------------------------

bool HandwheelLeadModeStepSize::isPermitted(PositionNameIndex buttonPosition) const
{
    return (getStepSize(buttonPosition) > 0);
}

// ----------------------------------------------------------------------

KeyCode::KeyCode(uint8_t code, const char* text, const char* altText) :
    code(code),
    text(text),
    altText(altText)
{
}

// ----------------------------------------------------------------------

KeyCode::KeyCode(const KeyCode& other) :
    code(other.code),
    text(other.text),
    altText(other.altText)
{
}

// ----------------------------------------------------------------------

bool KeyCode::operator==(const KeyCode& other) const
{
    return ((code == other.code) && (text == other.text) && (altText == other.altText));
}

// ----------------------------------------------------------------------

bool KeyCode::operator!=(const KeyCode& other) const
{
    return !(*this == other);
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const KeyCode& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << std::hex << std::setfill('0') << "code=0x" << std::setw(2) << static_cast<uint16_t>(data.code)
       << " text='" << data.text << "'"
       << " altText='" << data.altText << "'";

    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

MetaButtonCodes::MetaButtonCodes(const KeyCode& key, const KeyCode& modifier) :
    key(key),
    modifier(modifier)
{
}

// ----------------------------------------------------------------------

MetaButtonCodes::~MetaButtonCodes()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const MetaButtonCodes& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "key={" << data.key << "} modifier={" << data.modifier << "}";
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

bool MetaButtonCodes::containsKeys(const KeyCode& key, const KeyCode& modifier) const
{
    return (this->key.code == key.code) && (this->modifier.code == modifier.code);
}

// ----------------------------------------------------------------------

MetaButtonsCodes::MetaButtonsCodes(const ButtonsCode& buttons) :
    buttons{
        {new MetaButtonCodes(buttons.reset, buttons.undefined)},
        {new MetaButtonCodes(buttons.reset, buttons.function)},
        {new MetaButtonCodes(buttons.stop, buttons.undefined)},
        {new MetaButtonCodes(buttons.stop, buttons.function)},
        {new MetaButtonCodes(buttons.start, buttons.undefined)},
        {new MetaButtonCodes(buttons.start, buttons.function)},
        {new MetaButtonCodes(buttons.feed_plus, buttons.undefined)},
        {new MetaButtonCodes(buttons.feed_plus, buttons.function)},
        {new MetaButtonCodes(buttons.feed_minus, buttons.undefined)},
        {new MetaButtonCodes(buttons.feed_minus, buttons.function)},
        {new MetaButtonCodes(buttons.spindle_plus, buttons.undefined)},
        {new MetaButtonCodes(buttons.spindle_plus, buttons.function)},
        {new MetaButtonCodes(buttons.spindle_minus, buttons.undefined)},
        {new MetaButtonCodes(buttons.spindle_minus, buttons.function)},
        {new MetaButtonCodes(buttons.machine_home, buttons.undefined)},
        {new MetaButtonCodes(buttons.machine_home, buttons.function)},
        {new MetaButtonCodes(buttons.safe_z, buttons.undefined)},
        {new MetaButtonCodes(buttons.safe_z, buttons.function)},
        {new MetaButtonCodes(buttons.workpiece_home, buttons.undefined)},
        {new MetaButtonCodes(buttons.workpiece_home, buttons.function)},
        {new MetaButtonCodes(buttons.spindle_on_off, buttons.undefined)},
        {new MetaButtonCodes(buttons.spindle_on_off, buttons.function)},
        {new MetaButtonCodes(buttons.function, buttons.undefined)},
        {new MetaButtonCodes(buttons.probe_z, buttons.undefined)},
        {new MetaButtonCodes(buttons.probe_z, buttons.function)},
        {new MetaButtonCodes(buttons.macro10, buttons.undefined)},
        {new MetaButtonCodes(buttons.macro10, buttons.function)},
        {new MetaButtonCodes(buttons.manual_pulse_generator, buttons.undefined)},
        {new MetaButtonCodes(buttons.manual_pulse_generator, buttons.function)},
        {new MetaButtonCodes(buttons.step_continuous, buttons.undefined)},
        {new MetaButtonCodes(buttons.step_continuous, buttons.function)},
        {new MetaButtonCodes(buttons.undefined, buttons.undefined)}
    }
{
}

// ----------------------------------------------------------------------

MetaButtonsCodes::~MetaButtonsCodes()
{
    for (const MetaButtonCodes* b : buttons)
    {
        delete b;
    }
}

// ----------------------------------------------------------------------

Button::Button(const KeyCode& key) :
    mKey(&key)
{
}

// ----------------------------------------------------------------------

Button::~Button()
{
}

// ----------------------------------------------------------------------

Button& Button::operator=(const Button& other)
{
    mKey = other.mKey;
    return *this;
}

// ----------------------------------------------------------------------

AxisRotaryButtonCodes::AxisRotaryButtonCodes() :
    off(0x06, "OFF", ""),
    x(0x11, "X", ""),
    y(0x12, "Y", ""),
    z(0x13, "Z", ""),
    a(0x14, "A", ""),
    b(0x15, "B", ""),
    c(0x16, "C", ""),
    undefined(0x00, "", ""),
    codeMap{
        {off.code, &off},
        {x.code,   &x},
        {y.code,   &y},
        {z.code,   &z},
        {a.code,   &a},
        {b.code,   &b},
        {c.code,   &c}
    }
{
}

// ----------------------------------------------------------------------

FeedRotaryButtonCodes::FeedRotaryButtonCodes() :
    speed_0_001(0x0d, "0.001", "2%"),
    speed_0_01(0x0e, "0.01", "5%"),
    speed_0_1(0x0f, "0.1", "10%"),
    speed_1(0x10, "1", "30%"),
    percent_60(0x1a, "", "60%"),
    percent_100(0x1b, "", "100%"),
    lead(0x1c, "Lead", ""),
    undefined(0x00, "", ""),
    codeMap{
        {speed_0_001.code, &speed_0_001},
        {speed_0_01.code,  &speed_0_01},
        {speed_0_1.code,   &speed_0_1},
        {speed_1.code,     &speed_1},
        {percent_60.code,  &percent_60},
        {percent_100.code, &percent_100},
        {lead.code,        &lead},
        {undefined.code,   &undefined}
    }
{
}

// ----------------------------------------------------------------------

ButtonsCode::ButtonsCode() :
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
    undefined(0x00, "", ""),
    codeMap{
        {reset.code,                  &reset},
        {stop.code,                   &stop},
        {start.code,                  &start},
        {feed_plus.code,              &feed_plus},
        {feed_minus.code,             &feed_minus},
        {spindle_plus.code,           &spindle_plus},
        {spindle_minus.code,          &spindle_minus},
        {machine_home.code,           &machine_home},
        {safe_z.code,                 &safe_z},
        {workpiece_home.code,         &workpiece_home},
        {spindle_on_off.code,         &spindle_on_off},
        {function.code,               &function},
        {probe_z.code,                &probe_z},
        {macro10.code,                &macro10},
        {manual_pulse_generator.code, &manual_pulse_generator},
        {undefined.code,              &undefined}
    }
{
}


// ----------------------------------------------------------------------

const KeyCode& Button::keyCode() const
{
    return *mKey;
}

// ----------------------------------------------------------------------

void Button::setKeyCode(KeyCode& keyCode)
{
    mKey = &keyCode;
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Button& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << "key={" << data.keyCode() << "}";
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

ToggleButton::ToggleButton(const KeyCode& key, const KeyCode& modifier) :
    Button(key),
    mModifier(&modifier)
{
}

// ----------------------------------------------------------------------

ToggleButton::~ToggleButton()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const ToggleButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << *static_cast<const Button*>(&data) << " modifier={" << data.modifierCode() << "}";
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

ToggleButton& ToggleButton::operator=(const ToggleButton& other)
{
    Button::operator=(other);
    mModifier = other.mModifier;
    return *this;
}

// ----------------------------------------------------------------------

const KeyCode& ToggleButton::modifierCode() const
{
    return *mModifier;
}

// ----------------------------------------------------------------------

void ToggleButton::setModifierCode(KeyCode& modifierCode)
{
    mModifier = &modifierCode;
}

// ----------------------------------------------------------------------

bool ToggleButton::containsKeys(const KeyCode& key, const KeyCode& modifier) const
{
    return ((key.code == mKey->code) && (modifier.code == mModifier->code));
}

// ----------------------------------------------------------------------

RotaryButton::RotaryButton(const KeyCode& keyCode) :
    Button(keyCode)
{
}

// ----------------------------------------------------------------------

RotaryButton::~RotaryButton()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const RotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << *static_cast<const Button*>(&data);
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

RotaryButton& RotaryButton::operator=(const RotaryButton& other)
{
    Button::operator=(other);
    return *this;
}

// ----------------------------------------------------------------------
const HandwheelStepModeStepSize       FeedRotaryButton::mStepStepSizeMapper;
const HandwheelContinuousModeStepSize FeedRotaryButton::mContinuousSizeMapper;
const HandwheelLeadModeStepSize       FeedRotaryButton::mLeadStepSizeMapper;

const std::map<const KeyCode*, HandwheelStepModeStepSize::PositionNameIndex>       FeedRotaryButton::mStepKeycodeLut{
    {&KeyCodes::Feed.speed_0_001, HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0001},
    {&KeyCodes::Feed.speed_0_01,  HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0010},
    {&KeyCodes::Feed.speed_0_1,   HandwheelStepModeStepSize::PositionNameIndex::RotaryButton0100},
    {&KeyCodes::Feed.speed_1,     HandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined},
    {&KeyCodes::Feed.percent_60,  HandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined},
    {&KeyCodes::Feed.percent_100, HandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined},
    {&KeyCodes::Feed.lead,        HandwheelStepModeStepSize::PositionNameIndex::RotaryButtonUndefined}
};
const std::map<const KeyCode*, HandwheelContinuousModeStepSize::PositionNameIndex> FeedRotaryButton::mContinuousKeycodeLut{
    {&KeyCodes::Feed.speed_0_001, HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton2percent},
    {&KeyCodes::Feed.speed_0_01,  HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton5percent},
    {&KeyCodes::Feed.speed_0_1,   HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton10percent},
    {&KeyCodes::Feed.speed_1,     HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton30percent},
    {&KeyCodes::Feed.percent_60,  HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton60percent},
    {&KeyCodes::Feed.percent_100, HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton100percent},
    {&KeyCodes::Feed.lead,        HandwheelContinuousModeStepSize::PositionNameIndex::RotaryButtonUndefined}
};
const std::map<const KeyCode*, HandwheelLeadModeStepSize::PositionNameIndex>       FeedRotaryButton::mLeadKeycodeLut{
    {&KeyCodes::Feed.speed_0_001, HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.speed_0_01,  HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.speed_0_1,   HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.speed_1,     HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_60,  HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.percent_100, HandwheelLeadModeStepSize::PositionNameIndex::UNDEFINED},
    {&KeyCodes::Feed.lead,        HandwheelLeadModeStepSize::PositionNameIndex::LEAD}
};

// ----------------------------------------------------------------------

FeedRotaryButton::FeedRotaryButton(const KeyCode& keyCode,
                                   HandwheelStepmodes::Mode stepMode,
                                   KeyEventListener* listener) :
    RotaryButton(keyCode),
    mStepMode(stepMode),
    mIsPermitted(false),
    mStepSize(0),
    mEventListener(listener)
{
}

// ----------------------------------------------------------------------

FeedRotaryButton::~FeedRotaryButton()
{
}


// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const FeedRotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << *static_cast<const RotaryButton*>(&data) << " "
       << "isPermitted=" << ((data.isPermitted()) ? "TRUE" : "FALSE") << " "
       << "stepSize=" << data.stepSize() << " "
       << "stepMode=0x" << std::setfill('0') << std::hex << std::setw(2)
       << static_cast<std::underlying_type<HandwheelStepmodes::Mode>::type>(data.stepMode());
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

FeedRotaryButton& FeedRotaryButton::operator=(const FeedRotaryButton& other)
{
    RotaryButton::operator=(other);
    mStepMode = other.mStepMode;
    return *this;
}

// ----------------------------------------------------------------------

void FeedRotaryButton::setStepMode(HandwheelStepmodes::Mode stepMode)
{
    mStepMode = stepMode;
    update();
}

// ----------------------------------------------------------------------

HandwheelStepmodes::Mode FeedRotaryButton::stepMode() const
{
    return mStepMode;
}

// ----------------------------------------------------------------------

float FeedRotaryButton::stepSize() const
{
    return mStepSize;
}

// ----------------------------------------------------------------------

void FeedRotaryButton::update()
{
    if (mStepMode == HandwheelStepmodes::Mode::CONTINUOUS)
    {
        auto enumValue = mContinuousKeycodeLut.find(mKey);
        assert(enumValue != mContinuousKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mContinuousSizeMapper.getStepSize(second);
        mIsPermitted = mContinuousSizeMapper.isPermitted(second);
    }
    else if (mStepMode == HandwheelStepmodes::Mode::STEP)
    {
        auto enumValue = mStepKeycodeLut.find(mKey);
        assert(enumValue != mStepKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mStepStepSizeMapper.getStepSize(second);
        mIsPermitted = mStepStepSizeMapper.isPermitted(second);
    }
    else if (mStepMode == HandwheelStepmodes::Mode::LEAD)
    {
        auto enumValue = mLeadKeycodeLut.find(mKey);
        assert(enumValue != mLeadKeycodeLut.end());
        auto second = enumValue->second;
        mStepSize    = mLeadStepSizeMapper.getStepSize(second);
        mIsPermitted = mLeadStepSizeMapper.isPermitted(second);
    }
    else
    {
        assert(false);
    }
}

// ----------------------------------------------------------------------

bool FeedRotaryButton::isPermitted() const
{
    return mIsPermitted;
}

// ----------------------------------------------------------------------

AxisRotaryButton::AxisRotaryButton(const KeyCode& keyCode, KeyEventListener* listener) :
    RotaryButton(keyCode),
    mEventListener(listener)
{
}

// ----------------------------------------------------------------------

AxisRotaryButton::~AxisRotaryButton()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const AxisRotaryButton& data)
{
    std::ios init(NULL);
    init.copyfmt(os);
    os << *static_cast<const RotaryButton*>(&data)
       << " isPermitted=" << ((data.isPermitted()) ? "TRUE" : "FALSE");
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

AxisRotaryButton& AxisRotaryButton::operator=(const AxisRotaryButton& other)
{
    RotaryButton::operator=(other);
    return *this;
}

// ----------------------------------------------------------------------

bool AxisRotaryButton::isPermitted() const
{
    return (mKey != &KeyCodes::Axis.undefined);
}

// ----------------------------------------------------------------------

Handwheel::Handwheel(const FeedRotaryButton& feedButton, KeyEventListener* listener) :
    mCounts(0),
    mFeedButton(feedButton),
    mEventListener(listener)
{
}

// ----------------------------------------------------------------------

Handwheel::~Handwheel()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Handwheel& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << "counts={" << data.counts() << "}";
    return os;
}

// ----------------------------------------------------------------------

int32_t Handwheel::consumeScaledCounts()
{
    int32_t tmp = mCounts;
    mCounts = 0;

    if (mFeedButton.stepMode() == HandwheelStepmodes::Mode::STEP)
    {
        if (mFeedButton.isPermitted())
        {
            return tmp * mFeedButton.stepSize();
        }
        return 0;
    }
    return tmp;
}

// ----------------------------------------------------------------------

int32_t Handwheel::counts() const
{
    return mCounts;
}

// ----------------------------------------------------------------------

void Handwheel::produceCount(int8_t counts)
{
    mCounts += counts;
}

// ----------------------------------------------------------------------

ButtonsState::ButtonsState(KeyEventListener* listener, const ButtonsState* previousState) :
    mPressedButtons(),
    mCurrentMetaButton(KeyCodes::Meta.buttons.back()),
    mAxisButton(KeyCodes::Axis.undefined, listener),
    mFeedButton(KeyCodes::Feed.undefined, HandwheelStepmodes::Mode::CONTINUOUS, listener),
    mPreviousState(previousState),
    mEventListener(listener)
{
}

// ----------------------------------------------------------------------

ButtonsState::~ButtonsState()
{
}

// ----------------------------------------------------------------------

void ButtonsState::update(const KeyCode& keyCode,
                          const KeyCode& modifierCode,
                          const KeyCode& axisButton,
                          const KeyCode& feedButton)
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const ButtonsState& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << "pressed buttons={";
    for (const KeyCode* pb : data.pressedButtons())
    {
        assert(pb != nullptr);
        os << *pb << " ";
    }
    os << "} metaButton={" << *data.currentMetaButton()
       << "} axisButton={" << data.axisButton()
       << "} feedButton={" << data.feedButton() << "}";
    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

ButtonsState& ButtonsState::operator=(const ButtonsState& other)
{
    mPressedButtons    = other.mPressedButtons;
    mCurrentMetaButton = other.mCurrentMetaButton;
    mAxisButton        = other.mAxisButton;
    mFeedButton        = other.mFeedButton;
    return *this;
}

// ----------------------------------------------------------------------

void ButtonsState::clearPressedButtons()
{
    mPressedButtons.clear();
}

// ----------------------------------------------------------------------

const std::list<const KeyCode*>& ButtonsState::pressedButtons() const
{
    return mPressedButtons;
}

// ----------------------------------------------------------------------

const MetaButtonCodes* ButtonsState::currentMetaButton() const
{
    return mCurrentMetaButton;
}

// ----------------------------------------------------------------------

const AxisRotaryButton& ButtonsState::axisButton() const
{
    return mAxisButton;
}

// ----------------------------------------------------------------------

const FeedRotaryButton& ButtonsState::feedButton() const
{
    return mFeedButton;
}

// ----------------------------------------------------------------------

Pendant::Pendant() :
    mPreviousButtonsState(this),
    mCurrentButtonsState(this, &mPreviousButtonsState),
    mHandWheel(mCurrentButtonsState.feedButton(), this),
    mPrefix("pndnt "),
    mPendantCout(&std::cout)
{
    *mPendantCout << mPrefix << *this << endl;
}

// ----------------------------------------------------------------------

Pendant::~Pendant()
{
}

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Pendant& data)
{
    std::ios init(NULL);
    init.copyfmt(os);

    os << "currentButtonState={" << data.currentButtonsState() << "} "
       << "previousButtonState={" << data.previousButtonsState() << "} "
       << "handwheel={ " << data.handWheel() << "}";
    return os;
}

// ----------------------------------------------------------------------

void Pendant::update(uint8_t keyCode,
                     uint8_t modifierCode,
                     uint8_t rotaryButtonAxisKeyCode,
                     uint8_t rotaryButtonFeedKeyCode,
                     int8_t handWheelStepCount)
{
}

// ----------------------------------------------------------------------

void Pendant::update(const KeyCode& keyCode,
                     const KeyCode& modifierCode,
                     const KeyCode& rotaryButtonAxisKeyCode,
                     const KeyCode& rotaryButtonFeedKeyCode,
                     int8_t handWheelStepCount)
{
    mCurrentButtonsState.update(keyCode, modifierCode, rotaryButtonAxisKeyCode, rotaryButtonFeedKeyCode);
    mHandWheel.produceCount(handWheelStepCount);
}

// ----------------------------------------------------------------------

void Pendant::shiftButtonState()
{
    mPreviousButtonsState = mCurrentButtonsState;
    mCurrentButtonsState.clearPressedButtons();
}

// ----------------------------------------------------------------------

const ButtonsState& Pendant::currentButtonsState() const
{
    return mCurrentButtonsState;
}

// ----------------------------------------------------------------------

const ButtonsState& Pendant::previousButtonsState() const
{
    return mPreviousButtonsState;
}

// ----------------------------------------------------------------------

const Handwheel& Pendant::handWheel() const
{
    return mHandWheel;
}

// ----------------------------------------------------------------------

bool Pendant::onButtonPressedEvent(const MetaButtonCodes& metaButton)
{
    *mPendantCout << mPrefix << "button pressed  event" << metaButton;
    return true;
}

// ----------------------------------------------------------------------

bool Pendant::onButtonReleasedEvent(const MetaButtonCodes& metaButton)
{
    *mPendantCout << mPrefix << "button released event" << metaButton;
    return true;
}

// ----------------------------------------------------------------------

void Pendant::onAxisActiveEvent(const KeyCode& axis)
{
    *mPendantCout << mPrefix << "axis   active   event" << axis;
}

// ----------------------------------------------------------------------

void Pendant::onAxisInactiveEvent(const KeyCode& axis)
{
    *mPendantCout << mPrefix << "axis   inactive event" << axis;
}

// ----------------------------------------------------------------------

void Pendant::onFeedActiveEvent(const KeyCode& feed)
{
    *mPendantCout << mPrefix << "feed   active   event" << feed;
}

// ----------------------------------------------------------------------

void Pendant::onFeedInactiveEvent(const KeyCode& feed)
{
    *mPendantCout << mPrefix << "feed   inactive event" << feed;
}

// ----------------------------------------------------------------------

void Pendant::onJogDialEvent(int8_t delta)
{
    *mPendantCout << mPrefix << "wheel  event " << static_cast<int16_t>(delta);
}

// ----------------------------------------------------------------------

// ----------------------------------------------------------------------

}
