/*
   XHC-WHB04B-6 Wireless MPG pendant LinuxCNC HAL module for LinuxCNC

   Copyright (C) 2017 Raoul Rubien (github.com/rubienr)
   Copyright (C) 2014 Marius Alksnys (marius.alksnys@gmail.com)
   Copyright (C) 2013 Frederic Rible (frible@teaser.fr)
   Copyright (C) 2013 Rene Hopf (renehopf@mac.com)

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
#include  <iomanip>
#include <string.h>
#include "rtapi_math.h"
#include <assert.h>
#include <signal.h>
#include <string.h>
#include <libusb.h>
#include <unistd.h>
#include <stdarg.h>
#include <hal.h>
#include <inifile.hh>
#include "config.h"
#include "../lib/hal.h"
#include <google/protobuf/stubs/common.h>

using std::cout;
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

    class WhbHalEntity;

    class WhbHal;

    class WhbSleepDetect;

    class WhbKeyCode;

    class WhbSoftwareButton;

    class WhbAxisRotaryButtonCodes;

    class WhbFeedRotaryButtonCodes;

    class WhbButtonsState;

    class WhbHandwheelStepModeStepSize;

    class WhbHandwheelContiunuousModeStepSize;

    class WhbButtonsCode;

    class WhbStepModeStepSize;

    class WhbEntity;

    class WhbKeyCodes;

    class WhbVelocityComputation;

    class WhbStepHandler;

    class WhbUsb;

    class WhbContext;

    class UsbInputPackageHandler;

// ----------------------------------------------------------------------

//! HAL memory pointers. Each pointer represents an i/o hal pin.
    struct WhbHalMemory
    {
        hal_float_t* xWorkpieceCoordinate;
        hal_float_t* yWorkpieceCoordinate;
        hal_float_t* zWorkpieceCoordinate;
        hal_float_t* aWorkpieceCoordinate;
        hal_float_t* bWorkpieceCoordinate;
        hal_float_t* cWorkpieceCoordinate;
        hal_float_t* xMachineCoordinate;
        hal_float_t* yMachineCoordinate;
        hal_float_t* zMachineCoordinate;
        hal_float_t* aMachineCoordinate;
        hal_float_t* bMachineCoordinate;
        hal_float_t* cMachineCoordinate;

        hal_float_t* feedrateOverride;
        hal_float_t* feedrate;
        hal_float_t* spindleOverride;
        hal_float_t* spindleRps;

        hal_bit_t* button_pin[64];

        hal_bit_t  * jogEnableOff;
        hal_bit_t  * jogEnableX;
        hal_bit_t  * jogEnableY;
        hal_bit_t  * jogEnableZ;
        hal_bit_t  * jogEnableA;
        hal_bit_t  * jogEnableB;
        hal_bit_t  * jogEnableC;
        hal_float_t* jogScale;
        hal_s32_t  * jogCount;
        hal_s32_t  * jogCountNeg;
        hal_float_t* jogVelocity;
        hal_float_t* jogMaxVelocity;
        hal_float_t* jogIncrement;
        hal_bit_t  * jogPlusX;
        hal_bit_t  * jogPlusY;
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

        hal_bit_t* stepsizeUp;
        hal_s32_t* stepsize;
        hal_bit_t* sleeping;
        hal_bit_t* isPendantConnected;
        hal_bit_t* isPendantRequired;

        WhbHalMemory() :
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
            spindleOverride(nullptr),
            spindleRps(nullptr),
            button_pin{0},
            jogEnableOff(nullptr),
            jogEnableX(nullptr),
            jogEnableY(nullptr),
            jogEnableZ(nullptr),
            jogEnableA(nullptr),
            jogEnableB(nullptr),
            jogEnableC(nullptr),
            jogScale(nullptr),
            jogCount(nullptr),
            jogCountNeg(nullptr),
            jogVelocity(nullptr),
            jogMaxVelocity(nullptr),
            jogIncrement(nullptr),
            jogPlusX(nullptr),
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
            jogMinusC(nullptr),
            stepsizeUp(nullptr),
            stepsize(nullptr),
            sleeping(nullptr),
            isPendantConnected(nullptr),
            isPendantRequired(nullptr)
        {}
    };

// ----------------------------------------------------------------------

//! HAL and related parameters
    class WhbHal
    {
    public:
        WhbHalMemory memory;

        WhbHal();

        ~WhbHal();

        //! initializes hal pins according to simulation mode, must not be called twice
        //! \ref setIsSimulationMode() must be set before accordingly
        void halInit(WhbSoftwareButton* softwareButtons,
                     size_t buttonsCount,
                     const WhbKeyCodes& codes);


        bool getIsSimulationMode() const;

        //! indicates the program has been invoked in hal mode or normal
        void setSimulationMode(bool isSimulationMode);

        int getHalComponentId() const;

        const char* getHalComponentName() const;

    private:
        bool mIsSimulationMode;
        const char* mName;
        int mHalCompId;


        //! allocates new hal pin according to \ref mIsSimulationMode
        int newSimulatedHalPin(char* pin_name, void** ptr, int s);

        //! allocates new hal_float_t according to \ref mIsSimulationMode
        int newFloatHalPin(hal_pin_dir_t dir, hal_float_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! allocates new hal_s32_t pin according to \ref mIsSimulationMode
        int newSigned32HalPin(hal_pin_dir_t dir, hal_s32_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! allocates new hal_bit_t pin according to \ref mIsSimulationMode
        int newBitHalPin(hal_pin_dir_t dir, hal_bit_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! allocates new hal pin according to \ref mIsSimulationMode
        void freeSimulatedPin(void** pin);
    };

// ----------------------------------------------------------------------

//! pendant sleep/idle state parameters
    class WhbSleepDetect
    {
        friend XhcWhb04b6::WhbUsb;

        bool           dropNextInPackage;
        struct timeval last_wakeup;

    public:
        WhbSleepDetect() :
            dropNextInPackage(false),
            last_wakeup()
        {}
    };

// ----------------------------------------------------------------------

//! pendant button key code description
    struct WhbKeyCode
    {
        const uint8_t code;
        //! default button text as written on pendant (if available)
        const char* text;
        //! alternative button text as written on pendant (if available)
        const char* altText;

        WhbKeyCode(uint8_t code, const char* text, const char* altText) :
            code(code),
            text(text),
            altText(altText)
        {}

        WhbKeyCode(const WhbKeyCode& other) :
            code(other.code),
            text(other.text),
            altText(other.altText)
        {}
    };

// ----------------------------------------------------------------------

//! meta-button state which is dependent on the "Fn" modifier button's state
    struct WhbSoftwareButton
    {
        const WhbKeyCode& key;
        const WhbKeyCode& modifier;

        WhbSoftwareButton(const WhbKeyCode& key, const WhbKeyCode& modifier) :
            key(key),
            modifier(modifier)
        {}
    };

// ----------------------------------------------------------------------

//! rotary axis selection button related parameters
    struct WhbAxisRotaryButtonCodes
    {
        typedef enum
        {
            Off = 0, X = 1, Y = 2, Z = 3, A = 4, B = 5, C = 6
        } AxisIndexName;

        const WhbKeyCode off;
        const WhbKeyCode x;
        const WhbKeyCode y;
        const WhbKeyCode z;
        const WhbKeyCode a;
        const WhbKeyCode b;
        const WhbKeyCode c;
        const WhbKeyCode undefined;

        WhbAxisRotaryButtonCodes() :
            off(0x06, "OFF", ""),
            x(0x11, "X", ""),
            y(0x12, "Y", ""),
            z(0x13, "Z", ""),
            a(0x14, "A", ""),
            b(0x15, "B", ""),
            c(0x16, "C", ""),
            undefined(0x00, "", "")
        {}
    };

// ----------------------------------------------------------------------

//! rotary feed button related parameters
    struct WhbFeedRotaryButtonCodes
    {
        const WhbKeyCode speed_0_001;
        const WhbKeyCode speed_0_01;
        const WhbKeyCode speed_0_1;
        const WhbKeyCode speed_1;
        const WhbKeyCode percent_60;
        const WhbKeyCode percent_100;
        const WhbKeyCode lead;
        const WhbKeyCode undefined;

        WhbFeedRotaryButtonCodes() :
            speed_0_001(0x0d, "0.001", "2%"),
            speed_0_01(0x0e, "0.01", "5%"),
            speed_0_1(0x0f, "0.1", "10%"),
            speed_1(0x10, "1", "30%"),
            percent_60(0x1a, "", "60%"),
            percent_100(0x1b, "", "100%"),
            lead(0x1c, "Lead", ""),
            undefined(0x00, "", "")
        {}
    };

// ----------------------------------------------------------------------

//! pendant button related parameters
    struct WhbButtonsCode
    {
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

        WhbButtonsCode() :
            reset(0x01, "RESET", "Macro-11"),
            stop(0x02, "STOP", "Macro-12"),
            start(0x03, "Start", "Pause"),
            feed_plus(0x04, "Feed+", "Macro-1"),
            feed_minus(0x05, "Feed-", "Macro-2"),
            spindle_plus(0x06, "Spindle+", "Macro-3"),
            spindle_minus(0x07, "Spindle-", "Macro-4"),
            machine_home(0x08, "M-HOME", "Macro-5"),
            safe_z(0x09, "Safe-Z", "Macro-6"),
            workpiece_home(0x0a, "W-HOME", "Macro-7"),
            spindle_on_off(0x0b, "S-ON/OFF", "Macro-8"),
            function(0x0c, "Fn", "Fnx"),
            probe_z(0x0d, "Probe-Z", "Macro-9"),
            macro10(0x10, "Macro-10", "Macro-13"),
            manual_pulse_generator(0x0e, "MPG", "Macro-14"),
            step_continuous(0x0f, "STEP", "Continuous"),
            undefined(0x00, "", "")
        {}

    };

// ----------------------------------------------------------------------

//! whb button and rotary button codes
    struct WhbKeyCodes
    {
    public:
        WhbButtonsCode           buttons;
        WhbAxisRotaryButtonCodes axis;
        WhbFeedRotaryButtonCodes feed;

        WhbKeyCodes() :
            buttons(),
            axis(),
            feed()
        {}
    };

// ----------------------------------------------------------------------

    struct WhbVelocityComputation
    {
    public:
        hal_s32_t      last_jog_counts;
        struct timeval last_tv;

        WhbVelocityComputation() :
            last_jog_counts(-1),
            last_tv()
        {}
    };

    // ----------------------------------------------------------------------

    //! If hand wheel is in step mode (toggled by Step/Continuous" button) this speed setting is applied.
    //! In step mode the step is distance.
    class WhbHandwheelStepModeStepSize
    {

    public:

        typedef enum
        {
            RotaryButton0001      = 0,
            RotaryButton0010      = 1,
            RotaryButton0100      = 2,
            RotaryButton100       = 3,
            RotaryButtonUndefined = 4,
        } ButtonCodeToStepIndex;

    private:

        const float sequence[5];
    public:


        float getStepSize(ButtonCodeToStepIndex buttonPosition) const
        {
            return sequence[buttonPosition];
        }

        WhbHandwheelStepModeStepSize() :
            sequence{0.001, 0.01, 0.1, 1.0, 0.0}
        {}
    };

    // ----------------------------------------------------------------------

    //! If hand wheel is in continuous mode (toggled by Step/Continuous" button) this speed setting is applied.
    //! In continuous mode the step speed is in percent.
    class WhbHandwheelContiunuousModeStepSize
    {
    public:

        typedef enum
        {
            RotaryButton2percent   = 0,
            RotaryButton5percent   = 1,
            RotaryButton10percent  = 2,
            RotaryButton30percent  = 3,
            RotaryButton60percent  = 4,
            RotaryButton100percent = 5,
            RotaryButtonUndefined  = 6
        } ButtonCodeToStepIndex;

    private:

        const uint8_t sequence[7];

    public:

        uint8_t getStepSize(ButtonCodeToStepIndex buttonPosition) const
        {
            return sequence[buttonPosition];
        }

        WhbHandwheelContiunuousModeStepSize() :
            sequence{2, 5, 10, 30, 60, 100, 0}
        {}
    };

    class WhbStepModeStepSize
    {
        friend XhcWhb04b6::WhbContext;
        friend XhcWhb04b6::WhbButtonsState;

        WhbHandwheelStepModeStepSize        step;
        WhbHandwheelContiunuousModeStepSize continuous;

    public:
        WhbStepModeStepSize() :
            step(),
            continuous()
        {}
    };

    // ----------------------------------------------------------------------

    //! step handling related parameters
    class WhbStepHandler
    {
        friend XhcWhb04b6::WhbContext;

        unsigned char old_inc_step_status;

    public:

        WhbStepModeStepSize stepSize;

        WhbStepHandler() :
            old_inc_step_status(0),
            stepSize()
        {}
    };

// ----------------------------------------------------------------------

    //! Buttons state struct reflecting i.e. current or previous button state,
    //! by storing up to two button codes and axis and feed codes. The software
    //! button reflects the logical combination of two button codes if the modifier
    //! button code "Fn" is involved.
    class WhbButtonsState
    {
        friend XhcWhb04b6::WhbContext;

        const WhbButtonsCode                     & buttonCodesLookup;
        const WhbAxisRotaryButtonCodes           & axisRotaryButtonCodesLookup;
        const WhbFeedRotaryButtonCodes           & feedRotaryButtonCodesLookup;
        const WhbHandwheelStepModeStepSize       & stepModeStepSizeLookup;
        const WhbHandwheelContiunuousModeStepSize& continuousModeStepSizeLookup;
        WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex currentContinuousModeSize;
        WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex        currentStepModeSize;
        uint8_t                                                    currentButton1Code;
        uint8_t                                                    currentButton2Code;
        WhbSoftwareButton* softwareButton;
        uint8_t currentAxisCode;
        uint8_t currentFeedCode;

        void setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex stepSize)
        {
            currentStepModeSize       = stepSize;
            currentContinuousModeSize = WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined;
        }

        void setCurrentContinuousModeStepSize(WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex stepSize)
        {
            currentContinuousModeSize = stepSize;
            currentStepModeSize       = WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined;
        }

    public:

        uint8_t getButton1Code() const
        {
            return currentButton1Code;
        }

        uint8_t getButton2Code() const
        {
            return currentButton2Code;
        }

        uint8_t getAxisRotaryButtonCode() const
        {
            return currentAxisCode;
        }

        uint8_t getFeedRotaryButtonCode() const
        {
            return currentFeedCode;
        }

        WhbSoftwareButton& getSoftwareButton() const
        {
            assert(softwareButton != nullptr);
            return *softwareButton;
        }

        //! stores the buttons state and updates the software button state and step speed state
        void updateButtonState(uint8_t button1Code,
                               uint8_t button2Code,
                               uint8_t currentAxisRotaryButtonCode,
                               uint8_t currentFeedRotaryButtonCode)
        {
            // find out modifier, see print button
            // then other key is the key
            // find the software button accordingly
            // update the speed rotary button, see print rotary button
            // update the axis rotary button

        }

        //! returns the step size value according to the current button state considering "Fn" modifer
        float getStepSize()
        {
            if (currentStepModeSize == WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined)
            {
                return continuousModeStepSizeLookup.getStepSize(currentContinuousModeSize);
            }
            else if (currentContinuousModeSize ==
                     WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined)
            {
                return stepModeStepSizeLookup.getStepSize(currentStepModeSize);
            }
            else
            {
                assert(false);
            }
        }

        //! initialized lookup references needed to resolve software button state and step speed state
        WhbButtonsState(const WhbButtonsCode& buttonCodesLookup,
                        const WhbAxisRotaryButtonCodes& axisRotaryButtonCodesLookup,
                        const WhbFeedRotaryButtonCodes& feedRotaryButtonCodesLookup,
                        const WhbHandwheelStepModeStepSize& stepSizeLookup,
                        const WhbHandwheelContiunuousModeStepSize& continuousStepSizeLookup) :
            buttonCodesLookup(buttonCodesLookup),
            axisRotaryButtonCodesLookup(axisRotaryButtonCodesLookup),
            feedRotaryButtonCodesLookup(feedRotaryButtonCodesLookup),
            stepModeStepSizeLookup(stepSizeLookup),
            continuousModeStepSizeLookup(continuousStepSizeLookup),
            currentContinuousModeSize(
                WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined),
            currentStepModeSize(
                WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined
            ),
            currentButton1Code(buttonCodesLookup.undefined.code),
            currentButton2Code(buttonCodesLookup.undefined.code),
            currentAxisCode(axisRotaryButtonCodesLookup.undefined.code),
            currentFeedCode(feedRotaryButtonCodesLookup.undefined.code)
        {}
    };

    // ----------------------------------------------------------------------

    //! convenience structure for accessing data in package stream
    class WhbUsbInPackage
    {
    public:
        const unsigned char notAvailable1;
        const unsigned char notAvailable2;
        const unsigned char buttonKeyCode1;
        const unsigned char buttonKeyCode2;
        const unsigned char rotaryButtonFeedKeyCode;
        const unsigned char rotaryButtonAxisKeyCode;
        const signed char   stepCount;
        const unsigned char crc;

        WhbUsbInPackage() :
            notAvailable1(0),
            notAvailable2(0),
            buttonKeyCode1(0),
            buttonKeyCode2(0),
            rotaryButtonFeedKeyCode(0),
            rotaryButtonAxisKeyCode(0),
            stepCount(0),
            crc(0)
        {}

        WhbUsbInPackage(const unsigned char notAvailable1,
                        const unsigned char notAvailable2,
                        const unsigned char buttonKeyCode1,
                        const unsigned char buttonKeyCode2,
                        const unsigned char rotaryButtonFeedKeyCode,
                        const unsigned char rotaryButtonAxisKeyCode,
                        const signed char stepCount,
                        const unsigned char crc) :
            notAvailable1(notAvailable1),
            notAvailable2(notAvailable2),
            buttonKeyCode1(buttonKeyCode1),
            buttonKeyCode2(buttonKeyCode2),
            rotaryButtonFeedKeyCode(rotaryButtonFeedKeyCode),
            rotaryButtonAxisKeyCode(rotaryButtonAxisKeyCode),
            stepCount(stepCount),
            crc(crc)
        {}
    };

    //! convenience structure for casting data in package stream
    union WhbUsbInPackageBuffer
    {
    public:
        const WhbUsbInPackage asFields;
        // todo: investigate why buffer size is not exactly 8 bytes as expected
        unsigned char         asBuffer[32];

        WhbUsbInPackageBuffer();
    };

    // ----------------------------------------------------------------------

    WhbUsbInPackageBuffer::WhbUsbInPackageBuffer() :
        asBuffer{0}
    {}

    //! This package is sent as last but one package before xhc-whb04-6 is powered off,
    //! and is meant to be used with operator== for comparison.
    class WhbUsbEmptyPackage :
        public WhbUsbInPackage
    {
    public:

        WhbUsbEmptyPackage() :
            WhbUsbInPackage(0x04, 0xff, 0, 0, 0, 0, 0, 0xff)
        {}

        //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
        bool operator==(const WhbUsbInPackage& other) const
        {
            // equality constraints: 0x4 0x? 0x0 0x0 0x0 0x0 0x0 0x?
            if ((notAvailable1 == other.notAvailable1)
                // && (notAvailable2 == other.notAvailable2)
                && (buttonKeyCode1 == other.buttonKeyCode1)
                && (buttonKeyCode2 == other.buttonKeyCode2)
                && (rotaryButtonFeedKeyCode == other.rotaryButtonFeedKeyCode)
                && (rotaryButtonAxisKeyCode == other.rotaryButtonAxisKeyCode)
                && (stepCount == other.stepCount)
                // && (crc == other.crc)
                )
            {
                return true;
            }
            return false;
        }

        //! \see operator==(const WhbUsbInPackage&)
        bool operator!=(const WhbUsbInPackage& other) const
        {
            return !((*this) == other);
        }
    };


    //! This package is sent as last package before xhc-whb04-6 is powered off,
    //! and is meant to be used with operator== for comparison.
    class WhbUsbSleepPackage :
        public WhbUsbInPackage
    {
    public:
        WhbUsbSleepPackage() :
            WhbUsbInPackage(0x04, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff)
        {}

        //! caution: it is not guaranteed that (this == \p other) == (\p other == this)
        bool operator==(const WhbUsbInPackage& other) const
        {
            // equality constraints: 0x4 0x? 0x? 0x? 0x? 0x? 0x? 0x?
            if ((notAvailable1 == other.notAvailable1)
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

        //! \see operator==(const WhbUsbInPackage&)
        bool operator!=(const WhbUsbInPackage& other) const
        {
            return !((*this) == other);
        }
    };

    //! set of constant usb packages
    class WhbConstantUsbPackages
    {
    public:
        const WhbUsbSleepPackage sleepPackage;
        const WhbUsbEmptyPackage emptyPackage;

        WhbConstantUsbPackages() :
            sleepPackage(),
            emptyPackage()
        {}
    };


    //! USB related parameters
    class WhbUsb
    {
        friend XhcWhb04b6::WhbContext;

        const uint16_t usbVendorId;
        const uint16_t usbProductId;
        libusb_context      * context;
        libusb_device_handle* deviceHandle;
        bool                  do_reconnect;
        bool                  wait_for_pendant_before_HAL;
        bool                  mIsSimulationMode;
        WhbSleepDetect        sleepState;
        bool                  mIsRunning;
        WhbUsbInPackageBuffer inputPackage;
        UsbInputPackageHandler& mDataHandler;
        WhbHalMemory          & mHalMemory;

    public:

        static const WhbConstantUsbPackages ConstantPackages;

        uint16_t getUsbVendorId() const;

        uint16_t getUsbProductId() const;

        const bool isDeviceOpen() const;

        libusb_context** getContextReference();

        libusb_context* getContext();

        void setContext(libusb_context* context);

        libusb_device_handle* getDeviceHandle();

        void setDeviceHandle(libusb_device_handle* deviceHandle);

        void setWaitForPendantBeforeHAL(bool waitForPendantBeforeHAL);

        bool getWaitForPendantBeforeHAL() const;

        bool getDoReconnect() const;

        void setDoReconnect(bool doReconnect);

        void cbResponseIn(struct libusb_transfer* transfer);

        void setSimulationMode(bool isSimulationMode);

        void setIsRunning(bool enableRunning);

        void requestTermination();

        void setupAsyncTransfer();

        WhbUsb(UsbInputPackageHandler& onDataReceivedCallback, WhbHalMemory& halMemory);
    };

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

    void WhbUsb::setWaitForPendantBeforeHAL(bool waitForPendantBeforeHAL)
    {
        wait_for_pendant_before_HAL = waitForPendantBeforeHAL;
    }

    // ----------------------------------------------------------------------

    bool WhbUsb::getWaitForPendantBeforeHAL() const
    {
        return wait_for_pendant_before_HAL;
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

    const WhbConstantUsbPackages WhbUsb::ConstantPackages;

    // ----------------------------------------------------------------------

    WhbUsb::WhbUsb(UsbInputPackageHandler& onDataReceivedCallback, WhbHalMemory& halMemory) :
        usbVendorId(0x10ce),
        usbProductId(0xeb93),
        context(nullptr),
        deviceHandle(nullptr),
        do_reconnect(false),
        wait_for_pendant_before_HAL(false),
        mIsSimulationMode(false),
        sleepState(),
        mIsRunning(false),
        inputPackage(),
        mDataHandler(onDataReceivedCallback),
        mHalMemory(halMemory)
    {
        gettimeofday(&sleepState.last_wakeup, nullptr);
    }

    // ----------------------------------------------------------------------

    class UsbInputPackageHandler
    {
    public:
        virtual void handleInputData(const WhbUsbInPackage& inPackage) = 0;

        virtual ~UsbInputPackageHandler()
        {}
    };

    //! program context
    class WhbContext :
        public UsbInputPackageHandler
    {
    public:
        WhbHal hal;

        //! \return the name as specified to \ref WhbContext
        const char* getName() const;

        //! \return the name as specified to \ref hal
        const char* getHalName() const;

        //! callback method received by \ref WhbUsb when a \ref libusb_transfer is received
        virtual void handleInputData(const WhbUsbInPackage& inPackage) override;

        //! todo: doxy
        void initWhb();

        //! todo: doxy
        void halInit();

        //! todo: doxy
        void halExit();

        //! todo: doxy
        void cbResponseIn(struct libusb_transfer* transfer);

        //! todo: doxy
        void setupAsyncTransfer();

        //! todo: doxy
        void computeVelocity();

        // todo: refactor me
        void xhcSetDisplay();

        //! todo: doxy
        void handleStep();

        //! todo: doxy
        void linuxcncSimulate();

        //! todo: doxy
        void requestTermination(int signal = -42);

        //! todo: doxy, move
        bool isRunning() const;

        void setSimulationMode(bool enableSimulationMode);

        WhbContext();

        virtual ~WhbContext();

        WhbUsb usb;
    private:
        const char* mName;
        const WhbKeyCodes      codes;
        WhbStepHandler         stepHandler;
        WhbSoftwareButton      softwareButtons[31];
        WhbButtonsState        previousButtonCodes;
        WhbButtonsState        currentButtonCodes;
        WhbVelocityComputation velocityComputation;
        bool                   doRun;
        bool                   mIsSimulationMode;

        //! todo: doxy
        int encodeFloat(float value, unsigned char* buffer);

        //! todo: doxy
        int encodeInt16(int16_t value, unsigned char* buffer);

        // todo: refactor me
        void xhcDisplayEncode(unsigned char* data, int len);


        //! todo: doxy
        void halTeardown();

        //! todo: doxy
        void printPushButtonText(uint8_t keyCode, uint8_t modifierCode);

        //! todo: doxy
        void printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode);

        //! todo: doxy
        void printData(const WhbUsbInPackage& inPackage);

        //! todo: doxy
        void printHexdump(const WhbUsbInPackage& inPackage);

    };

// -- begin garbage

#define STEPSIZE_BYTE 35

// the defines below were found for an 18 button device
// bit 'or' patterns for STEPSIZE_BYTE
#define DISPLAY_HOME_ICON           0x10
#define DISPLAY_HEIGHT_SETTING_ICON 0x20
#define DISPLAY_HEIGHT_UNKNOWN_40   0x40
#define DISPLAY_HEIGHT_UNKNOWN_80   0x80

#define STEPSIZE_DISPLAY_0          0x00
#define STEPSIZE_DISPLAY_1          0x01
#define STEPSIZE_DISPLAY_5          0x02
#define STEPSIZE_DISPLAY_10         0x03
#define STEPSIZE_DISPLAY_20         0x04
#define STEPSIZE_DISPLAY_30         0x05
#define STEPSIZE_DISPLAY_40         0x06
#define STEPSIZE_DISPLAY_50         0x07
#define STEPSIZE_DISPLAY_100        0x08
#define STEPSIZE_DISPLAY_500        0x09
#define STEPSIZE_DISPLAY_1000       0x0A
#define STEPSIZE_DISPLAY_P6         0x0B
#define STEPSIZE_DISPLAY_UNKNOWN_0C 0x0C
#define STEPSIZE_DISPLAY_UNKNOWN_0D 0x0D
#define STEPSIZE_DISPLAY_UNKNOWN_0E 0x0E
#define STEPSIZE_DISPLAY_UNKNOWN_0F 0x0F

// -- end garbage

    // ----------------------------------------------------------------------
    void WhbContext::halInit()
    {
        hal.halInit(softwareButtons, sizeof(softwareButtons) / sizeof(WhbSoftwareButton), codes);
    }

    // ----------------------------------------------------------------------

    void WhbContext::halExit()
    {
        halTeardown();
        hal_exit(hal.getHalComponentId());
    }

    // ----------------------------------------------------------------------

    const char* WhbContext::getName() const
    {
        return mName;
    }

    // ----------------------------------------------------------------------

    const char* WhbContext::getHalName() const
    {
        return hal.getHalComponentName();
    }

    // ----------------------------------------------------------------------

    void WhbContext::handleInputData(const WhbUsbInPackage& inPackage)
    {
        if (mIsSimulationMode)
        {
            printHexdump(inPackage);
        }

        uint8_t modifierCode       = codes.buttons.undefined.code;
        uint8_t keyCode            = codes.buttons.undefined.code;

        //! found modifier at key one, key two is the key
        if (inPackage.buttonKeyCode1 == codes.buttons.function.code)
        {
            modifierCode = codes.buttons.function.code;
            keyCode      = inPackage.buttonKeyCode2;
        }
            //! found modifier at key two, key one is the key
        else if (inPackage.buttonKeyCode2 == codes.buttons.function.code)
        {
            modifierCode = codes.buttons.function.code;
            keyCode      = inPackage.buttonKeyCode1;
        }
            //! no modifier, key one and key two are defined, fallback to key two which is the lastly one pressed
        else if (inPackage.buttonKeyCode2 != codes.buttons.undefined.code)
        {
            keyCode = inPackage.buttonKeyCode2;
        }
            //! fallback to whatever key one is
        else
        {
            keyCode = inPackage.buttonKeyCode1;
        }

        // update current axis and step to hal
        *(hal.memory.jogCount) += inPackage.stepCount;
        *(hal.memory.jogCountNeg)  = -*(hal.memory.jogCount);
        *(hal.memory.jogEnableOff) = (inPackage.rotaryButtonAxisKeyCode == codes.axis.off.code);
        *(hal.memory.jogEnableX)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.x.code);
        *(hal.memory.jogEnableY)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.y.code);
        *(hal.memory.jogEnableZ)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.z.code);
        *(hal.memory.jogEnableA)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.a.code);
        *(hal.memory.jogEnableB)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.b.code);
        *(hal.memory.jogEnableC)   = (inPackage.rotaryButtonAxisKeyCode == codes.axis.c.code);

        //! print human readable data
        if (hal.getIsSimulationMode())
        {
            if (inPackage.rotaryButtonFeedKeyCode != 0)
            {
                ios init(NULL);
                init.copyfmt(cout);
                cout << " delta " << setfill(' ') << setw(2) << (unsigned short)inPackage.rotaryButtonFeedKeyCode;
                cout.copyfmt(init);
            }
            cout << " => ";
            printData(inPackage);
        }

        //! update all buttons state to hal
        int      buttonsCount = sizeof(softwareButtons) / sizeof(WhbSoftwareButton);
        for (int idx          = 0; idx < buttonsCount; idx++)
        {
            if ((softwareButtons[idx].key.code == keyCode) &&
                (softwareButtons[idx].modifier.code == modifierCode))
            {
                *(hal.memory.button_pin[idx]) = true;
                if (hal.getIsSimulationMode())
                {
                    cout << " pressed ";
                    printPushButtonText(keyCode, modifierCode);
                }
            }
            else
            {
                *(hal.memory.button_pin[idx]) = false;
            }
        }

        if (hal.getIsSimulationMode())
        {
            cout << endl;
        }
    }

    // ----------------------------------------------------------------------

    void WhbContext::initWhb()
    {
        stepHandler.old_inc_step_status = -1;
        //gettimeofday(&sleepState.last_wakeup, nullptr);
        doRun = true;
        usb.setIsRunning(true);
    }

    //! writes \p value to \p buffer
    //! \return sizeof(float)
    int WhbContext::encodeFloat(float value, unsigned char* buffer)
    {
        unsigned int   int_v      = (int)rtapi_rint(rtapi_fabs(value) * 10000.0);
        unsigned short int_part   = int_v / 10000;
        unsigned short fract_part = int_v % 10000;
        if (value < 0) fract_part = fract_part | 0x8000;
        *(short*)buffer       = int_part;
        *((short*)buffer + 1) = fract_part;
        return sizeof(float);
    }

    // ----------------------------------------------------------------------

    //! writes \p value on position \p buffer
    //! \return the sizeof(int16_t)
    int WhbContext::encodeInt16(int16_t value, unsigned char* buffer)
    {
        *((int16_t*)buffer) = value;
        return sizeof(int16_t);
    }

// ----------------------------------------------------------------------

    void WhbContext::xhcDisplayEncode(unsigned char* data, int len)
    {
        unsigned char buffer[6 * 7];
        unsigned char* iter = buffer;

        assert(len == 6 * 8);

        memset(buffer, 0, sizeof(buffer));

        *(iter++) = 0xFE;
        *(iter++) = 0xFD;
        *(iter++) = 0x0C;

        if (currentButtonCodes.currentAxisCode == codes.axis.a.code)
        {
            iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.aWorkpieceCoordinate)) / 1000, iter);
        }
        else
        {
            iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.xWorkpieceCoordinate)) / 1000, iter);
        }
        iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.yWorkpieceCoordinate)) / 1000, iter);
        iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.zWorkpieceCoordinate)) / 1000, iter);
        if (currentButtonCodes.currentAxisCode == codes.axis.a.code)
        {
            iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.aMachineCoordinate)) / 1000, iter);
        }
        else
        {
            iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.xMachineCoordinate)) / 1000, iter);
        }
        iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.yMachineCoordinate)) / 1000, iter);
        iter += encodeFloat(rtapi_rint(1000 * *(hal.memory.zMachineCoordinate)) / 1000, iter);
        iter += encodeInt16((int16_t)rtapi_rint(100.0 * *(hal.memory.feedrateOverride)), iter);
        iter += encodeInt16((int16_t)rtapi_rint(100.0 * *(hal.memory.spindleOverride)), iter);
        iter += encodeInt16((int16_t)rtapi_rint(60.0 * *(hal.memory.feedrate)), iter);
        iter += encodeInt16((int16_t)rtapi_rint(60.0 * *(hal.memory.spindleRps)), iter);

        switch (*(hal.memory.stepsize))
        {
            case 0:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0;
                break;
            case 1:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1;
                break;
            case 5:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_5;
                break;
            case 10:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_10;
                break;
            case 20:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_20;
                break;
            case 30:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_30;
                break;
            case 40:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_40;
                break;
            case 50:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_50;
                break;
            case 100:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_100;
                break;
            case 500:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_500;
                break;
            case 1000:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1000;
                break;
                //!step size not supported on the display
            default:
                buffer[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0;
                break;
        }

        // Multiplex to 6 USB transactions

        iter = buffer;
        for (int packet = 0; packet < 6; packet++)
        {
            for (int i = 0; i < 8; i++)
            {
                if (i == 0) data[i + 8 * packet] = 6;
                else data[i + 8 * packet] = *iter++;
            }
        }
    }

// ----------------------------------------------------------------------

    void WhbContext::requestTermination(int signal)
    {
        if (signal >= 0)
        {
            cout << "termination requested upon signal number " << signal << "..." << endl;
        }
        else
        {
            cout << "termination requested ... " << endl;
        }
        usb.requestTermination();
        doRun = false;

    }

// ----------------------------------------------------------------------

    bool WhbContext::isRunning() const
    {
        return doRun;
    }

    WhbContext::WhbContext() :
        hal(),
        usb(*this, hal.memory),
        mName("XHC-WHB04B-6"),
        codes(),
        stepHandler(),
        softwareButtons{
            WhbSoftwareButton(codes.buttons.reset, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.reset, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.stop, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.stop, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.start, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.start, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.feed_plus, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.feed_plus, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.feed_minus, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.feed_minus, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.spindle_plus, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.spindle_plus, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.spindle_minus, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.spindle_minus, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.machine_home, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.machine_home, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.safe_z, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.safe_z, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.workpiece_home, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.workpiece_home, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.spindle_on_off, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.spindle_on_off, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.undefined, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.probe_z, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.probe_z, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.macro10, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.macro10, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.manual_pulse_generator, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.manual_pulse_generator, codes.buttons.function),
            WhbSoftwareButton(codes.buttons.step_continuous, codes.buttons.undefined),
            WhbSoftwareButton(codes.buttons.step_continuous, codes.buttons.function)
        },
        previousButtonCodes(codes.buttons,
                            codes.axis,
                            codes.feed,
                            stepHandler.stepSize.step,
                            stepHandler.stepSize.continuous),
        currentButtonCodes(codes.buttons,
                           codes.axis,
                           codes.feed,
                           stepHandler.stepSize.step,
                           stepHandler.stepSize.continuous),
        velocityComputation(),
        doRun(false),
        mIsSimulationMode(false)
    {}

    WhbContext::~WhbContext()
    {}

// ----------------------------------------------------------------------

    void WhbContext::xhcSetDisplay()
    {
        unsigned char data[6 * 8];
        int           packet;

        xhcDisplayEncode(data, sizeof(data));

        for (packet = 0; packet < 6; packet++)
        {
            int r = libusb_control_transfer(usb.deviceHandle,
                                            LIBUSB_DT_HID, //bmRequestType 0x21
                                            LIBUSB_REQUEST_SET_CONFIGURATION, //bRequest 0x09
                                            0x0306,         //wValue
                                            0x00,           //wIndex
                                            data + 8 * packet,  //*data
                                            8,              //wLength
                                            0);             //timeout
            if (r < 0)
            {
                usb.setDoReconnect(true);
            }
        }
    }

// ----------------------------------------------------------------------

    void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode)
    {
        ios init(NULL);
        init.copyfmt(cout);
        int indent = 10;
        cout << setfill(' ');

        // no key code
        if (keyCode == codes.buttons.undefined.code)
        {
            // modifier specified
            if (modifierCode == codes.buttons.function.code)
            {
                cout << setw(indent) << codes.buttons.function.text;
            }
                // no modifier specified
            else
            {
                cout << setw(indent) << codes.buttons.undefined.text;
            }
            return;
        }

        // find key code
        const WhbKeyCode* whbKeyCode = (WhbKeyCode*)&codes.buttons;
        while (whbKeyCode->code != 0)
        {
            if (whbKeyCode->code == keyCode)
            {
                break;
            }
            whbKeyCode++;
        }
        // print key text
        if (modifierCode == codes.buttons.function.code)
        {
            cout << setw(indent) << whbKeyCode->altText;
        }
        else
        {
            cout << setw(indent) << whbKeyCode->text;
        }
        cout.copyfmt(init);
    }

// ----------------------------------------------------------------------

    void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode)
    {
        ios init(NULL);
        init.copyfmt(cout);

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
        cout << setw(5) << whbKeyCode->text << "(" << setw(4) << whbKeyCode->altText << ")";
        cout.copyfmt(init);
    }

// ----------------------------------------------------------------------

    void WhbContext::printData(const WhbUsbInPackage& inPackage)
    {
        ios init(NULL);
        init.copyfmt(cout);

        cout << "| " << setfill('0') << hex
             << setw(2) << static_cast<unsigned short>(inPackage.notAvailable1)
             << " | " << setw(2) << static_cast<unsigned short>(inPackage.notAvailable2)
             << " | ";
        cout.copyfmt(init);
        printPushButtonText(inPackage.buttonKeyCode1, inPackage.buttonKeyCode2);
        cout << " | ";
        printPushButtonText(inPackage.buttonKeyCode2, inPackage.buttonKeyCode1);
        cout << " | ";
        printRotaryButtonText((WhbKeyCode*)&codes.feed, inPackage.rotaryButtonFeedKeyCode);
        cout << " | ";
        printRotaryButtonText((WhbKeyCode*)&codes.axis, inPackage.rotaryButtonAxisKeyCode);
        cout << " | "
             << setfill(' ') << setw(3) << static_cast<short>(inPackage.stepCount)
             << " | "
             << hex << setfill('0') << setw(2) << static_cast<unsigned short>(inPackage.crc);

        cout.copyfmt(init);
    }

// ----------------------------------------------------------------------

    void WhbContext::printHexdump(const WhbUsbInPackage& inPackage)
    {
        ios init(NULL);
        init.copyfmt(cout);

        cout << setfill('0') << hex << "0x"
             << setw(2) << static_cast<unsigned short>(inPackage.notAvailable1) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.notAvailable2) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.buttonKeyCode1) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.buttonKeyCode2) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.rotaryButtonFeedKeyCode) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.rotaryButtonAxisKeyCode) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.stepCount & 0xff) << " "
             << setw(2) << static_cast<unsigned short>(inPackage.crc);

        cout.copyfmt(init);
    }

// ----------------------------------------------------------------------

    void WhbContext::linuxcncSimulate()
    {
        static int last_jog_counts = 0; // todo: move to class field
        //*(hal->stepsizeUp) = ((xhc->button_step != 0) && (currentButtonCodes.currentButton1Code == xhc->button_step));

        if (*(hal.memory.jogCount) != last_jog_counts)
        {
            int   delta_int = *(hal.memory.jogCount) - last_jog_counts;
            float delta     = delta_int * *(hal.memory.jogScale);
            if (*(hal.memory.jogEnableX))
            {
                *(hal.memory.xMachineCoordinate) += delta;
                *(hal.memory.xWorkpieceCoordinate) += delta;
            }

            if (*(hal.memory.jogEnableY))
            {
                *(hal.memory.yMachineCoordinate) += delta;
                *(hal.memory.yWorkpieceCoordinate) += delta;
            }

            if (*(hal.memory.jogEnableZ))
            {
                *(hal.memory.zMachineCoordinate) += delta;
                *(hal.memory.zWorkpieceCoordinate) += delta;
            }

            if (*(hal.memory.jogEnableA))
            {
                *(hal.memory.aMachineCoordinate) += delta;
                *(hal.memory.aWorkpieceCoordinate) += delta;
            }

            if (*(hal.memory.jogEnableB))
            {
                *(hal.memory.bMachineCoordinate) += delta;
                *(hal.memory.bWorkpieceCoordinate) += delta;
            }

            if (*(hal.memory.jogEnableC))
            {
                *(hal.memory.cMachineCoordinate) += delta;
                *(hal.memory.cWorkpieceCoordinate) += delta;
            }

            /*if (*(hal->jog_enable_spindle)) {
                *(hal->spindle_override) += delta_int * 0.01;
                if (*(hal->spindle_override) > 1) *(hal->spindle_override) = 1;
                if (*(hal->spindle_override) < 0) *(hal->spindle_override) = 0;
                *(hal->spindle_rps) = 25000.0/60.0 * *(hal->spindle_override);
            }

            if (*(hal->jog_enable_feedrate)) {
                *(hal->feedrate_override) += delta_int * 0.01;
                if (*(hal->feedrate_override) > 1) *(hal->feedrate_override) = 1;
                if (*(hal->feedrate_override) < 0) *(hal->feedrate_override) = 0;
                *(hal->feedrate) = 3000.0/60.0 * *(hal->feedrate_override);
            }*/

            last_jog_counts = *(hal.memory.jogCount);
        }
    }

// ----------------------------------------------------------------------

    void WhbContext::computeVelocity()
    {
        timeval now, delta_tv;
        gettimeofday(&now, nullptr);

        if (velocityComputation.last_tv.tv_sec == 0)
        {
            velocityComputation.last_tv = now;
        }
        timersub(&now, &velocityComputation.last_tv, &delta_tv);
        float elapsed = delta_tv.tv_sec + 1e-6f * delta_tv.tv_usec;
        if (elapsed <= 0) return;

        float delta_pos = (*(hal.memory.jogCount) -
                           velocityComputation.last_jog_counts) *
                          *(hal.memory.jogScale);
        float velocity  = *(hal.memory.jogMaxVelocity) *
                          60.0f *
                          *(hal.memory.jogScale);
        float k         = 0.05f;

        if (delta_pos)
        {
            *(hal.memory.jogVelocity)  = (1 - k) * *(hal.memory.jogVelocity) + k * velocity;
            *(hal.memory.jogIncrement) = rtapi_fabs(delta_pos);
            *(hal.memory.jogPlusX)     = (delta_pos > 0) && *(hal.memory.jogEnableX);
            *(hal.memory.jogMinusX)    = (delta_pos < 0) && *(hal.memory.jogEnableX);
            *(hal.memory.jogPlusY)     = (delta_pos > 0) && *(hal.memory.jogEnableY);
            *(hal.memory.jogMinusY)    = (delta_pos < 0) && *(hal.memory.jogEnableY);
            *(hal.memory.jogPlusZ)     = (delta_pos > 0) && *(hal.memory.jogEnableZ);
            *(hal.memory.jogMinusZ)    = (delta_pos < 0) && *(hal.memory.jogEnableZ);
            *(hal.memory.jogPlusA)     = (delta_pos > 0) && *(hal.memory.jogEnableA);
            *(hal.memory.jogMinusA)    = (delta_pos < 0) && *(hal.memory.jogEnableA);
            *(hal.memory.jogPlusB)     = (delta_pos > 0) && *(hal.memory.jogEnableB);
            *(hal.memory.jogMinusB)    = (delta_pos < 0) && *(hal.memory.jogEnableB);
            *(hal.memory.jogPlusC)     = (delta_pos > 0) && *(hal.memory.jogEnableC);
            *(hal.memory.jogMinusC)    = (delta_pos < 0) && *(hal.memory.jogEnableC);
            velocityComputation.last_jog_counts = *(hal.memory.jogCount);
            velocityComputation.last_tv         = now;
        }
        else
        {
            *(hal.memory.jogVelocity) = (1 - k) * *(hal.memory.jogVelocity);
            if (elapsed > 0.25)
            {
                *(hal.memory.jogVelocity) = 0;
                *(hal.memory.jogPlusX)    = 0;
                *(hal.memory.jogMinusX)   = 0;
                *(hal.memory.jogPlusY)    = 0;
                *(hal.memory.jogMinusY)   = 0;
                *(hal.memory.jogPlusZ)    = 0;
                *(hal.memory.jogMinusZ)   = 0;
                *(hal.memory.jogPlusA)    = 0;
                *(hal.memory.jogMinusA)   = 0;
                *(hal.memory.jogPlusB)    = 0;
                *(hal.memory.jogMinusB)   = 0;
                *(hal.memory.jogPlusC)    = 0;
                *(hal.memory.jogMinusC)   = 0;
            }
        }
    }

// ----------------------------------------------------------------------

    void WhbContext::handleStep()
    {
/*
        int inc_step_status = *(hal.memory.stepsizeUp);
        //! Use a local variable to avoid STEP display as 0 on pendant during transitions
        int stepSize        = *(hal.memory.stepsize);

        if (inc_step_status && !stepHandler.old_inc_step_status)
        {
            stepsize_idx++;
            // restart idx when 0 terminator reached:
            if (stepsize_sequence[stepsize_idx] == 0) stepsize_idx = 0;
            stepSize = stepsize_sequence[stepsize_idx];
        }

        stepHandler.old_inc_step_status = inc_step_status;
*/
        // todo: refactor me
        *(hal.memory.stepsize) = currentButtonCodes.getStepSize() * 100;
        // todo: refactor me
        *(hal.memory.jogScale) = *(hal.memory.stepsize);// * 0.001f;
    }

// ----------------------------------------------------------------------

    void WhbContext::setupAsyncTransfer()
    {
        usb.setupAsyncTransfer();
    }

// ----------------------------------------------------------------------

    void WhbContext::cbResponseIn(struct libusb_transfer* transfer)
    {
        // pass transfer to usb data parser
        usb.cbResponseIn(transfer);
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

    void WhbUsb::setupAsyncTransfer()
    {
        libusb_transfer* transfer = libusb_alloc_transfer(0);
        assert(transfer != nullptr);
        libusb_fill_bulk_transfer(transfer,
                                  deviceHandle,
                                  (0x1 | LIBUSB_ENDPOINT_IN),
                                  inputPackage.asBuffer,
                                  sizeof(inputPackage.asBuffer),
                                  usbInputResponseCallback,
                                  nullptr,
                                  750); // timeout
        int r = libusb_submit_transfer(transfer);
        assert(0 == r);
    }
// ----------------------------------------------------------------------

    void WhbUsb::cbResponseIn(struct libusb_transfer* transfer)
    {
        int expectedPckageSize = static_cast<int>(sizeof(WhbUsbInPackage));
        ios init(NULL);
        init.copyfmt(cout);
        switch (transfer->status)
        {
            case (LIBUSB_TRANSFER_COMPLETED):
                // sleep mode was previously detected, drop current package
                if (sleepState.dropNextInPackage)
                {
                    if (WhbUsb::ConstantPackages.sleepPackage != inputPackage.asFields)
                    {
                        cout << "expected sleep package starting with "
                             << hex << setfill('0') << setw(2)
                             << static_cast<unsigned short>(WhbUsb::ConstantPackages.sleepPackage.notAvailable1)
                             << " but got "
                             << hex << setfill('0') << setw(2)
                             << static_cast<unsigned short>(inputPackage.asFields.notAvailable1)
                             << endl;
                        cout.copyfmt(init);
                    }

                    sleepState.dropNextInPackage = false;
                    goto ___TRUNCATE_PACKAGE;
                }


                if (transfer->actual_length == expectedPckageSize)
                {
                    //! detect pendant going to sleep:
                    //! when powering off pedant sends two packages
                    //! 1st: \ref WhbUsbEmptyPackage
                    //! 2nd: \ref WhbUsbSleepPackage
                    if (WhbUsb::ConstantPackages.emptyPackage == inputPackage.asFields)
                    {
                        sleepState.dropNextInPackage = true;
                        *(mHalMemory.sleeping) = 1;
                        if (mIsSimulationMode)
                        {
                            struct timeval now;
                            gettimeofday(&now, nullptr);
                            cout << "going to sleep: device was idle for "
                                 << (now.tv_sec - sleepState.last_wakeup.tv_sec)
                                 << " seconds" << endl;
                        }
                    }
                        // on regular package
                    else
                    {
                        if (*(mHalMemory.sleeping) == 1)
                        {
                            *(mHalMemory.sleeping) = 0;
                            if (mIsSimulationMode)
                            {
                                struct timeval now;
                                gettimeofday(&now, nullptr);
                                cout << "woke up: device was sleeping for "
                                     << (now.tv_sec - sleepState.last_wakeup.tv_sec)
                                     << " seconds" << endl;
                            }
                            gettimeofday(&sleepState.last_wakeup, nullptr);
                        }

                    }
                    // pass structured transfer to usb data handler
                    mDataHandler.handleInputData(inputPackage.asFields);
                }
                else
                {
                    cout << "received unexpected package size: expected="
                         << (transfer->actual_length)
                         << ", current=" << expectedPckageSize << endl;
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
                cout << "transfer error: " << transfer->status << endl;
                requestTermination();
                break;

            default:
                cout << "unknown transfer status: " << transfer->status << endl;
                requestTermination();
                break;
        }

        libusb_free_transfer(transfer);
    }


// ----------------------------------------------------------------------

    void WhbContext::setSimulationMode(bool enableSimulationMode)
    {
        mIsSimulationMode = enableSimulationMode;
        hal.setSimulationMode(mIsSimulationMode);
        usb.setSimulationMode(mIsSimulationMode);
    }

// ----------------------------------------------------------------------

    void WhbContext::halTeardown()
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
        memory(),
        mIsSimulationMode(true),
        mName("xhc-whb04b-6"),
        mHalCompId(-1)
    {}

// ----------------------------------------------------------------------


    WhbHal::~WhbHal()
    {
        if (mIsSimulationMode == false)
        {
            // do not free hal pins
            return;
        }

        freeSimulatedPin((void**)(&memory.xWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.yWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.zWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.aWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.bWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.cWorkpieceCoordinate));
        freeSimulatedPin((void**)(&memory.xMachineCoordinate));
        freeSimulatedPin((void**)(&memory.yMachineCoordinate));
        freeSimulatedPin((void**)(&memory.zMachineCoordinate));
        freeSimulatedPin((void**)(&memory.aMachineCoordinate));
        freeSimulatedPin((void**)(&memory.bMachineCoordinate));
        freeSimulatedPin((void**)(&memory.cMachineCoordinate));
        freeSimulatedPin((void**)(&memory.feedrateOverride));
        freeSimulatedPin((void**)(&memory.feedrate));
        freeSimulatedPin((void**)(&memory.spindleOverride));
        freeSimulatedPin((void**)(&memory.spindleRps));
        for (size_t idx = 0; idx < (sizeof(memory.button_pin) / sizeof(hal_bit_t * )); idx++)
        {
            freeSimulatedPin((void**)(&memory.button_pin[idx]));
        }
        freeSimulatedPin((void**)(&memory.jogEnableOff));
        freeSimulatedPin((void**)(&memory.jogEnableX));
        freeSimulatedPin((void**)(&memory.jogEnableY));
        freeSimulatedPin((void**)(&memory.jogEnableZ));
        freeSimulatedPin((void**)(&memory.jogEnableA));
        freeSimulatedPin((void**)(&memory.jogEnableB));
        freeSimulatedPin((void**)(&memory.jogEnableC));
        freeSimulatedPin((void**)(&memory.jogScale));
        freeSimulatedPin((void**)(&memory.jogCount));
        freeSimulatedPin((void**)(&memory.jogCountNeg));
        freeSimulatedPin((void**)(&memory.jogVelocity));
        freeSimulatedPin((void**)(&memory.jogMaxVelocity));
        freeSimulatedPin((void**)(&memory.jogIncrement));
        freeSimulatedPin((void**)(&memory.jogPlusX));
        freeSimulatedPin((void**)(&memory.jogPlusY));
        freeSimulatedPin((void**)(&memory.jogPlusZ));
        freeSimulatedPin((void**)(&memory.jogPlusA));
        freeSimulatedPin((void**)(&memory.jogPlusB));
        freeSimulatedPin((void**)(&memory.jogPlusC));
        freeSimulatedPin((void**)(&memory.jogMinusX));
        freeSimulatedPin((void**)(&memory.jogMinusY));
        freeSimulatedPin((void**)(&memory.jogMinusZ));
        freeSimulatedPin((void**)(&memory.jogMinusA));
        freeSimulatedPin((void**)(&memory.jogMinusB));
        freeSimulatedPin((void**)(&memory.jogMinusC));
        freeSimulatedPin((void**)(&memory.stepsizeUp));
        freeSimulatedPin((void**)(&memory.stepsize));
        freeSimulatedPin((void**)(&memory.sleeping));
        freeSimulatedPin((void**)(&memory.isPendantConnected));
        freeSimulatedPin((void**)(&memory.isPendantRequired));
    }

// ----------------------------------------------------------------------

    int WhbHal::newSimulatedHalPin(char* pin_name, void** ptr, int s)
    {
        *ptr = calloc(s, 1);
        assert(*ptr != nullptr);
        memset(*ptr, 0, s);
        cout << "registered " << pin_name << endl;
        return 0;
    }

// ----------------------------------------------------------------------

    int WhbHal::newFloatHalPin(hal_pin_dir_t dir, hal_float_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (mIsSimulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_float_t));
        }
        else
        {
            //cout << "registered " << pin_name << endl;
            return hal_pin_float_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

// ----------------------------------------------------------------------

    int WhbHal::newSigned32HalPin(hal_pin_dir_t dir, hal_s32_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (mIsSimulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_s32_t));
        }
        else
        {
            //cout << "registered " << pin_name << endl;
            return hal_pin_s32_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

// ----------------------------------------------------------------------

    int WhbHal::newBitHalPin(hal_pin_dir_t dir, hal_bit_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (mIsSimulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_bit_t));
        }
        else
        {
            //cout << "registered " << pin_name << endl;
            return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

// ----------------------------------------------------------------------

    bool WhbHal::getIsSimulationMode() const
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

    void WhbHal::halInit(WhbSoftwareButton* softwareButtons,
                         size_t buttonsCount,
                         const WhbKeyCodes& codes)
    {
        if (!mIsSimulationMode)
        {
            mHalCompId = hal_init(mName);
            if (mHalCompId < 1)
            {
                cerr << mName << ": ERROR: hal_init failed " << endl;
                exit(EXIT_FAILURE);
            }
        }

        // register all known xhc-whb04b-6 buttons
        for (size_t idx = 0; idx < buttonsCount; idx++)
        {
            const char* buttonName = nullptr;
            if (&softwareButtons[idx].modifier == &codes.buttons.undefined)
            {
                buttonName = softwareButtons[idx].key.text;
            }
            else
            {
                buttonName = softwareButtons[idx].key.altText;
            }
            int r = newBitHalPin(HAL_OUT, &(memory.button_pin[idx]), mHalCompId,
                                 "%s.%s", mName, buttonName);
            assert(0 == r);
        }

        int r = 0;
        r |= newFloatHalPin(HAL_IN, &(memory.xMachineCoordinate), mHalCompId, "%s.x.pos-absolute", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.yMachineCoordinate), mHalCompId, "%s.y.pos-absolute", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.zMachineCoordinate), mHalCompId, "%s.z.pos-absolute", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.aMachineCoordinate), mHalCompId, "%s.a.pos-absolute", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.bMachineCoordinate), mHalCompId, "%s.b.pos-absolute", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.cMachineCoordinate), mHalCompId, "%s.c.pos-absolute", mName);

        r |= newFloatHalPin(HAL_IN, &(memory.yWorkpieceCoordinate), mHalCompId, "%s.y.pos-relative", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.xWorkpieceCoordinate), mHalCompId, "%s.x.pos-relative", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.zWorkpieceCoordinate), mHalCompId, "%s.z.pos-relative", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.aWorkpieceCoordinate), mHalCompId, "%s.a.pos-relative", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.bWorkpieceCoordinate), mHalCompId, "%s.b.pos-relative", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.cWorkpieceCoordinate), mHalCompId, "%s.c.pos-relative", mName);

        r |= newFloatHalPin(HAL_IN, &(memory.feedrate), mHalCompId, "%s.feed-value", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.feedrateOverride), mHalCompId, "%s.feed-override", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.spindleRps), mHalCompId, "%s.spindle-rps", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.spindleOverride), mHalCompId, "%s.spindle-override", mName);

        r |= newBitHalPin(HAL_OUT, &(memory.sleeping), mHalCompId, "%s.sleeping", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.isPendantConnected), mHalCompId, "%s.connected", mName);
        r |= newBitHalPin(HAL_IN, &(memory.stepsizeUp), mHalCompId, "%s.stepsize-up", mName);
        r |= newSigned32HalPin(HAL_OUT, &(memory.stepsize), mHalCompId, "%s.stepsize", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.isPendantRequired), mHalCompId, "%s.require_pendant", mName);

        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableOff), mHalCompId, "%s.jog.enable-off", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableX), mHalCompId, "%s.jog.enable-x", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableY), mHalCompId, "%s.jog.enable-y", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableZ), mHalCompId, "%s.jog.enable-z", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableA), mHalCompId, "%s.jog.enable-a", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableB), mHalCompId, "%s.jog.enable-b", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogEnableC), mHalCompId, "%s.jog.enable-c", mName);

        //r |= _hal_pin_bit_newf(HAL_OUT, &(hal.memory.jog_enable_feedrate), hal_comp_id, "%s.jog.enable-feed-override", modname);
        //r |= _hal_pin_bit_newf(HAL_OUT, &(hal.memory.jog_enable_spindle), hal_comp_id, "%s.jog.enable-spindle-override", modname);

        r |= newFloatHalPin(HAL_OUT, &(memory.jogScale), mHalCompId, "%s.jog.scale", mName);
        r |= newSigned32HalPin(HAL_OUT, &(memory.jogCount), mHalCompId, "%s.jog.counts", mName);
        r |= newSigned32HalPin(HAL_OUT, &(memory.jogCountNeg), mHalCompId, "%s.jog.counts-neg", mName);

        r |= newFloatHalPin(HAL_OUT, &(memory.jogVelocity), mHalCompId, "%s.jog.velocity", mName);
        r |= newFloatHalPin(HAL_IN, &(memory.jogMaxVelocity), mHalCompId, "%s.jog.max-velocity", mName);
        r |= newFloatHalPin(HAL_OUT, &(memory.jogIncrement), mHalCompId, "%s.jog.increment", mName);

        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusX), mHalCompId, "%s.jog.plus-x", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusY), mHalCompId, "%s.jog.plus-y", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusZ), mHalCompId, "%s.jog.plus-z", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusA), mHalCompId, "%s.jog.plus-a", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusB), mHalCompId, "%s.jog.plus-b", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogPlusC), mHalCompId, "%s.jog.plus-c", mName);

        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusX), mHalCompId, "%s.jog.minus-x", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusY), mHalCompId, "%s.jog.minus-y", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusZ), mHalCompId, "%s.jog.minus-z", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusA), mHalCompId, "%s.jog.minus-a", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusB), mHalCompId, "%s.jog.minus-b", mName);
        r |= newBitHalPin(HAL_OUT, &(memory.jogMinusC), mHalCompId, "%s.jog.minus-c", mName);

        assert (r == 0);
    }

}

// ----------------------------------------------------------------------

XhcWhb04b6::WhbContext

    Whb;

// ----------------------------------------------------------------------

static void Usage(char* name)
{
    cerr << name << " version " << PACKAGE_VERSION << " 2017 by Raoul Rubien (github.com/rubienr)" << endl;
    cerr << "Usage: " << name << " [-h] [-H] " << endl;
    cerr << " -h: usage (this)" << endl;
    cerr << " -H: run in real-time HAL mode (run in simulation mode by default)" << endl;
    cerr << " -x: wait for pendant detection before creating HAL pins" << endl;
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
    Whb.cbResponseIn(transfer);
}

// ----------------------------------------------------------------------

int main(int argc, char** argv)
{
    int     r;
    ssize_t cnt;
#define MAX_WAIT_SECS 10
    int wait_secs       = 0;

    int  opt;
    bool hal_ready_done = false;

    Whb.initWhb();
    Whb.setSimulationMode(true);
    while ((opt = getopt(argc, argv, "HhI:xs:")) != -1)
    {
        switch (opt)
        {
            case 'H':
                Whb.setSimulationMode(false);
                break;
            case 'x':
                Whb.usb.setWaitForPendantBeforeHAL(true);
                break;
            default:
                Usage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    Whb.halInit();

    registerSignalHandler();


    if (!Whb.usb.getWaitForPendantBeforeHAL() && !Whb.hal.getIsSimulationMode())
    {
        hal_ready(Whb.hal.getHalComponentId());
        hal_ready_done = true;
    }

    while (Whb.isRunning())
    {
        //on reconnect wait for device to be gone
        if (Whb.usb.getDoReconnect() == true)
        {
            sleep(5);
            Whb.usb.setDoReconnect(false);
        }

        // todo: refactor me
        r = libusb_init(Whb.usb.getContextReference());

        if (r != 0)
        {
            perror("libusb_init");
            return 1;
        }
        libusb_set_debug(Whb.usb.getContext(), 3);

        cout << Whb.getName() << "waiting for device ..." << endl;
        *(Whb.hal.memory.isPendantConnected) = 0;
        wait_secs = 0;
        *(Whb.hal.memory.isPendantRequired) = Whb.usb.getWaitForPendantBeforeHAL();
        //*(Whb.hal.memory.stepsize)          = stepsize_sequence[0];

        do
        {
            libusb_device** devs;
            cnt = libusb_get_device_list(Whb.usb.getContext(), &devs);
            if (cnt < 0)
            {
                perror("libusb_get_device_list");
                return 1;
            }

            // todo: refactor me
            Whb.usb.setDeviceHandle(libusb_open_device_with_vid_pid(Whb.usb.getContext(),
                                                                    Whb.usb.getUsbVendorId(),
                                                                    Whb.usb.getUsbProductId()));
            libusb_free_device_list(devs, 1);
            if (Whb.usb.isDeviceOpen() == false)
            {
                if (Whb.usb.getWaitForPendantBeforeHAL())
                {
                    wait_secs++;
                    if (wait_secs >= MAX_WAIT_SECS / 2)
                    {
                        cout << Whb.getName() << ": waiting " << wait_secs << "s for device " << Whb.getName()
                             << "..." << endl;
                    }
                    if (wait_secs > MAX_WAIT_SECS)
                    {
                        cout << Whb.getName() << ": MAX_WAIT_SECS=" << MAX_WAIT_SECS << " exceeded, exiting"
                             << endl;
                        exit(EXIT_FAILURE);
                    }
                }
                sleep(1);
            }
        } while ((Whb.usb.isDeviceOpen() == false) && Whb.isRunning());

        cout << Whb.getName() << ": found " << Whb.getName() << " device" << endl;

        if (Whb.usb.isDeviceOpen() != false)
        {
            // todo: refactor me
            if (libusb_kernel_driver_active(Whb.usb.getDeviceHandle(), 0) == 1)
            {
                int r = libusb_detach_kernel_driver(Whb.usb.getDeviceHandle(), 0);
                assert(0 == r);
            }

            r = libusb_claim_interface(Whb.usb.getDeviceHandle(), 0);
            if (r != 0)
            {
                perror("libusb_claim_interface");
                return 1;
            }
        }

        *(Whb.hal.memory.isPendantConnected) = 1;

        if (!hal_ready_done && !Whb.hal.getIsSimulationMode())
        {
            hal_ready(Whb.hal.getHalComponentId());
            hal_ready_done = true;
        }

        if (Whb.usb.isDeviceOpen())
        {
            Whb.setupAsyncTransfer();
            Whb.xhcSetDisplay();
        }

        if (Whb.usb.isDeviceOpen())
        {
            while (Whb.isRunning() && !Whb.usb.getDoReconnect())
            {
                struct timeval tv;
                tv.tv_sec  = 0;
                tv.tv_usec = 30000;
                libusb_handle_events_timeout_completed(Whb.usb.getContext(), &tv, nullptr);
                Whb.computeVelocity();
                if (Whb.hal.getIsSimulationMode())
                {
                    Whb.linuxcncSimulate();
                }
                Whb.handleStep();
                Whb.xhcSetDisplay();
            }

            *(Whb.hal.memory.isPendantConnected) = 0;
            cout << Whb.getName() << ": connection lost, cleaning up" << endl;
            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 750000;
            int r = libusb_handle_events_timeout_completed(Whb.usb.getContext(), &tv, nullptr);
            assert(0 == r);
            // todo: refactor me
            r = libusb_release_interface(Whb.usb.getDeviceHandle(), 0);
            assert(0 == r);
            libusb_close(Whb.usb.getDeviceHandle());
            Whb.usb.setDeviceHandle(nullptr);
        }
        else
        {
            while (!Whb.isRunning()) usleep(70000);
        }
        // todo: refactor me
        libusb_exit(Whb.usb.getContext());
        // todo: refactor me
        Whb.usb.setContext(nullptr);
    }
    // todo: refactor me
    Whb.halExit();

    //! hotfix for https://github.com/machinekit/machinekit/issues/1266
    if (Whb.hal.getIsSimulationMode())
    {
        google::protobuf::ShutdownProtobufLibrary();
    }
    return 0;
}
