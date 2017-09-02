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


//! function for libusb's incoming data callback function
void usbInputResponseCallback(struct libusb_transfer* transfer);

//! registers signal handler
void registerSignalHandler();

//! called on program termination requested
static void quit(int signal);


namespace XhcWhb04b6 {

    class WhbHalMemory;

    class WhbHalEntity;

    class WhbCleanupRef;

    class WhbHal;

    class WhbSleepDetect;

    class WhbInPackageInfo;

    class WhbPackageInfo;

    class WhbKeyCode;

    class WhbSoftwareButton;

    class WhbAxisRotaryButtonCodes;

    class WhbFeedRotaryButtonCodes;

    class WhbButtonsCode;

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

//! HAL component parameters
    class WhbHalEntity
    {
        friend XhcWhb04b6::WhbContext;

        int halCompId;

    public:

        int getHalComponentId() const
        {
            return halCompId;
        }

        void setHalCompnentId(int componentId)
        {
            this->halCompId = componentId;
        }

        WhbHalEntity() :
            halCompId(0)
        {}
    };

// ----------------------------------------------------------------------

//! allocated memory reference lookup
    struct WhbCleanupRef
    {
        void* refs[256];
        uint16_t nextIndex;

        WhbCleanupRef() :
            refs{0},
            nextIndex(0)
        {}
    };

// ----------------------------------------------------------------------

//! HAL and related parameters
    class WhbHal
    {
        friend XhcWhb04b6::WhbContext;

        bool          simulationMode;
        //! if \p simulationMode == true, \p cleanup keeps malloced references for freeing
        WhbCleanupRef cleanup;
    public:
        WhbHalEntity entity;
        WhbHalMemory memory;

    public:
        bool getIsSimulationMode() const
        {
            return simulationMode;
        }

        void setIsSimulationMode(bool isSimulationMode)
        {
            this->simulationMode = isSimulationMode;
        }

        WhbHal() :
            simulationMode(true),
            cleanup(),
            entity(),
            memory()
        {}
    };

// ----------------------------------------------------------------------

//! pendant sleep/idle state parameters
    class WhbSleepDetect
    {
        friend XhcWhb04b6::WhbContext;

        bool           dropNextInPackage;
        struct timeval last_wakeup;

    public:
        WhbSleepDetect() :
            dropNextInPackage(false),
            last_wakeup()
        {}
    };

// ----------------------------------------------------------------------

//! from pendant coming communication package description
    struct WhbInPackageInfo
    {
        //! names the byte type at the specified position in data stream
        typedef enum
        {
            NA1 = 0, NA2 = 1, Key1 = 2, Key2 = 3, Feed = 4, Axis = 5, Step = 6, CRC = 7
        }              ByteType;
        //! inverse type to position map (position to type)
        const ByteType positionToType[8];
        const uint8_t  expectedSize;

        WhbInPackageInfo() :
            positionToType{
                ByteType::NA1,
                ByteType::NA2,
                ByteType::Key1,
                ByteType::Key2,
                ByteType::Feed,
                ByteType::Axis,
                ByteType::Step,
                ByteType::CRC
            },
            expectedSize(sizeof(positionToType) / sizeof(ByteType))
        {}
    };

// ----------------------------------------------------------------------

//! pendant i/o package related parameters
    struct WhbPackageInfo
    {
        const WhbInPackageInfo in;

        WhbPackageInfo() :
            in()
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
            function(0x0c, "Fn", ""),
            probe_z(0x0d, "Probe-Z", "Macro-9"),
            macro10(0x10, "Macro-10", "Macro-13"),
            manual_pulse_generator(0x0e, "MPG", "Macro-14"),
            step_continuous(0x0f, "STEP", "Continuous"),
            undefined(0x00, "", "")
        {}

    };

// ----------------------------------------------------------------------

//! general pendant description parameters
    class WhbEntity
    {
    public:
        const char* name;
        const char* configSectionName;
        const uint16_t usbVendorId;
        const uint16_t usbProductId;

        WhbEntity() :
            name("xhc-whb04b-6"),
            configSectionName("XHC-WHB04B-6"),
            usbVendorId(0x10ce),
            usbProductId(0xeb93)
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

    //! USB related parameters
    class WhbUsb
    {
        friend XhcWhb04b6::WhbContext;

        libusb_context      * context;
        libusb_device_handle* deviceHandle;
        bool          do_reconnect;
        bool          wait_for_pendant_before_HAL;
        unsigned char inputBuffer[32];


    public:

        const bool isDeviceOpen() const
        {
            return deviceHandle != nullptr;
        }

        libusb_context** getContextReference()
        {
            return &context;
        }

        libusb_context* getContext()
        {
            return context;
        }

        void setContext(libusb_context* context)
        {
            this->context = context;
        }


        libusb_device_handle* getDeviceHandle()
        {
            return deviceHandle;
        }

        void setDeviceHandle(libusb_device_handle* deviceHandle)
        {
            this->deviceHandle = deviceHandle;
        }

        void setWaitForPendantBeforeHAL(bool waitForPendantBeforeHAL)
        {
            wait_for_pendant_before_HAL = waitForPendantBeforeHAL;
        }

        bool getWaitForPendantBeforeHAL() const
        {
            return wait_for_pendant_before_HAL;
        }

        bool getDoReconnect() const
        {
            return do_reconnect;
        }

        void setDoReconnect(bool doReconnect)
        {
            this->do_reconnect = doReconnect;
        }

        WhbUsb() :
            context(nullptr),
            deviceHandle(nullptr),
            do_reconnect(false),
            wait_for_pendant_before_HAL(false),
            inputBuffer{0}
        {}
    };

    // ----------------------------------------------------------------------

    //! program context
    class WhbContext
    {
    public:
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


        WhbContext() :
            entity(),
            hal(),
            usb(),
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
                WhbSoftwareButton(codes.buttons.function, codes.buttons.undefined),
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
            packageInfo(),
            velocityComputation(),
            sleepState(),
            doRun(false)
        {}

        const WhbEntity entity;
        WhbHal          hal;
        WhbUsb          usb;

    private:

        const WhbKeyCodes      codes;
        WhbStepHandler         stepHandler;
        WhbSoftwareButton      softwareButtons[31];
        WhbButtonsState        previousButtonCodes;
        WhbButtonsState        currentButtonCodes;
        const WhbPackageInfo   packageInfo;
        WhbVelocityComputation velocityComputation;
        WhbSleepDetect         sleepState;
        bool                   doRun;

        //! todo: doxy
        void printPushButtonText(uint8_t keyCode, uint8_t modifierCode);

        //! todo: doxy
        void printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode);

        //! todo: doxy
        void printData(const unsigned char* data, int length);

        //! todo: doxy
        void printHexdump(unsigned char* data, int len);

        //! todo: doxy
        int encodeFloat(float value, unsigned char* buffer);

        //! todo: doxy
        int encodeInt16(int16_t value, unsigned char* buffer);

        // todo: refactor me
        void xhcDisplayEncode(unsigned char* data, int len);

        //! todo: doxy
        int newSimulatedHalPin(char* pin_name, void** ptr, int s);

        //! todo: doxy
        int newFloatHalPin(hal_pin_dir_t dir, hal_float_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! todo: doxy
        int newSigned32HalPin(hal_pin_dir_t dir, hal_s32_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! todo: doxy
        int newBitHalPin(hal_pin_dir_t dir, hal_bit_t** data_ptr_addr, int comp_id, const char* fmt, ...);

        //! todo: doxy
        void halTeardown();

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

    extern "C" const char*
    iniFind(FILE* fp, const char* tag, const char* configSecitonName)
    {
        IniFile f(false, fp);

        return (f.Find(tag, configSecitonName));
    }

    // ----------------------------------------------------------------------

    void WhbContext::halExit()
    {
        halTeardown();
        hal_exit(hal.entity.halCompId);
    }
    // ----------------------------------------------------------------------

    void WhbContext::initWhb()
    {
        stepHandler.old_inc_step_status = -1;
        gettimeofday(&sleepState.last_wakeup, nullptr);
        doRun = true;
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
        doRun = false;
    }

    // ----------------------------------------------------------------------

    bool WhbContext::isRunning() const
    {
        return doRun;
    }

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
        uint8_t indent = 10;
        const WhbKeyCode* keyCodeBase = (WhbKeyCode*)&codes.buttons;
        // no key
        if (keyCode == codes.buttons.undefined.code)
        {
            if (modifierCode == codes.buttons.function.code)
            {
                printf("%*s", indent, codes.buttons.undefined.altText);
            }
            else
            {
                printf("%*s", indent, codes.buttons.undefined.text);
            }
            return;
        }

        // key is modifier key itself
        if (keyCode == codes.buttons.function.code)
        {
            printf("%*s", indent, codes.buttons.function.text);
            return;
        }

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

        if (modifierCode == codes.buttons.function.code)
        {
            printf("%*s", indent, whbKeyCode->altText);
        }
        else
        {
            printf("%*s", indent, whbKeyCode->text);
        }
    }

    // ----------------------------------------------------------------------

    void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode)
    {
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
        printf("%*s (%*s)", 5, whbKeyCode->text, 4, whbKeyCode->altText);
    }

    // ----------------------------------------------------------------------

    void WhbContext::printData(const unsigned char* data, int length)
    {
        if (length != packageInfo.in.expectedSize) return;

        for (uint8_t idx = 0; idx < packageInfo.in.expectedSize; idx++)
        {
            WhbInPackageInfo::ByteType dataType = packageInfo.in.positionToType[idx];
            switch (dataType)
            {
                case WhbInPackageInfo::ByteType::NA1:
                    printf("| %02X | ", data[idx]);
                    break;
                case WhbInPackageInfo::ByteType::NA2:
                    printf("%02X | ", data[idx]);
                    break;
                case WhbInPackageInfo::ByteType::Key1:
                    printPushButtonText((uint8_t)data[idx], (uint8_t)data[idx + 1]);
                    printf(" | ");
                    break;
                case WhbInPackageInfo::ByteType::Key2:
                    printPushButtonText((uint8_t)data[idx], (uint8_t)data[idx - 1]);
                    printf(" | ");
                    break;
                case WhbInPackageInfo::ByteType::Feed:
                    printRotaryButtonText((WhbKeyCode*)&codes.feed, (uint8_t)data[idx]);
                    printf(" | ");
                    break;
                case WhbInPackageInfo::ByteType::Axis:
                    printRotaryButtonText((WhbKeyCode*)&codes.axis, (uint8_t)data[idx]);
                    printf(" | ");
                    break;
                case WhbInPackageInfo::ByteType::Step:
                    printf("%*d | ", 3, (int8_t)data[idx]);
                    break;
                case WhbInPackageInfo::ByteType::CRC:
                    printf("%02X |", data[idx]);
                    break;
                default:
                    break;
            }
        }
    }

    // ----------------------------------------------------------------------

    void WhbContext::printHexdump(unsigned char* data, int len)
    {
        for (int i = 0; i < len; i++) printf("%02X ", data[i]);
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
        libusb_transfer* transfer = libusb_alloc_transfer(0);
        assert(transfer != nullptr);
        libusb_fill_bulk_transfer(transfer,
                                  usb.deviceHandle,
                                  (0x1 | LIBUSB_ENDPOINT_IN),
                                  usb.inputBuffer,
                                  sizeof(usb.inputBuffer),
                                  usbInputResponseCallback, nullptr, // no user data
                                  750); // timeout
        int r = libusb_submit_transfer(transfer);
        assert(0 == r);
    }

    // ----------------------------------------------------------------------

    void WhbContext::cbResponseIn(struct libusb_transfer* transfer)
    {
        switch (transfer->status)
        {
            case (LIBUSB_TRANSFER_COMPLETED):
                // detetec sleep mode, truncate subsequent package once
                if (sleepState.dropNextInPackage)
                {
                    sleepState.dropNextInPackage = false;
                    goto ___TRUNCATE_PACKAGE;
                }

                if (transfer->actual_length > 0)
                {
                    if (hal.getIsSimulationMode()) printHexdump(usb.inputBuffer, transfer->actual_length);

                    // clarify modifier and key
                    uint8_t buttonCode1 = usb.inputBuffer[WhbInPackageInfo::ByteType::Key1];
                    uint8_t buttonCode2 = usb.inputBuffer[WhbInPackageInfo::ByteType::Key2];

                    uint8_t modifierCode = codes.buttons.undefined.code;
                    uint8_t keyCode      = codes.buttons.undefined.code;

                    //! found modifier on key1, key2 is the key
                    if (buttonCode1 == codes.buttons.function.code)
                    {
                        modifierCode = codes.buttons.function.code;
                        keyCode      = buttonCode2;
                    }
                        //! found modifier on key2, key1 is the key
                    else if (buttonCode2 == codes.buttons.function.code)
                    {
                        modifierCode = codes.buttons.function.code;
                        keyCode      = buttonCode1;
                    }
                        //! no modifier, key1 and key2 are defined, fallback to key2
                    else if (buttonCode2 != codes.buttons.undefined.code)
                    {
                        keyCode = buttonCode2;
                    }
                        //! fallback to whatever key one is
                    else
                    {
                        keyCode = buttonCode1;
                    }

                    // update current axis and step to hal
                    currentButtonCodes.currentAxisCode = usb.inputBuffer[WhbInPackageInfo::ByteType::Axis];
                    *(hal.memory.jogCount) += ((signed char)usb.inputBuffer[WhbInPackageInfo::ByteType::Step]);
                    *(hal.memory.jogCountNeg)          = -*(hal.memory.jogCount);
                    *(hal.memory.jogEnableOff)         = (currentButtonCodes.currentAxisCode == codes.axis.off.code);
                    *(hal.memory.jogEnableX)           = (currentButtonCodes.currentAxisCode == codes.axis.x.code);
                    *(hal.memory.jogEnableY)           = (currentButtonCodes.currentAxisCode == codes.axis.y.code);
                    *(hal.memory.jogEnableZ)           = (currentButtonCodes.currentAxisCode == codes.axis.z.code);
                    *(hal.memory.jogEnableA)           = (currentButtonCodes.currentAxisCode == codes.axis.a.code);
                    *(hal.memory.jogEnableB)           = (currentButtonCodes.currentAxisCode == codes.axis.b.code);
                    *(hal.memory.jogEnableC)           = (currentButtonCodes.currentAxisCode == codes.axis.c.code);

                    //! print human readable data
                    if (hal.getIsSimulationMode())
                    {
                        if ((char)usb.inputBuffer[4] != 0) printf(" delta %+3d", (char)usb.inputBuffer[4]);
                        printf(" => ");
                        printData(usb.inputBuffer, transfer->actual_length);
                        printf("\n");
                    }

                    //! update button state to hal
                    int      buttonsCount = sizeof(softwareButtons) / sizeof(WhbSoftwareButton);
                    for (int idx          = 0; idx < buttonsCount; idx++)
                    {
                        if ((softwareButtons[idx].key.code == keyCode) &&
                            (softwareButtons[idx].modifier.code == modifierCode))
                        {
                            *(hal.memory.button_pin[idx]) = true;
                            if (hal.getIsSimulationMode())
                            {
                                printPushButtonText(keyCode, modifierCode);
                                printf(" pressed\n");
                            }
                        }
                        else
                        {
                            *(hal.memory.button_pin[idx]) = false;
                        }
                    }

                    //! detect pendant going to sleep:
                    //! when powering off pedant sends two packages
                    //! 1st: 0x4 0x? 0x0 0x0 0x0 0x0 0x?
                    //! 2nd; 0x4 0x? 0x? 0x? 0x? 0x? 0x?
                    if (usb.inputBuffer[WhbInPackageInfo::ByteType::NA1] == 0x04 &&
                        usb.inputBuffer[WhbInPackageInfo::ByteType::Key1] == 0 &&
                        usb.inputBuffer[WhbInPackageInfo::ByteType::Key2] == 0 &&
                        usb.inputBuffer[WhbInPackageInfo::ByteType::Feed] == 0 &&
                        usb.inputBuffer[WhbInPackageInfo::ByteType::Axis] == 0 &&
                        usb.inputBuffer[WhbInPackageInfo::ByteType::Step] == 0)
                    {
                        sleepState.dropNextInPackage = true;
                        *(hal.memory.sleeping) = 1;
                        if (hal.getIsSimulationMode())
                        {
                            struct timeval now;
                            gettimeofday(&now, nullptr);
                            fprintf(stderr, "Sleep, %s was idle for %ld seconds\n",
                                    entity.name, now.tv_sec - sleepState.last_wakeup.tv_sec);
                        }
                    }
                    else
                    {
                        gettimeofday(&sleepState.last_wakeup, nullptr);
                        if (*(hal.memory.sleeping))
                        {
                            if (hal.getIsSimulationMode())
                            {
                                fprintf(stderr, "Wake\n");
                            }
                        }
                        *(hal.memory.sleeping) = 0;
                    }
                }

                if (isRunning())
                {
                    setupAsyncTransfer();
                }

                break;

            ___TRUNCATE_PACKAGE:
            case (LIBUSB_TRANSFER_TIMED_OUT):
                if (isRunning())
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
                printf("transfer error: %d", transfer->status);
                requestTermination();
                break;
            default:
                printf("unknown transfer status %d\n", transfer->status);
                requestTermination();
                break;
        }

        libusb_free_transfer(transfer);
    }

    // ----------------------------------------------------------------------

    int WhbContext::newSimulatedHalPin(char* pin_name, void** ptr, int s)
    {
        *ptr = calloc(s, 1);
        assert(*ptr != nullptr);
        memset(*ptr, 0, s);
        hal.cleanup.refs[hal.cleanup.nextIndex++] = *ptr;
        printf("registered %s\n", pin_name);
        return 0;
    }

    // ----------------------------------------------------------------------

    int WhbContext::newFloatHalPin(hal_pin_dir_t dir, hal_float_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (hal.simulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_float_t));
        }
        else
        {
            //printf("registered %s\n", pin_name);
            return hal_pin_float_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

    // ----------------------------------------------------------------------

    int WhbContext::newSigned32HalPin(hal_pin_dir_t dir, hal_s32_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (hal.simulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_s32_t));
        }
        else
        {
            //printf("registered %s\n", pin_name);
            return hal_pin_s32_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

    // ----------------------------------------------------------------------

    int WhbContext::newBitHalPin(hal_pin_dir_t dir, hal_bit_t** data_ptr_addr, int comp_id, const char* fmt, ...)
    {
        char    pin_name[256];
        va_list args;
        va_start(args, fmt);
        vsprintf(pin_name, fmt, args);
        va_end(args);

        if (hal.simulationMode)
        {
            return newSimulatedHalPin(pin_name, (void**)data_ptr_addr, sizeof(hal_bit_t));
        }
        else
        {
            //printf("registered %s\n", pin_name);
            return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
        }
    }

    // ----------------------------------------------------------------------

    void WhbContext::halTeardown()
    {
        if (hal.simulationMode)
        {
            for (uint16_t idx = 0; idx < hal.cleanup.nextIndex; idx++)
            {
                free(hal.cleanup.refs[idx]);
                hal.cleanup.refs[idx] = nullptr;

            }
            hal.cleanup.nextIndex = 0;
        }
    }

    // ----------------------------------------------------------------------

    void WhbContext::halInit()
    {
        const char* modname = entity.name;

        if (!hal.simulationMode)
        {
            hal.entity.halCompId = hal_init(modname);
            if (hal.entity.halCompId < 1)
            {
                fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
                exit(EXIT_FAILURE);
            }
        }

        // register all known whb04b-6 buttons
        int      buttonsCount = sizeof(softwareButtons) / sizeof(WhbSoftwareButton);
        for (int idx          = 0; idx < buttonsCount; idx++)
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
            int r = newBitHalPin(HAL_OUT, &(hal.memory.button_pin[idx]), hal.entity.halCompId,
                                 "%s.%s", modname, buttonName);
            assert(0 == r);
        }

        int halCompId = hal.entity.halCompId;
        int r         = 0;
        r |= newFloatHalPin(HAL_IN, &(hal.memory.xMachineCoordinate), halCompId, "%s.x.pos-absolute", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.yMachineCoordinate), halCompId, "%s.y.pos-absolute", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.zMachineCoordinate), halCompId, "%s.z.pos-absolute", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.aMachineCoordinate), halCompId, "%s.a.pos-absolute", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.bMachineCoordinate), halCompId, "%s.b.pos-absolute", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.cMachineCoordinate), halCompId, "%s.c.pos-absolute", modname);

        r |= newFloatHalPin(HAL_IN, &(hal.memory.yWorkpieceCoordinate), halCompId, "%s.y.pos-relative", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.xWorkpieceCoordinate), halCompId, "%s.x.pos-relative", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.zWorkpieceCoordinate), halCompId, "%s.z.pos-relative", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.aWorkpieceCoordinate), halCompId, "%s.a.pos-relative", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.bWorkpieceCoordinate), halCompId, "%s.b.pos-relative", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.cWorkpieceCoordinate), halCompId, "%s.c.pos-relative", modname);

        r |= newFloatHalPin(HAL_IN, &(hal.memory.feedrate), halCompId, "%s.feed-value", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.feedrateOverride), halCompId, "%s.feed-override", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.spindleRps), halCompId, "%s.spindle-rps", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.spindleOverride), halCompId, "%s.spindle-override", modname);

        r |= newBitHalPin(HAL_OUT, &(hal.memory.sleeping), halCompId, "%s.sleeping", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.isPendantConnected), halCompId, "%s.connected", modname);
        r |= newBitHalPin(HAL_IN, &(hal.memory.stepsizeUp), halCompId, "%s.stepsize-up", modname);
        r |= newSigned32HalPin(HAL_OUT, &(hal.memory.stepsize), halCompId, "%s.stepsize", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.isPendantRequired), halCompId, "%s.require_pendant", modname);

        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableOff), halCompId, "%s.jog.enable-off", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableX), halCompId, "%s.jog.enable-x", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableY), halCompId, "%s.jog.enable-y", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableZ), halCompId, "%s.jog.enable-z", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableA), halCompId, "%s.jog.enable-a", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableB), halCompId, "%s.jog.enable-b", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogEnableC), halCompId, "%s.jog.enable-c", modname);

        //r |= _hal_pin_bit_newf(HAL_OUT, &(hal.memory.jog_enable_feedrate), hal_comp_id, "%s.jog.enable-feed-override", modname);
        //r |= _hal_pin_bit_newf(HAL_OUT, &(hal.memory.jog_enable_spindle), hal_comp_id, "%s.jog.enable-spindle-override", modname);

        r |= newFloatHalPin(HAL_OUT, &(hal.memory.jogScale), halCompId, "%s.jog.scale", modname);
        r |= newSigned32HalPin(HAL_OUT, &(hal.memory.jogCount), halCompId, "%s.jog.counts", modname);
        r |= newSigned32HalPin(HAL_OUT, &(hal.memory.jogCountNeg), halCompId, "%s.jog.counts-neg", modname);

        r |= newFloatHalPin(HAL_OUT, &(hal.memory.jogVelocity), halCompId, "%s.jog.velocity", modname);
        r |= newFloatHalPin(HAL_IN, &(hal.memory.jogMaxVelocity), halCompId, "%s.jog.max-velocity", modname);
        r |= newFloatHalPin(HAL_OUT, &(hal.memory.jogIncrement), halCompId, "%s.jog.increment", modname);

        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusX), halCompId, "%s.jog.plus-x", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusY), halCompId, "%s.jog.plus-y", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusZ), halCompId, "%s.jog.plus-z", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusA), halCompId, "%s.jog.plus-a", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusB), halCompId, "%s.jog.plus-b", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogPlusC), halCompId, "%s.jog.plus-c", modname);

        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusX), halCompId, "%s.jog.minus-x", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusY), halCompId, "%s.jog.minus-y", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusZ), halCompId, "%s.jog.minus-z", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusA), halCompId, "%s.jog.minus-a", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusB), halCompId, "%s.jog.minus-b", modname);
        r |= newBitHalPin(HAL_OUT, &(hal.memory.jogMinusC), halCompId, "%s.jog.minus-c", modname);

        assert (r == 0);

        return;
    }
}

// ----------------------------------------------------------------------

XhcWhb04b6::WhbContext Whb;

// ----------------------------------------------------------------------

static void Usage(char* name)
{
    fprintf(stderr, "%s version %s by Frederic RIBLE (frible@teaser.fr)\n", name, PACKAGE_VERSION);
    fprintf(stderr, "Usage: %s [-I ini-file] [-h] [-H] [-s 1|2]\n", name);
    //fprintf(stderr, " -I ini-file: configuration file defining the MPG keyboard layout\n");
    fprintf(stderr, " -h: usage (this)\n");
    fprintf(stderr, " -H: run in real-time HAL mode (run in simulation mode by default)\n");
    fprintf(stderr, " -x: wait for pendant detection before creating HAL pins\n");
    //fprintf(stderr, " -s: step sequence (*.001 unit):\n");
    fprintf(stderr, "     1: 1,10,100,1000 (default)\n");
    fprintf(stderr, "     2: 1,5,10,20\n\n");
    fprintf(stderr, "Configuration file section format:\n");
    fprintf(stderr, "[%s]\n", Whb.entity.configSectionName);
    fprintf(stderr, "BUTTON=XX:button-thename\n");
    fprintf(stderr, "...\n");
    fprintf(stderr, "    where XX=hexcode, thename=nameforbutton\n");
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

    while ((opt = getopt(argc, argv, "HhI:xs:")) != -1)
    {
        switch (opt)
        {
            case 'I':
                /*if (read_ini_file(optarg)) {
                    printf("Problem reading ini file: %s\n\n",optarg);
                    Usage(argv[0]);
                    exit(EXIT_FAILURE);
                }*/
                break;
            case 'H':
                Whb.hal.setIsSimulationMode(false);
                break;
            case 's':
                /*switch (optarg[0])
                {
                    case '1':
                        stepsize_sequence = stepsize_sequence_1;
                        break;
                    case '2':
                        stepsize_sequence = stepsize_sequence_2;
                        break;
                    default:
                        printf("Unknown sequence: %s\n\n", optarg);
                        Usage(argv[0]);
                        exit(EXIT_FAILURE);
                        break;
                }*/
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
        hal_ready(Whb.hal.entity.getHalComponentId());
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

        printf("%s: waiting for %s device\n", Whb.entity.name, Whb.entity.configSectionName);
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
                                                                    Whb.entity.usbVendorId,
                                                                    Whb.entity.usbProductId));
            libusb_free_device_list(devs, 1);
            if (Whb.usb.isDeviceOpen() == false)
            {
                if (Whb.usb.getWaitForPendantBeforeHAL())
                {
                    wait_secs++;
                    if (wait_secs >= MAX_WAIT_SECS / 2)
                    {
                        printf("%s: waiting for %s device (%d)\n", Whb.entity.name,
                               Whb.entity.configSectionName, wait_secs);
                    }
                    if (wait_secs > MAX_WAIT_SECS)
                    {
                        printf("%s: MAX_WAIT_SECS exceeded, exiting\n", Whb.entity.name);
                        exit(EXIT_FAILURE);
                    }
                }
                sleep(1);
            }
        } while ((Whb.usb.isDeviceOpen() == false) && Whb.isRunning());

        printf("%s: found %s device\n", Whb.entity.name, Whb.entity.configSectionName);

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
            hal_ready(Whb.hal.entity.getHalComponentId());
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
            printf("%s: connection lost, cleaning up\n", Whb.entity.name);
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
