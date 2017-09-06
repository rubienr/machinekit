/*
   XHC-WHB04B-6 Wireless MPG pendant LinuxCNC HAL module for LinuxCNC.
   Based on XHC-HB04. The implementation supports no configuration file,
   nor other variations as XHC-WHB04B-4.

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

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include  <iomanip>
#include <assert.h>
#include <signal.h>
#include <libusb.h>
#include <unistd.h>
#include <stdarg.h>
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
using std::setprecision;
using std::fixed;

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

class WhbHandwheelContiunuousModeStepSize;

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

class OnUsbInputPackageReceivedHandler;

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
    {
    }
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
    void halInit(WhbSoftwareButton* softwareButtons, size_t buttonsCount, const WhbKeyCodes& codes);

    bool isSimulationModeEnabled() const;

    //! indicates the program has been invoked in hal mode or normal
    void setSimulationMode(bool isSimulationMode);

    int getHalComponentId() const;

    const char* getHalComponentName() const;

    void setEnableVerbose(bool enable);

private:
    bool mIsSimulationMode;
    const char* mName;
    int          mHalCompId;
    std::ostream mDevNull;
    std::ostream* mHalCout;

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

    WhbKeyCode(uint8_t code, const char* text, const char* altText);

    WhbKeyCode(const WhbKeyCode& other);
};

// ----------------------------------------------------------------------

//! meta-button state which is dependent on the "Fn" modifier button's state
class WhbSoftwareButton
{
public:
    const WhbKeyCode& key;
    const WhbKeyCode& modifier;

    WhbSoftwareButton(const WhbKeyCode& key, const WhbKeyCode& modifier);
};

// ----------------------------------------------------------------------

//! rotary axis selection button related parameters
class WhbAxisRotaryButtonCodes
{
public:
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

    WhbAxisRotaryButtonCodes();
};

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
};

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

    WhbButtonsCode();

};

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

class WhbVelocityComputation
{
public:
    hal_s32_t      last_jog_counts;
    struct timeval last_tv;

    WhbVelocityComputation();
};

// ----------------------------------------------------------------------

//! If hand wheel is in step mode (toggled by Step/Continuous" button) this speed setting is applied.
//! In step mode the step is distance.
class WhbHandwheelStepModeStepSize
{
public:
    typedef enum
    {
        RotaryButton0001 = 0, RotaryButton0010 = 1, RotaryButton0100 = 2, RotaryButton100 = 3, RotaryButtonUndefined = 4
    } ButtonCodeToStepIndex;

    float getStepSize(ButtonCodeToStepIndex buttonPosition) const;

    WhbHandwheelStepModeStepSize();

private:
    const float mSequence[5];
};

// ----------------------------------------------------------------------

//! If hand wheel is in continuous mode (toggled by Step/Continuous" button) this speed setting is applied.
//! In continuous mode the step speed is in percent.
class WhbHandwheelContiunuousModeStepSize
{
public:

    typedef enum
    {
        RotaryButton2percent,
        RotaryButton5percent,
        RotaryButton10percent,
        RotaryButton30percent,
        RotaryButton60percent,
        RotaryButton100percent,
        RotaryButtonUndefined
    } ButtonCodeToStepIndex;

    uint8_t getStepSize(ButtonCodeToStepIndex buttonPosition) const;

    WhbHandwheelContiunuousModeStepSize();

private:
    const uint8_t sequence[7];
};

// ----------------------------------------------------------------------

class WhbStepModeStepSize
{
    friend XhcWhb04b6::WhbContext;
    friend XhcWhb04b6::WhbButtonsState;

public:
    WhbStepModeStepSize();

private:
    WhbHandwheelStepModeStepSize        step;
    WhbHandwheelContiunuousModeStepSize continuous;
};

// ----------------------------------------------------------------------

//! step handling related parameters
class WhbStepHandler
{
    friend XhcWhb04b6::WhbContext;

public:
    WhbStepModeStepSize stepSize;

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

    uint8_t getButton1Code() const;

    uint8_t getButton2Code() const;

    uint8_t getAxisRotaryButtonCode() const;

    uint8_t getFeedRotaryButtonCode() const;

    WhbSoftwareButton& getSoftwareButton() const;

    //! stores the buttons state and updates the software button state and step speed state
    void updateButtonState(uint8_t button1Code, uint8_t button2Code, uint8_t currentAxisRotaryButtonCode,
                           uint8_t currentFeedRotaryButtonCode);

    //! returns the step size value according to the current button state considering "Fn" modifer
    float getStepSize();

    //! initialized lookup references are needed to resolve software button state and step speed state
    WhbButtonsState(const WhbButtonsCode& buttonCodesLookup,
                    const WhbAxisRotaryButtonCodes& axisRotaryButtonCodesLookup,
                    const WhbFeedRotaryButtonCodes& feedRotaryButtonCodesLookup,
                    const WhbHandwheelStepModeStepSize& stepSizeLookup,
                    const WhbHandwheelContiunuousModeStepSize& continuousStepSizeLookup);

private:
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

    void setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex stepSize);

    void setCurrentContinuousModeStepSize(WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex stepSize);
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

//! \see DisplayIndicatorBitFields::stepMode
typedef enum
{
    //! displays "CONT <xx>%"
        CONTINUOUS             = 0x00,
    //! displays "STP: <x.xxxx>"
        STEP                   = 0x01,
    //! displays "MPG <xx>%"
        MANUAL_PULSE_GENERATOR = 0x02

} DisplayIndicatorStepMode;

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
    uint16_t header;
    uint8_t  dayOfMonth;

    DisplayIndicator displayModeFlags;

    WhbUsbOutPackageAxisCoordinate row1Coordinate;
    WhbUsbOutPackageAxisCoordinate row2Coordinate;
    WhbUsbOutPackageAxisCoordinate row3Coordinate;

    //! printed on feed+/- button pressed
    uint16_t feedRate;
    //! printed on spindle+/- button pressed
    uint16_t spindleSpeed;

    WhbUsbOutPackageData();

    void clear();

private:
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
    const unsigned char header;
    const unsigned char dayOfMonth;
    const unsigned char buttonKeyCode1;
    const unsigned char buttonKeyCode2;
    const unsigned char rotaryButtonFeedKeyCode;
    const unsigned char rotaryButtonAxisKeyCode;
    const signed char   stepCount;
    const unsigned char crc;

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
    unsigned char         asBuffer[sizeof(WhbUsbInPackage)];

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

}__attribute__((packed));

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

}__attribute__((packed));

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

    WhbUsb(const char* name, OnUsbInputPackageReceivedHandler& onDataReceivedCallback, WhbHalMemory& halMemory);

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

    void cbResponseIn(struct libusb_transfer* transfer);

    void setSimulationMode(bool isSimulationMode);

    void setIsRunning(bool enableRunning);

    void requestTermination();

    bool setupAsyncTransfer();

    void sendDisplayData();

    void enableVerboseTx(bool enable);

    void enableVerboseRx(bool enable);

    void enableVerboseInit(bool enable);

    bool init();

    void setWaitWithTimeout(uint8_t waitSecs);

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
    OnUsbInputPackageReceivedHandler& mDataHandler;
    WhbHalMemory                    & mHalMemory;
    struct libusb_transfer          * inTransfer;
    struct libusb_transfer          * outTransfer;
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

class OnUsbInputPackageReceivedHandler
{
public:
    virtual void handleInputData(const WhbUsbInPackage& inPackage) = 0;

    virtual ~OnUsbInputPackageReceivedHandler()
    {
    }
};

// ----------------------------------------------------------------------

//! program context
class WhbContext :
    public OnUsbInputPackageReceivedHandler
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
    virtual void handleInputData(const WhbUsbInPackage& inPackage) override;

    //! todo: doxy
    void initWhb();

    //! todo: doxy
    void halInit();

    //! todo: doxy
    void teardownHal();

    //! todo: doxy
    void cbResponseIn(struct libusb_transfer* transfer);

    //! todo: doxy
    bool enableReceiveAsyncTransfer();

    //! todo: doxy
    void computeVelocity();

    // todo: refactor me
    void sendDisplayData();

    //! todo: doxy
    void handleStep();

    //! todo: doxy
    void linuxcncSimulate();

    //! todo: doxy
    void requestTermination(int signal = -42);

    //! todo: doxy, move
    bool isRunning() const;

    int run();

    bool isSimulationModeEnabled() const;

    void setSimulationMode(bool enableSimulationMode);

    void enableVerboseRx(bool enable);

    void enableVerboseTx(bool enable);

    void enableVerboseHalInit(bool enable);

    void enableVerboseInit(bool enable);

    void setWaitWithTimeout(uint8_t waitSecs = 3);

private:
    WhbHal hal;
    const char* mName;
    WhbUsb                 usb;
    const WhbKeyCodes      mKeyCodes;
    WhbStepHandler         mStepHandler;
    WhbSoftwareButton      mSoftwareButtons[31];
    WhbButtonsState        mPreviousButtonCodes;
    WhbButtonsState        mCurrentButtonCodes;
    WhbVelocityComputation velocityComputation;
    bool                   mIsRunning;
    bool                   mIsSimulationMode;
    std::ostream           devNull;
    std::ostream* verboseTxOut;
    std::ostream* verboseRxOut;
    std::ostream* verboseHalInitOut;
    std::ostream* verboseInitOut;

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

};

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

WhbSoftwareButton::WhbSoftwareButton(const WhbKeyCode& key, const WhbKeyCode& modifier) :
    key(key),
    modifier(modifier)
{
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
{
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

float WhbHandwheelStepModeStepSize::getStepSize(ButtonCodeToStepIndex buttonPosition) const
{
    return mSequence[buttonPosition];
}

// ----------------------------------------------------------------------

WhbHandwheelStepModeStepSize::WhbHandwheelStepModeStepSize() :
    mSequence{0.001, 0.01, 0.1, 1.0, 0.0}
{
}

// ----------------------------------------------------------------------

uint8_t WhbHandwheelContiunuousModeStepSize::getStepSize(ButtonCodeToStepIndex buttonPosition) const
{
    return sequence[buttonPosition];
}

// ----------------------------------------------------------------------

WhbHandwheelContiunuousModeStepSize::WhbHandwheelContiunuousModeStepSize() :
    sequence{2, 5, 10, 30, 60, 100, 0}
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

void WhbButtonsState::setCurrentStepModeStepSize(WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex stepSize)
{
    currentStepModeSize       = stepSize;
    currentContinuousModeSize = WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined;
}

// ----------------------------------------------------------------------

void
WhbButtonsState::setCurrentContinuousModeStepSize(WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex stepSize)
{
    currentContinuousModeSize = stepSize;
    currentStepModeSize       = WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getButton1Code() const
{
    return currentButton1Code;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getButton2Code() const
{
    return currentButton2Code;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getAxisRotaryButtonCode() const
{
    return currentAxisCode;
}

// ----------------------------------------------------------------------

uint8_t WhbButtonsState::getFeedRotaryButtonCode() const
{
    return currentFeedCode;
}

// ----------------------------------------------------------------------

WhbSoftwareButton& WhbButtonsState::getSoftwareButton() const
{
    assert(softwareButton != nullptr);
    return *softwareButton;
}

// ----------------------------------------------------------------------

void WhbButtonsState::updateButtonState(uint8_t button1Code, uint8_t button2Code, uint8_t currentAxisRotaryButtonCode,
                                        uint8_t currentFeedRotaryButtonCode)
{
    // TODO:
    // find out modifier, see print button
    // then other key is the key
    // find the software button accordingly
    // update the speed rotary button, see print rotary button
    // update the axis rotary button

}

// ----------------------------------------------------------------------

float WhbButtonsState::getStepSize()
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

// ----------------------------------------------------------------------

WhbButtonsState::WhbButtonsState(const WhbButtonsCode& buttonCodesLookup,
                                 const WhbAxisRotaryButtonCodes& axisRotaryButtonCodesLookup,
                                 const WhbFeedRotaryButtonCodes& feedRotaryButtonCodesLookup,
                                 const WhbHandwheelStepModeStepSize& stepSizeLookup,
                                 const WhbHandwheelContiunuousModeStepSize& continuousStepSizeLookup) :
    buttonCodesLookup(buttonCodesLookup),
    axisRotaryButtonCodesLookup(axisRotaryButtonCodesLookup),
    feedRotaryButtonCodesLookup(feedRotaryButtonCodesLookup),
    stepModeStepSizeLookup(stepSizeLookup),
    continuousModeStepSizeLookup(continuousStepSizeLookup),
    currentContinuousModeSize(WhbHandwheelContiunuousModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined),
    currentStepModeSize(WhbHandwheelStepModeStepSize::ButtonCodeToStepIndex::RotaryButtonUndefined),
    currentButton1Code(buttonCodesLookup.undefined.code),
    currentButton2Code(buttonCodesLookup.undefined.code),
    currentAxisCode(axisRotaryButtonCodesLookup.undefined.code),
    currentFeedCode(feedRotaryButtonCodesLookup.undefined.code)
{
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

WhbUsb::WhbUsb(const char* name, OnUsbInputPackageReceivedHandler& onDataReceivedCallback, WhbHalMemory& halMemory) :
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
    mHalMemory(halMemory),
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

    if (mIsSimulationMode)
    {
        *verboseTxOut << dec << "out bytes count " << sizeof(outputPackageBuffer.asBlockArray)
                      << endl << "0x" << outputPackageBuffer.asBlocks << endl << outputPackageData << endl;
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

void WhbContext::halInit()
{
    hal.halInit(mSoftwareButtons, sizeof(mSoftwareButtons) / sizeof(WhbSoftwareButton), mKeyCodes);
}

// ----------------------------------------------------------------------

void WhbContext::teardownHal()
{
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

    uint8_t modifierCode       = mKeyCodes.buttons.undefined.code;
    uint8_t keyCode            = mKeyCodes.buttons.undefined.code;

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

    // update current axis and step to hal
    *(hal.memory.jogCount) += inPackage.stepCount;
    *(hal.memory.jogCountNeg)  = -*(hal.memory.jogCount);
    *(hal.memory.jogEnableOff) = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.off.code);
    *(hal.memory.jogEnableX)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.x.code);
    *(hal.memory.jogEnableY)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.y.code);
    *(hal.memory.jogEnableZ)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.z.code);
    *(hal.memory.jogEnableA)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.a.code);
    *(hal.memory.jogEnableB)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.b.code);
    *(hal.memory.jogEnableC)   = (inPackage.rotaryButtonAxisKeyCode == mKeyCodes.axis.c.code);

    //! print human readable data
    if (hal.isSimulationModeEnabled())
    {
        if (inPackage.rotaryButtonFeedKeyCode != 0)
        {
            ios init(NULL);
            init.copyfmt(*verboseRxOut);
            *verboseRxOut << " delta " << setfill(' ') << setw(2) << (unsigned short)inPackage.rotaryButtonFeedKeyCode;
            verboseRxOut->copyfmt(init);
        }
        *verboseRxOut << " => ";
        printInputData(inPackage);
    }

    //! update all buttons state to hal
    int      buttonsCount = sizeof(mSoftwareButtons) / sizeof(WhbSoftwareButton);
    for (int idx          = 0; idx < buttonsCount; idx++)
    {
        if ((mSoftwareButtons[idx].key.code == keyCode) && (mSoftwareButtons[idx].modifier.code == modifierCode))
        {
            *(hal.memory.button_pin[idx]) = true;
            if (hal.isSimulationModeEnabled())
            {
                *verboseRxOut << " pressed ";
                printPushButtonText(keyCode, modifierCode);
            }
        }
        else
        {
            *(hal.memory.button_pin[idx]) = false;
        }
    }

    if (hal.isSimulationModeEnabled())
    {
        *verboseRxOut << endl;
    }
}

// ----------------------------------------------------------------------

void WhbContext::initWhb()
{
    //stepHandler.old_inc_step_status = -1;
    //gettimeofday(&sleepState.mLastWakeupTimestamp, nullptr);
    mIsRunning = true;
    usb.setIsRunning(true);
}

// ----------------------------------------------------------------------

void WhbContext::requestTermination(int signal)
{
    if (signal >= 0)
    {
        *verboseInitOut << "termination requested upon signal number " << signal << " ..." << endl;
    }
    else
    {
        *verboseInitOut << "termination requested ... " << endl;
    }
    usb.requestTermination();
    mIsRunning = false;

}

// ----------------------------------------------------------------------

bool WhbContext::isRunning() const
{
    return mIsRunning;
}

// ----------------------------------------------------------------------

WhbContext::WhbContext() :
    hal(),
    mName("XHC-WHB04B-6"),
    usb(mName, *this, hal.memory),
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
                     WhbSoftwareButton(mKeyCodes.buttons.undefined, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.probe_z, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.probe_z, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.macro10, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.macro10, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.manual_pulse_generator, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.manual_pulse_generator, mKeyCodes.buttons.function),
                     WhbSoftwareButton(mKeyCodes.buttons.step_continuous, mKeyCodes.buttons.undefined),
                     WhbSoftwareButton(mKeyCodes.buttons.step_continuous, mKeyCodes.buttons.function)
    },
    mPreviousButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                         mStepHandler.stepSize.continuous),
    mCurrentButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                        mStepHandler.stepSize.continuous),
    velocityComputation(),
    mIsRunning(false),
    mIsSimulationMode(false),
    devNull(nullptr),
    verboseTxOut(&devNull),
    verboseRxOut(&devNull),
    verboseHalInitOut(&devNull),
    verboseInitOut(&devNull)
{
    setSimulationMode(true);
    enableVerboseRx(false);
    enableVerboseTx(false);
    enableVerboseInit(false);
    enableVerboseHalInit(false);
}

WhbContext::~WhbContext()
{
}

// ----------------------------------------------------------------------

void WhbContext::sendDisplayData()
{
    usb.sendDisplayData();
}



// ----------------------------------------------------------------------

void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode, std::ostream& out)
{
    ios init(NULL);
    init.copyfmt(out);
    int indent = 10;
    out << setfill(' ');

    // no key code
    if (keyCode == mKeyCodes.buttons.undefined.code)
    {
        // modifier specified
        if (modifierCode == mKeyCodes.buttons.function.code)
        {
            out << setw(indent) << mKeyCodes.buttons.function.text;
        }
            // no modifier specified
        else
        {
            out << setw(indent) << mKeyCodes.buttons.undefined.text;
        }
        return;
    }

    // find key code
    const WhbKeyCode* whbKeyCode = (WhbKeyCode*)&mKeyCodes.buttons;
    while (whbKeyCode->code != 0)
    {
        if (whbKeyCode->code == keyCode)
        {
            break;
        }
        whbKeyCode++;
    }
    // print key text
    if (modifierCode == mKeyCodes.buttons.function.code)
    {
        out << setw(indent) << whbKeyCode->altText;
    }
    else
    {
        out << setw(indent) << whbKeyCode->text;
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

    os << ((coordinate.coordinateSign == 1) ? "-" : "") << static_cast<unsigned short>(coordinate.integerValue) << "."
       << setfill('0') << setw(4) << static_cast<unsigned short>(coordinate.fractionValue);

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
    header     = 0xfdfe;
    dayOfMonth = 0x0c;
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

    os << hex << setfill('0') << "header       0x" << setw(2) << data.header << endl << "day of month   0x" << setw(2)
       << static_cast<unsigned short>(data.dayOfMonth) << endl << "status 0x" << setw(2)
       << static_cast<unsigned short>(data.displayModeFlags.asByte) << endl << dec << "coordinate1  "
       << data.row1Coordinate << endl << "coordinate2  " << data.row2Coordinate << endl << "coordinate3  "
       << data.row3Coordinate << endl << "feed rate        " << data.feedRate << endl << "spindle rps      "
       << data.spindleSpeed;

    os.copyfmt(init);
    return os;
}

// ----------------------------------------------------------------------

WhbUsbOutPackageBuffer::WhbUsbOutPackageBuffer() :
    asBlocks()
{
    /*cout << "sizeof usb data " << sizeof(WhbUsbOutPackageData) << endl
         << " blocks count   " << sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlockFields) << endl
         << " sizeof block   " << sizeof(WhbUsbOutPackageBlockFields) << endl
         << " sizeof blocks  " << sizeof(WhbUsbOutPackageBlocks) << endl
         << " sizeof array   " << sizeof(asBlockArray) << endl
         << " sizeof package " << sizeof(WhbUsbOutPackageData) << endl
         << " sizeof buffer  " << sizeof(asBytes) << endl;*/
    assert(sizeof(WhbUsbOutPackageBlocks) == sizeof(asBlockArray));
    size_t blocksCount = sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlockFields);
    assert ((sizeof(WhbUsbOutPackageData) + blocksCount) == sizeof(WhbUsbOutPackageBlocks));
}

// ----------------------------------------------------------------------

WhbUsbInPackage::WhbUsbInPackage() :
    header(0),
    dayOfMonth(0),
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
    dayOfMonth(notAvailable2),
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
        << static_cast<unsigned short>(inPackage.dayOfMonth) << " | ";
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
        << static_cast<unsigned short>(inPackage.dayOfMonth) << " " << setw(2)
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
    bool isHalReady = false;
    initWhb();
    halInit();

    if (!usb.isWaitForPendantBeforeHalEnabled() && !hal.isSimulationModeEnabled())
    {
        hal_ready(hal.getHalComponentId());
        isHalReady = true;
    }

    while (isRunning())
    {
        *(hal.memory.isPendantConnected) = 0;
        *(hal.memory.isPendantRequired)  = usb.isWaitForPendantBeforeHalEnabled();

        initWhb();
        if (false == usb.init())
        {
            return EXIT_FAILURE;
        }

        *(hal.memory.isPendantConnected) = 1;

        if (!isHalReady && !hal.isSimulationModeEnabled())
        {
            hal_ready(hal.getHalComponentId());
            isHalReady = true;
        }

        if (usb.isDeviceOpen())
        {
            *verboseInitOut << "enabling reception ...";
            if (!enableReceiveAsyncTransfer())
            {
                cerr << endl << "failed to enable reception" << endl;
                return EXIT_FAILURE;
            }
            *verboseInitOut << " ok" << endl;
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
    if (elapsed <= 0)
        return;

    float delta_pos = (*(hal.memory.jogCount) - velocityComputation.last_jog_counts) * *(hal.memory.jogScale);
    float velocity  = *(hal.memory.jogMaxVelocity) * 60.0f * *(hal.memory.jogScale);
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
    *(hal.memory.stepsize) = mCurrentButtonCodes.getStepSize() * 100;
    // todo: refactor me
    *(hal.memory.jogScale) = *(hal.memory.stepsize);// * 0.001f;
}

// ----------------------------------------------------------------------

bool WhbContext::enableReceiveAsyncTransfer()
{
    return usb.setupAsyncTransfer();
}

// ----------------------------------------------------------------------

void WhbContext::cbResponseIn(struct libusb_transfer* transfer)
{
    // pass transfer to usb data parser
    usb.cbResponseIn(transfer);
}

// ----------------------------------------------------------------------

void WhbContext::setSimulationMode(bool enableSimulationMode)
{
    mIsSimulationMode = enableSimulationMode;
    hal.setSimulationMode(mIsSimulationMode);
    usb.setSimulationMode(mIsSimulationMode);
}

// ----------------------------------------------------------------------


void WhbContext::setUsbContext(libusb_context* context)
{
    usb.setContext(context);
}

// ----------------------------------------------------------------------

libusb_device_handle* WhbContext::getUsbDeviceHandle()
{
    return usb.getDeviceHandle();
}

// ----------------------------------------------------------------------

libusb_context* WhbContext::getUsbContext()
{
    return usb.getContext();
}

// ----------------------------------------------------------------------

void WhbContext::process()
{
    if (usb.isDeviceOpen())
    {
        while (isRunning() && !usb.getDoReconnect())
        {
            struct timeval tv;
            tv.tv_sec  = 4;
            tv.tv_usec = 0;
            // TODO: investigate why no synchronization is implemented here
            int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
            assert((r == LIBUSB_SUCCESS) || (r == LIBUSB_ERROR_NO_DEVICE) || (r == LIBUSB_ERROR_BUSY) ||
                   (r == LIBUSB_ERROR_TIMEOUT) || (r == LIBUSB_ERROR_INTERRUPTED));
            computeVelocity();
            if (hal.isSimulationModeEnabled())
            {
                linuxcncSimulate();
            }
            //handleStep();
            sendDisplayData();
        }

        *(hal.memory.isPendantConnected) = 0;
        *verboseInitOut << "connection lost, cleaning up" << endl;
        struct timeval tv;
        tv.tv_sec  = 1;
        tv.tv_usec = 0;
        int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
        assert(0 == r);
        r = libusb_release_interface(getUsbDeviceHandle(), 0);
        assert((0 == r) || (r == LIBUSB_ERROR_NO_DEVICE));
        libusb_close(getUsbDeviceHandle());
        usb.setDeviceHandle(nullptr);
    }
}

// ----------------------------------------------------------------------

void WhbContext::teardownUsb()
{
    libusb_exit(getUsbContext());
    usb.setContext(nullptr);
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseRx(bool enable)
{
    usb.enableVerboseRx(enable);
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

void WhbContext::enableVerboseTx(bool enable)
{
    usb.enableVerboseTx(enable);
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

void WhbContext::enableVerboseHalInit(bool enable)
{
    hal.setEnableVerbose(enable);

    if (enable)
    {
        verboseHalInitOut = &std::cout;
    }
    else
    {
        verboseHalInitOut = &devNull;
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableVerboseInit(bool enable)
{
    usb.enableVerboseInit(enable);
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

void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode)
{
    printPushButtonText(keyCode, modifierCode, *verboseRxOut);
}

// ----------------------------------------------------------------------

void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode)
{
    printRotaryButtonText(keyCodeBase, keyCode, *verboseRxOut);
}

// ----------------------------------------------------------------------

void WhbContext::printInputData(const WhbUsbInPackage& inPackage)
{
    printInputData(inPackage, *verboseRxOut);
}

// ----------------------------------------------------------------------

void WhbContext::printHexdump(const WhbUsbInPackage& inPackage)
{
    printHexdump(inPackage, *verboseRxOut);
}

void WhbContext::setWaitWithTimeout(uint8_t waitSecs)
{
    usb.setWaitWithTimeout(waitSecs);
}

bool WhbContext::isSimulationModeEnabled() const
{
    return mIsSimulationMode;
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

void WhbUsb::cbResponseIn(struct libusb_transfer* transfer)
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
                    *(mHalMemory.sleeping) = 1;
                    if (mIsSimulationMode)
                    {
                        struct timeval now;
                        gettimeofday(&now, nullptr);
                        *verboseTxOut << "going to sleep: device was idle for "
                                      << (now.tv_sec - sleepState.mLastWakeupTimestamp.tv_sec) << " seconds" << endl;
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
                            *verboseTxOut << "woke up: device was sleeping for "
                                          << (now.tv_sec - sleepState.mLastWakeupTimestamp.tv_sec) << " seconds"
                                          << endl;
                        }
                        gettimeofday(&sleepState.mLastWakeupTimestamp, nullptr);
                    }

                }
                // pass structured transfer to usb data handler
                mDataHandler.handleInputData(inputPackageBuffer.asFields);
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
        *verboseInitOut << "pausing " << pauseSecs << "s, waiting for device to be gone ...";
        while ((pauseSecs--) >= 0)
        {
            *verboseInitOut << "." << std::flush;
            sleep(1);
        }
        setDoReconnect(false);
        *verboseInitOut << " done" << endl;
    }

    *verboseInitOut << "init usb context ...";
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

    if (isWaitWithTimeout)
    {
        *verboseInitOut << "waiting maximum " << static_cast<unsigned short>(mWaitSecs) << "s for device "
                        << mName << getUsbVendorId() << " productId=" << usbProductId << " ...";
    }
    else
    {
        *verboseInitOut << "not waiting for device, will continue" << static_cast<unsigned short>(mWaitSecs)
                        << "s for device " << mName << usbVendorId << " productId=" << usbProductId << " ...";
    }

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
    *verboseInitOut << " ok" << endl << mName << " device found" << endl;

    if (isDeviceOpen())
    {
        *verboseInitOut << "detaching active kernel driver ...";
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
        *verboseInitOut << "claiming interface ...";
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
    mHalCompId(-1),
    mDevNull(nullptr),
    mHalCout(&mDevNull)
{
}

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
    *mHalCout << "allocated hal pin " << pin_name << endl;
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
        *mHalCout << "registered " << pin_name << endl;
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
        *mHalCout << "registered " << pin_name << endl;
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
        *mHalCout << "registered " << pin_name << endl;
        return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
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

void WhbHal::halInit(WhbSoftwareButton* softwareButtons, size_t buttonsCount, const WhbKeyCodes& mKeyCodes)
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
        if (&softwareButtons[idx].modifier == &mKeyCodes.buttons.undefined)
        {
            buttonName = softwareButtons[idx].key.text;
        }
        else
        {
            buttonName = softwareButtons[idx].key.altText;
        }
        int r = newBitHalPin(HAL_OUT, &(memory.button_pin[idx]), mHalCompId, "%s.%s", mName, buttonName);
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

} // namespace

// ----------------------------------------------------------------------

XhcWhb04b6::WhbContext Whb;

// ----------------------------------------------------------------------

static int printUsage(char* programName, bool isError = false)
{
    std::ostream* os = &std::cout;
    if (isError)
    {
        os = &cerr;
    }
    *os << programName << " version " << PACKAGE_VERSION << " 2017 by Raoul Rubien (github.com/rubienr)" << endl
        << "Usage: " << programName << " [-h] | [-H] [-x] [[-u|-U] [-p] | [-a] | [-s]] " << endl
        << " -h usage help text" << endl << " -H run " << Whb.getName() << "in HAL-mode instead of interactive mode"
        << endl << " -t wait for USB device before processing with HAL initialization" << endl
        << " -u print received data" << endl << " -U print received and transmitted data" << endl
        << " -p print initialized HAL pins " << endl << " -a enable all verbose facilities" << "-s be silent" << endl;

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
    Whb.cbResponseIn(transfer);
}

// ----------------------------------------------------------------------

int main(int argc, char** argv)
{
    const char* optargs = "HtuUpahs";
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
                Whb.enableVerboseHalInit(true);
                break;
            case 'a':
                Whb.enableVerboseInit(true);
                Whb.enableVerboseRx(true);
                Whb.enableVerboseTx(true);
                Whb.enableVerboseHalInit(true);
                break;
            case 's':
                break;
            case 'h':
                return printUsage(argv[0]);
                break;
            default:
                return printUsage(argv[0], true);
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
