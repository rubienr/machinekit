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

// local includes

// forward declarations
struct libusb_device_handle;
struct libusb_context;
struct libusb_transfer;

namespace XhcWhb04b6 {

// forward declarations
class WhbHalMemory;
class WhbUsb;

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

class UsbInputPackageListener
{
public:
    //! callback with structured input data
    virtual void onInputDataReceived(const WhbUsbInPackage& inPackage) = 0;

    virtual ~UsbInputPackageListener();
};

// ----------------------------------------------------------------------

class UsbRawInputListener
{
public:
    //! callback with raw input data
    virtual void onUsbDataReceived(struct libusb_transfer* transfer) = 0;

    virtual ~UsbRawInputListener();
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

//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union WhbUsbOutPackageBuffer
{
public:
    WhbUsbOutPackageBlock  asBlockArray[sizeof(WhbUsbOutPackageBlocks) / sizeof(WhbUsbOutPackageBlock)];
    WhbUsbOutPackageBlocks asBlocks;
    WhbUsbOutPackageBuffer();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! Convenience structure for casting data in package stream.
//! Caution: do not reorder fields!
union WhbUsbInPackageBuffer
{
public:
    const WhbUsbInPackage asFields;
    uint8_t               asBuffer[sizeof(WhbUsbInPackage)];
    WhbUsbInPackageBuffer();
} __attribute__((packed));

// ----------------------------------------------------------------------

//! pendant sleep/idle state parameters
class WhbSleepDetect
{
    friend WhbUsb;

public:

    WhbSleepDetect();

private:
    bool           mDropNextInPackage;
    struct timeval mLastWakeupTimestamp;
};

// ----------------------------------------------------------------------

//! USB related parameters
class WhbUsb : public UsbRawInputListener
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
    void onUsbDataReceived(struct libusb_transfer* transfer) override;
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
    void (* const mRawDataCallback)(struct libusb_transfer*);
    WhbHalMemory          * mHalMemory;
    struct libusb_transfer* inTransfer;
    struct libusb_transfer* outTransfer;
    std::ostream devNull;
    std::ostream* verboseTxOut;
    std::ostream* verboseRxOut;
    std::ostream* verboseInitOut;
    const char  * mName;
    uint8_t mWaitSecs;
};

// ----------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageAxisCoordinate& coordinate);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageData& data);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlockFields& block);
std::ostream& operator<<(std::ostream& os, const WhbUsbOutPackageBlocks& blocks);
}
