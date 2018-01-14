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

// local includes
#include "./hal.h"
#include "./usb.h"
#include "./pendant.h"

// forward declarations

namespace XhcWhb04b6 {

// forward declarations

// ----------------------------------------------------------------------

class WhbKeyEventListener
{
public:
    //! Called when button is pressed.
    //! \param softwareButton the button pressed
    //! \return true if a subsequent re-evaluation should be performed.
    //! Example: A button event changes the feed rotary buttons step mode from
    //! step to continuous. The button must be re-evaluated, otherwise the
    //! button state remains untouched until the next button's event.
    //virtual bool onButtonPressedEvent(const WhbSoftwareButton& softwareButton) = 0;
    //! Called when button is released.
    //! \param softwareButton the button released
    //! \return true if a subsequent re-evaluation should be performed.
    //! Example: A button event changes the feed rotary buttons step mode from
    //! step to continuous. The button must be re-evaluated, otherwise the
    //! button state remains untouched until the next button's event.
    //virtual bool onButtonReleasedEvent(const WhbSoftwareButton& softwareButton) = 0;
    //virtual void onAxisActiveEvent(const WhbKeyCode& axis) = 0;
    //virtual void onAxisInactiveEvent(const WhbKeyCode& axis) = 0;
    //virtual void onFeedActiveEvent(const WhbKeyCode& axis) = 0;
    //virtual void onFeedInactiveEvent(const WhbKeyCode& axis) = 0;
    //virtual void onJogDialEvent(int8_t delta) = 0;
    virtual ~WhbKeyEventListener();
};

// ----------------------------------------------------------------------

class UsbInputPackageInterpreted
{
public:
    //virtual void onDataInterpreted() = 0;
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
    //size_t getSoftwareButtonIndex(uint8_t keyCode) const;
    void initWhb();
    void initHal();
    void teardownHal();
    bool enableReceiveAsyncTransfer();
    void updateDisplay();
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
    void printCrcDebug(const WhbUsbInPackage& inPackage, const WhbUsbOutPackageData& outPackageBuffer) const;
    void offerHalMemory();

private:
    const char* mName;
    WhbHal                  mHal;
    const WhbKeyCodes       mKeyCodes;
    const WhbStepHandler    mStepHandler;
    const WhbSoftwareButton mSoftwareButtons[32];
    WhbUsb                  mUsb;
    bool                    mIsRunning;
    bool                    mIsSimulationMode;
    std::ostream            mDevNull;
    std::ostream              * mTxCout;
    std::ostream              * mRxCout;
    std::ostream              * mKeyEventCout;
    std::ostream              * mHalInitCout;
    std::ostream              * mInitCout;
    WhbKeyEventListener       & keyEventReceiver;
    UsbInputPackageListener   & packageReceivedEventReceiver;
    UsbInputPackageInterpreted& packageInterpretedEventReceiver;
    bool    mIsCrcDebuggingEnabled;
    Pendant mPendant;

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
}
