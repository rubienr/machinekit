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

#include "./xhc-whb04b6.h"

// system includes
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <libusb.h>
#include <bitset>

// 3rd party includes

// local library includes

// local includes

using std::endl;

namespace XhcWhb04b6 {

// ----------------------------------------------------------------------

UsbInputPackageInterpreted::~UsbInputPackageInterpreted()
{
}

// ----------------------------------------------------------------------

WhbKeyEventListener::~WhbKeyEventListener()
{
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
    std::ios init(NULL);
    init.copyfmt(*mRxCout);
    *mRxCout << std::setfill('0') << std::hex;

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
        *mRxCout << "0x key " << std::setw(8) << static_cast<unsigned short>(inPackage.buttonKeyCode1)
                 << " random " << std::setw(8) << static_cast<unsigned short>(inPackage.randomByte)
                 << " crc " << std::setw(8) << static_cast<unsigned short>(inPackage.crc)
                 << " delta " << std::setw(8) << static_cast<unsigned short>(delta.to_ulong()) << endl;

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
                 << " = calculated delta " << std::setw(2)
                 << static_cast<unsigned short>(keyXorNonSeedAndRandom.to_ulong())
                 << " vs " << std::setw(2) << static_cast<unsigned short>(delta.to_ulong())
                 << ((keyXorNonSeedAndRandom == delta) ? " OK" : " FAIL") << endl
                 << "calculated crc         " << calculatedCrcBitset << " " << std::setw(2) << calculatedCrc << " vs "
                 << std::setw(2)
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
    //if (mIsSimulationMode)
    //{
    *mRxCout << "in    ";
    printHexdump(inPackage);
    if (inPackage.rotaryButtonFeedKeyCode != KeyCodes::Feed.undefined.code)
    {
        std::ios init(NULL);
        init.copyfmt(*mRxCout);
        *mRxCout << " delta " << std::setfill(' ') << std::setw(2)
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
    //}

    uint8_t keyCode      = inPackage.buttonKeyCode1;
    uint8_t modifierCode = inPackage.buttonKeyCode2;

    // in case key code == undefined
    if (keyCode == KeyCodes::Buttons.undefined.code)
    {
        // swap codes
        keyCode      = modifierCode;
        modifierCode = KeyCodes::Buttons.undefined.code;
    }

    // in case key code == "fn" and modifier == defined
    if ((keyCode == KeyCodes::Buttons.function.code) &&
        (modifierCode != KeyCodes::Buttons.undefined.code))
    {
        // swap codes
        keyCode      = modifierCode;
        modifierCode = KeyCodes::Buttons.function.code;
    }

    // in case of key code == defined and key code != "fn" and modifier == defined and modifier != "fn"
    if ((keyCode != KeyCodes::Buttons.undefined.code) &&
        (modifierCode != KeyCodes::Buttons.undefined.code) &&
        (modifierCode != KeyCodes::Buttons.function.code))
    {
        // last key press overrules last but one key press
        keyCode      = modifierCode;
        modifierCode = KeyCodes::Buttons.undefined.code;
    }

    if (keyCode == KeyCodes::Buttons.undefined.code)
    {
        assert(modifierCode == KeyCodes::Buttons.undefined.code);
    }

    if (keyCode == KeyCodes::Buttons.function.code)
    {
        assert(modifierCode == KeyCodes::Buttons.undefined.code);
    }

    mPendant.processEvent(keyCode, modifierCode,
                          inPackage.rotaryButtonAxisKeyCode,
                          inPackage.rotaryButtonFeedKeyCode,
                          inPackage.stepCount);
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
    mDevNull(nullptr),
    mTxCout(&mDevNull),
    mRxCout(&mDevNull),
    mKeyEventCout(&mDevNull),
    mHalInitCout(&mDevNull),
    mInitCout(&mDevNull),
    keyEventReceiver(*this),
    packageReceivedEventReceiver(*this),
    packageInterpretedEventReceiver(*this),
    mIsCrcDebuggingEnabled(false),
    mPendant(mHal, mUsb.getOutputPackageData())
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

void WhbContext::updateDisplay()
{
    if (mIsRunning)
    {
        mPendant.updateDisplayData();
    }
    else
    {
        mPendant.clearDisplayData();
    }
    mUsb.sendDisplayData();
}

// ----------------------------------------------------------------------

void WhbContext::printPushButtonText(uint8_t keyCode, uint8_t modifierCode, std::ostream& out)
{
    std::ios init(NULL);
    init.copyfmt(out);
    int indent = 15;
    out << std::setfill(' ');

    // no key code
    if (keyCode == mKeyCodes.buttons.undefined.code)
    {
        out << std::setw(indent) << "";
        return;
    }

    const WhbKeyCode& whbKeyCode = mKeyCodes.buttons.getKeyCode(keyCode);

    // print key text
    if (modifierCode == mKeyCodes.buttons.function.code)
    {
        out << std::setw(indent) << whbKeyCode.altText;
    }
    else
    {
        out << std::setw(indent) << whbKeyCode.text;
    }
    out.copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbContext::printRotaryButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode, std::ostream& out)
{
    std::ios init(NULL);
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
    out << std::setw(5) << whbKeyCode->text << "(" << std::setw(4) << whbKeyCode->altText << ")";
    out.copyfmt(init);
}


// ----------------------------------------------------------------------

DisplayIndicator::DisplayIndicator() :
    asByte(0)
{
}

// ----------------------------------------------------------------------

void WhbContext::printInputData(const WhbUsbInPackage& inPackage, std::ostream& out)
{
    std::ios init(NULL);
    init.copyfmt(out);

    out << "| " << std::setfill('0') << std::hex << std::setw(2) << static_cast<unsigned short>(inPackage.header)
        << " | " << std::setw(2)
        << static_cast<unsigned short>(inPackage.randomByte) << " | ";
    out.copyfmt(init);
    printPushButtonText(inPackage.buttonKeyCode1, inPackage.buttonKeyCode2, out);
    out << " | ";
    printPushButtonText(inPackage.buttonKeyCode2, inPackage.buttonKeyCode1, out);
    out << " | ";
    printRotaryButtonText((WhbKeyCode*)&mKeyCodes.feed, inPackage.rotaryButtonFeedKeyCode, out);
    out << " | ";
    printRotaryButtonText((WhbKeyCode*)&mKeyCodes.axis, inPackage.rotaryButtonAxisKeyCode, out);
    out << " | " << std::setfill(' ') << std::setw(3) << static_cast<short>(inPackage.stepCount) << " | " << std::hex
        << std::setfill('0')
        << std::setw(2) << static_cast<unsigned short>(inPackage.crc);

    out.copyfmt(init);
}


// ----------------------------------------------------------------------

void WhbContext::printHexdump(const WhbUsbInPackage& inPackage, std::ostream& out)
{
    std::ios init(NULL);
    init.copyfmt(out);

    out << std::setfill('0') << std::hex << "0x" << std::setw(2) << static_cast<unsigned short>(inPackage.header) << " "
        << std::setw(2)
        << static_cast<unsigned short>(inPackage.randomByte) << " " << std::setw(2)
        << static_cast<unsigned short>(inPackage.buttonKeyCode1) << " " << std::setw(2)
        << static_cast<unsigned short>(inPackage.buttonKeyCode2) << " " << std::setw(2)
        << static_cast<unsigned short>(inPackage.rotaryButtonFeedKeyCode) << " " << std::setw(2)
        << static_cast<unsigned short>(inPackage.rotaryButtonAxisKeyCode) << " " << std::setw(2)
        << static_cast<unsigned short>(inPackage.stepCount & 0xff) << " " << std::setw(2)
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

        initWhb();
        if (!mUsb.init())
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
                std::cerr << endl << "failed to enable reception" << endl;
                return EXIT_FAILURE;
            }
            *mInitCout << " ok" << endl;
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
}

// ----------------------------------------------------------------------

bool WhbContext::enableReceiveAsyncTransfer()
{
    return mUsb.setupAsyncTransfer();
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
            struct timeval timeout;
            timeout.tv_sec  = 0;
            timeout.tv_usec = 200 * 1000;

            int r = libusb_handle_events_timeout_completed(getUsbContext(), &timeout, nullptr);
            assert((r == LIBUSB_SUCCESS) || (r == LIBUSB_ERROR_NO_DEVICE) || (r == LIBUSB_ERROR_BUSY) ||
                   (r == LIBUSB_ERROR_TIMEOUT) || (r == LIBUSB_ERROR_INTERRUPTED));
            if (mHal.isSimulationModeEnabled())
            {
                linuxcncSimulate();
            }
            updateDisplay();
        }
        updateDisplay();

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

void WhbContext::enableCrcDebugging(bool enable)
{
    mIsCrcDebuggingEnabled = enable;
}

// ----------------------------------------------------------------------

void WhbContext::offerHalMemory()
{
    assert(mHal.memory != nullptr);
    mUsb.takeHalMemoryReference(mHal.memory);
}
}
