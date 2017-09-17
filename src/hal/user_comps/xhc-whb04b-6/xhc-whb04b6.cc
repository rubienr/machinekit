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

    mPendant.update(keyCode, modifierCode,
                    inPackage.rotaryButtonAxisKeyCode,
                    inPackage.rotaryButtonFeedKeyCode,
                    inPackage.stepCount);

    /* deprecated
    //! update previous and current button state
    mPreviousButtonCodes = mCurrentButtonCodes;
    mCurrentButtonCodes.updateButtonState(keyCode, modifierCode, inPackage.rotaryButtonAxisKeyCode,
                                          inPackage.rotaryButtonFeedKeyCode);

    bool forceUpdate = updateHalButtons(inPackage, keyCode, modifierCode);
    updateAxisRotaryButton(inPackage);
    updateStepRotaryButton(inPackage, forceUpdate);
    updateJogDial(inPackage);
    onDataInterpreted();*/
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
    mPreviousButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                         mStepHandler.stepSize.continuous),
    mCurrentButtonCodes(mKeyCodes.buttons, mKeyCodes.axis, mKeyCodes.feed, mStepHandler.stepSize.step,
                        mStepHandler.stepSize.continuous),
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
    mMachineConfig(),
    mPendant()
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

void WhbContext::sendDisplayData()
{
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
        *(mHal.memory->out.isPendantRequired)  = mUsb.isWaitForPendantBeforeHalEnabled();

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
    // *(hal->stepsizeUp) = ((xhc->button_step != 0) && (currentButtonCodes.mCurrentButton1Code == xhc->button_step));

    if (*(mHal.memory->out.jogCount) != last_jog_counts)
    {
        /*
        int   delta_int = *(mHal.memory.jogCount) - last_jog_counts;
        float delta     = delta_int * *(mHal.memory.jogScale);
        if (*(mHal.memory.jogEnableX))
        {
            *(mHal.memory.xMachineCoordinate) += delta;
            *(mHal.memory.xWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableY))
        {
            *(mHal.memory.yMachineCoordinate) += delta;
            *(mHal.memory.yWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableZ))
        {
            *(mHal.memory.zMachineCoordinate) += delta;
            *(mHal.memory.zWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableA))
        {
            *(mHal.memory.aMachineCoordinate) += delta;
            *(mHal.memory.aWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableB))
        {
            *(mHal.memory.bMachineCoordinate) += delta;
            *(mHal.memory.bWorkpieceCoordinate) += delta;
        }

        if (*(mHal.memory.jogEnableC))
        {
            *(mHal.memory.cMachineCoordinate) += delta;
            *(mHal.memory.cWorkpieceCoordinate) += delta;
        }*/

        //if (*(hal->jog_enable_spindle)) {
        //*(hal->spindle_override) += delta_int * 0.01;
        //if (*(hal->spindle_override) > 1) *(hal->spindle_override) = 1;
        //if (*(hal->spindle_override) < 0) *(hal->spindle_override) = 0;
        //*(hal->spindle_rps) = 25000.0/60.0 * *(hal->spindle_override);
        //}

        //if (*(hal->jog_enable_feedrate)) {
        //*(hal->feedrate_override) += delta_int * 0.01;
        //if (*(hal->feedrate_override) > 1) *(hal->feedrate_override) = 1;
        //if (*(hal->feedrate_override) < 0) *(hal->feedrate_override) = 0;
        //*(hal->feedrate) = 3000.0/60.0 * *(hal->feedrate_override);
        //}

        last_jog_counts = *(mHal.memory->out.jogCount);
    }
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
            struct timeval tv;
            tv.tv_sec  = 4;
            tv.tv_usec = 0;

            int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
            assert((r == LIBUSB_SUCCESS) || (r == LIBUSB_ERROR_NO_DEVICE) || (r == LIBUSB_ERROR_BUSY) ||
                   (r == LIBUSB_ERROR_TIMEOUT) || (r == LIBUSB_ERROR_INTERRUPTED));
            //mHal.computeVelocity();
            if (mHal.isSimulationModeEnabled())
            {
                linuxcncSimulate();
            }
            sendDisplayData();
        }

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

size_t WhbContext::getSoftwareButtonIndex(uint8_t keyCode) const
{
    int      buttonsCount = sizeof(mSoftwareButtons) / sizeof(WhbSoftwareButton);
    for (int idx          = 0; idx < buttonsCount; idx++)

        if (mSoftwareButtons[idx].key.code == keyCode)
        {
            return idx;
        }
    assert (false);
    return 0;
}

// ----------------------------------------------------------------------

bool WhbContext::dispatchButtonEventToHal(const WhbSoftwareButton& softwareButton, bool isButtonPressed)
{
    const WhbButtonsCode& buttonCodes = mKeyCodes.buttons;

    // reset button
    if (softwareButton.containsKeys(buttonCodes.reset, buttonCodes.undefined))
    {
        mHal.setReset(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // stop button
    else if (softwareButton.containsKeys(buttonCodes.stop, buttonCodes.undefined))
    {
        mHal.setStop(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // start pause button
    else if (softwareButton.containsKeys(buttonCodes.start, buttonCodes.undefined))
    {
        mHal.setStart(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // feed + button
    else if (softwareButton.containsKeys(buttonCodes.feed_plus, buttonCodes.undefined))
    {
        mHal.setFeedPlus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // feed - button
    else if (softwareButton.containsKeys(buttonCodes.feed_minus, buttonCodes.undefined))
    {
        mHal.setFeedMinus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle + button
    else if (softwareButton.containsKeys(buttonCodes.spindle_plus, buttonCodes.undefined))
    {
        mHal.setSpindlePlus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle - button
    else if (softwareButton.containsKeys(buttonCodes.spindle_minus, buttonCodes.undefined))
    {
        mHal.setSpindleMinus(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // machine home button
    else if (softwareButton.containsKeys(buttonCodes.machine_home, buttonCodes.undefined))
    {
        mHal.setMachineHome(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // safe-z button
    else if (softwareButton.containsKeys(buttonCodes.safe_z, buttonCodes.undefined))
    {
        mHal.setSafeZ(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // workpiece home button
    else if (softwareButton.containsKeys(buttonCodes.workpiece_home, buttonCodes.undefined))
    {
        mHal.setWorkpieceHome(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // spindle on/off button
    else if (softwareButton.containsKeys(buttonCodes.spindle_on_off, buttonCodes.undefined))
    {
        mHal.setSpindleOn(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // function - button
    else if (softwareButton.containsKeys(buttonCodes.function, buttonCodes.undefined))
    {
        mHal.setFunction(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // probe-z button
    else if (softwareButton.containsKeys(buttonCodes.probe_z, buttonCodes.undefined))
    {
        mHal.setProbeZ(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 10 button
    else if (softwareButton.containsKeys(buttonCodes.macro10, buttonCodes.undefined))
    {
        mHal.setMacro10(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 11 button
    else if (softwareButton.containsKeys(buttonCodes.reset, buttonCodes.function))
    {
        mHal.setMacro11(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 12 button
    else if (softwareButton.containsKeys(buttonCodes.stop, buttonCodes.function))
    {
        mHal.setMacro12(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 13 button
    else if (softwareButton.containsKeys(buttonCodes.start, buttonCodes.function))
    {
        mHal.setMacro13(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 1 button
    else if (softwareButton.containsKeys(buttonCodes.feed_plus, buttonCodes.function))
    {
        mHal.setMacro1(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 2 button
    else if (softwareButton.containsKeys(buttonCodes.feed_minus, buttonCodes.function))
    {
        mHal.setMacro2(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 3 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_plus, buttonCodes.function))
    {
        mHal.setMacro3(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 4 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_minus, buttonCodes.function))
    {
        mHal.setMacro4(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 5 button
    else if (softwareButton.containsKeys(buttonCodes.machine_home, buttonCodes.function))
    {
        mHal.setMacro5(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 6 button
    else if (softwareButton.containsKeys(buttonCodes.safe_z, buttonCodes.function))
    {
        mHal.setMacro6(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 7 button
    else if (softwareButton.containsKeys(buttonCodes.workpiece_home, buttonCodes.function))
    {
        mHal.setMacro7(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 8 button
    else if (softwareButton.containsKeys(buttonCodes.spindle_on_off, buttonCodes.function))
    {
        mHal.setMacro8(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 9 button
    else if (softwareButton.containsKeys(buttonCodes.probe_z, buttonCodes.function))
    {
        mHal.setMacro9(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 14 button
    else if (softwareButton.containsKeys(buttonCodes.macro10, buttonCodes.function))
    {
        mHal.setMacro14(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 15 button
    else if (softwareButton.containsKeys(buttonCodes.manual_pulse_generator, buttonCodes.function))
    {
        mHal.setMacro15(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }
        // macro 16 button
    else if (softwareButton.containsKeys(buttonCodes.step_continuous, buttonCodes.function))
    {
        mHal.setMacro16(isButtonPressed, getHalPinNumber(softwareButton));
        return true;
    }

    return false;
}
// ----------------------------------------------------------------------


bool WhbContext::onButtonPressedEvent(const WhbSoftwareButton& softwareButton)
{
    bool isUpdateRecommended = false;

    printPushButtonText(softwareButton.key.code, softwareButton.modifier.code, *mKeyEventCout);
    *mKeyEventCout << endl;
    *mKeyEventCout << "event pressed ";
    if (!dispatchButtonEventToHal(softwareButton, true))
    {
        const WhbButtonsCode& buttonCodes = mKeyCodes.buttons;
        // TODO: implement validation if mpg/step can be applied on the current feed rotary button state
        // continuous button (MPG) and valid velocity rotary button position xxx
        if (softwareButton.containsKeys(buttonCodes.manual_pulse_generator, buttonCodes.undefined))
        {
            // obsolete
            //mCurrentButtonCodes.setCurrentModeContinuousMode();
            isUpdateRecommended = true;
        }
            // step button and valid step rotary button position
        else if (softwareButton.containsKeys(buttonCodes.step_continuous, buttonCodes.undefined))
        {
            // obsolete
            //mCurrentButtonCodes.setCurrentModeStepMode();
            isUpdateRecommended = true;
        }
    }

    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

bool WhbContext::onButtonReleasedEvent(const WhbSoftwareButton& softwareButton)
{
    bool isUpdateRecommended = false;
    *mKeyEventCout << "event released ";
    printPushButtonText(softwareButton.key.code, softwareButton.modifier.code, *mKeyEventCout);
    *mKeyEventCout << endl;

    dispatchButtonEventToHal(softwareButton, false);

    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

void WhbContext::onAxisActiveEvent(const WhbKeyCode& axis)
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event axis active   " << std::setw(5) << axis.text << " (" << std::setw(4) << axis.altText << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);

    dispatchAxisEventToHal(axis, true);
}

// ----------------------------------------------------------------------

void WhbContext::onAxisInactiveEvent(const WhbKeyCode& axis)
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event axis inactive " << std::setw(5) << axis.text << " (" << std::setw(4) << axis.altText << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);

    dispatchAxisEventToHal(axis, false);
}

// ----------------------------------------------------------------------

void WhbContext::onDataInterpreted()
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event data interpreted, display data ready" << endl;
    // mIsDisplayDataReady = true;
    mKeyEventCout->copyfmt(init);
}

// ----------------------------------------------------------------------

//! update axis rotary button's state to hal and detect active/inactive event
void WhbContext::updateAxisRotaryButton(const WhbUsbInPackage& inPackage)
{
    uint8_t newAxisKeyCode = inPackage.rotaryButtonAxisKeyCode;
    const WhbKeyCode& currentAxisCode = mCurrentButtonCodes.getAxisCode();
    if (currentAxisCode.code != newAxisKeyCode)
    {
        const WhbKeyCode* newAxisCode = nullptr;
        keyEventReceiver.onAxisInactiveEvent(currentAxisCode);
        if (newAxisKeyCode == mKeyCodes.axis.off.code)
        {
            newAxisCode = &mKeyCodes.axis.off;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.x.code)
        {
            newAxisCode = &mKeyCodes.axis.x;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.y.code)
        {
            newAxisCode = &mKeyCodes.axis.y;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.z.code)
        {
            newAxisCode = &mKeyCodes.axis.z;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.a.code)
        {
            newAxisCode = &mKeyCodes.axis.a;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.b.code)
        {
            newAxisCode = &mKeyCodes.axis.b;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.c.code)
        {
            newAxisCode = &mKeyCodes.axis.c;
        }
        else if (newAxisKeyCode == mKeyCodes.axis.undefined.code)
        {
            newAxisCode = &mKeyCodes.axis.undefined;
        }
        else
        {
            *mRxCout << "received unexpected axis id " << static_cast<unsigned short>(newAxisKeyCode) << endl;
            assert(false);
        }
        mCurrentButtonCodes.setAxisCode(*newAxisCode);
        keyEventReceiver.onAxisActiveEvent(*newAxisCode);
    }
}

// ----------------------------------------------------------------------

bool WhbContext::updateHalButtons(const WhbUsbInPackage& inPackage, uint8_t keyCode, uint8_t modifierCode)
{
    bool isUpdateRecommended = false;

    for (int idx = 0; !((mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].key.code) &&
                        (mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].modifier.code)); idx++)
    {
        const WhbSoftwareButton& softwareButton = mSoftwareButtons[idx];
        hal_bit_t halButtonState             = *(mHal.memory->out.button_pin[idx]);
        uint8_t   softwareButtonKeyCode      = softwareButton.key.code;
        uint8_t   softwareButtonModifierCode = softwareButton.modifier.code;

        if ((halButtonState) && // on last button state was pressed
            !((softwareButtonKeyCode == keyCode) && // and current state is not pressed
              (softwareButtonModifierCode == modifierCode)))
        {
            // on button released event
            *(mHal.memory->out.button_pin[idx]) = false;
            keyEventReceiver.onButtonReleasedEvent(softwareButton);
        }
        else if ((!halButtonState) &&  // on last button state was unpressed
                 ((softwareButtonKeyCode == keyCode) && // and current state is pressed
                  (softwareButtonModifierCode == modifierCode)))
        {
            // on button pressed event
            *(mHal.memory->out.button_pin[idx]) = true;
            mCurrentButtonCodes.updateButtonState(softwareButton);
            isUpdateRecommended = keyEventReceiver.onButtonPressedEvent(softwareButton);
        }
    }
    return isUpdateRecommended;
}

// ----------------------------------------------------------------------

//! update axis rotary button's state to hal and detect active/inactive event
void WhbContext::updateStepRotaryButton(const WhbUsbInPackage& inPackage, bool forceEvents)
{
    uint8_t newStepKeyCode = inPackage.rotaryButtonFeedKeyCode;
    const WhbKeyCode& currentStepCode = mCurrentButtonCodes.getFeedCode();

    if ((currentStepCode.code != newStepKeyCode) || forceEvents)
    {
        keyEventReceiver.onFeedInactiveEvent(currentStepCode);
        const WhbKeyCode* newFeedCode = nullptr;

        if (newStepKeyCode == mKeyCodes.feed.speed_0_001.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(
                    WhbHandwheelStepModeStepSize::PositionNameIndex::RotaryButton0001);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton2percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_001;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_0_01.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(
                    WhbHandwheelStepModeStepSize::PositionNameIndex::RotaryButton0010);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton5percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_01;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_0_1.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(
                    WhbHandwheelStepModeStepSize::PositionNameIndex::RotaryButton0100);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton10percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_0_1;
        }
        else if (newStepKeyCode == mKeyCodes.feed.speed_1.code)
        {
            if (mCurrentButtonCodes.isCurrentModeStepMode())
            {
                mCurrentButtonCodes.setCurrentStepModeStepSize(
                    WhbHandwheelStepModeStepSize::PositionNameIndex::RotaryButton100);
            }
            else
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton30percent);
            }
            newFeedCode = &mKeyCodes.feed.speed_1;
        }
        else if (newStepKeyCode == mKeyCodes.feed.percent_60.code)
        {
            if (mCurrentButtonCodes.isCurrentModeContinuousMode())
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton60percent);
                newFeedCode = &mKeyCodes.feed.percent_60;
            }
        }
        else if (newStepKeyCode == mKeyCodes.feed.percent_100.code)
        {
            if (mCurrentButtonCodes.isCurrentModeContinuousMode())
            {
                mCurrentButtonCodes.setCurrentContinuousModeStepSize(
                    WhbHandwheelContinuousModeStepSize::PositionNameIndex::RotaryButton100percent);
                newFeedCode = &mKeyCodes.feed.percent_100;
            }
        }
        else if (newStepKeyCode == mKeyCodes.feed.lead.code)
        {
            newFeedCode = &mKeyCodes.feed.lead;
            mHal.setLead();
        }
        else
        {
            // \p newFeedCode value may be unexpected i.e. on device power off
        }

        //! feed rotary button position changed but no valid feed step size available for this mode
        if (newFeedCode != nullptr)
        {
            if ((*newFeedCode) != mKeyCodes.feed.lead)
            {
                mHal.setJogWheelStepMode(mCurrentButtonCodes.currentStepMode());
                mHal.setStepSize(mCurrentButtonCodes.getStepSize());
            }
            mCurrentButtonCodes.setFeedCode(*newFeedCode);
            keyEventReceiver.onFeedActiveEvent(*newFeedCode);
        }
    }
}

// ----------------------------------------------------------------------

void WhbContext::onFeedActiveEvent(const WhbKeyCode& axis)
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event feed active  " << std::setfill(' ') << std::setw(5) << axis.text << "(" << std::setw(4)
                   << axis.altText
                   << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);
}

// ----------------------------------------------------------------------

void WhbContext::onFeedInactiveEvent(const WhbKeyCode& axis)
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event feed inactive  " << std::setfill(' ') << std::setw(5) << axis.text << "(" << std::setw(4)
                   << axis.altText
                   << ")"
                   << endl;
    mKeyEventCout->copyfmt(init);
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

void WhbContext::onJogDialEvent(int8_t delta)
{
    std::ios init(NULL);
    init.copyfmt(*mKeyEventCout);
    *mKeyEventCout << "event jog dial " << std::setfill(' ') << std::setw(3) << static_cast<signed short>(delta)
                   << endl;
    mKeyEventCout->copyfmt(init);
    mHal.newJogDialDelta(delta);
}

// ----------------------------------------------------------------------

void WhbContext::updateJogDial(const WhbUsbInPackage& inPackage)
{
    if (inPackage.stepCount != 0)
    {
        onJogDialEvent(inPackage.stepCount);
    }
}

// ----------------------------------------------------------------------

void WhbContext::enableCrcDebugging(bool enable)
{
    mIsCrcDebuggingEnabled = enable;
}

// ----------------------------------------------------------------------

size_t WhbContext::getHalPinNumber(const WhbSoftwareButton& button)
{
    for (size_t idx = 0; !((mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].key.code) &&
                           (mKeyCodes.buttons.undefined.code == mSoftwareButtons[idx].modifier.code)); idx++)
    {
        if ((mSoftwareButtons[idx].key.code == button.key.code) &&
            (mSoftwareButtons[idx].modifier.code == button.modifier.code))
        {
            return idx;
        }
    }
    assert(false);
    return 0;
}

// ----------------------------------------------------------------------

void WhbContext::offerHalMemory()
{
    assert(mHal.memory != nullptr);
    mUsb.takeHalMemoryReference(mHal.memory);
}

// ----------------------------------------------------------------------

void WhbContext::dispatchAxisEventToHal(const WhbKeyCode& axis, bool isActive)
{
    if (axis.code == mKeyCodes.axis.off.code)
    {
        mHal.setNoAxisActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.x.code)
    {
        mHal.setAxisXActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.y.code)
    {
        mHal.setAxisYActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.z.code)
    {
        mHal.setAxisZActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.a.code)
    {
        mHal.setAxisAActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.b.code)
    {
        mHal.setAxisBActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.c.code)
    {
        mHal.setAxisCActive(isActive);
    }
    else if (axis.code == mKeyCodes.axis.undefined.code)
    {
        mHal.setNoAxisActive(isActive);
    }
}

void WhbContext::setMachineConfig(const MachineConfiguration& machineConfig)
{
    *mInitCout << "init  setting machine configuration to scale="
               << machineConfig.getScale() << " "
               << "max_velocity=" << machineConfig.getMaxVelocity() << endl;
    mMachineConfig = machineConfig;
}

// ----------------------------------------------------------------------

float MachineConfiguration::getScale() const
{
    return mScale;
}

// ----------------------------------------------------------------------

float MachineConfiguration::getMaxVelocity() const
{
    return mMaxVelocity;
}

// ----------------------------------------------------------------------

MachineConfiguration::MachineConfiguration(float scale, float maxVelocity) :
    mScale(scale),
    mMaxVelocity(maxVelocity)
{
}

// ----------------------------------------------------------------------

void MachineConfiguration::setScale(float scale)
{
    mScale = scale;
}

// ----------------------------------------------------------------------

void MachineConfiguration::setMaxVelocity(float maxVelocity)
{
    mMaxVelocity = maxVelocity;
}

// ----------------------------------------------------------------------

MachineConfiguration& MachineConfiguration::operator=(const MachineConfiguration other)
{
    mScale       = other.mScale;
    mMaxVelocity = other.mMaxVelocity;
    return *this;
}
}
