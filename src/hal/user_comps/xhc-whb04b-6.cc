/*
   XHC-HB04 Wireless MPG pendant LinuxCNC HAL module for LinuxCNC

   Copyright (C) 2013 Frederic Rible (frible@teaser.fr)
   Copyright (C) 2013 Rene Hopf (renehopf@mac.com)
   Copyright (C) 2014 Marius Alksnys (marius.alksnys@gmail.com)
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

typedef struct {
    libusb_context *context;
    libusb_device_handle *deviceHandle;
} XhbUsb;

typedef struct {
    bool dropNextInPackage;
} WhbSleepDetect;

typedef struct {
    typedef enum { NA1=0, NA2=1, Key1=2, Key2=3, Feed=4, Axis=5, Step=6, CRC=7 } ByteType;

    const ByteType positionToType[8];
    const uint8_t expectedSize;
} WhbInPackageInfo;


typedef struct {
    const WhbInPackageInfo in;
} WhbPackageInfo;


typedef struct {
    const uint8_t code;
    const char* text;
    const char* altText;
} WhbKeyCode;

typedef struct WhbSoftwareButton{
    const WhbKeyCode* key;
    const WhbKeyCode* modifier;
} WhbSoftwareButton;

typedef struct {
    typedef enum { Off=0, X=1, Y=2, Z=3, A=4, B=5, C=6 } AxisIndexName;

    const WhbKeyCode off;
	const WhbKeyCode x;
	const WhbKeyCode y;
	const WhbKeyCode z;
	const WhbKeyCode a;
	const WhbKeyCode b;
	const WhbKeyCode c;
    const WhbKeyCode undefined;
} WhbAxisButtonCodes;

typedef struct {
    const WhbKeyCode speed_0_001;
    const WhbKeyCode speed_0_01;
    const WhbKeyCode speed_0_1;
    const WhbKeyCode speed_1;
    const WhbKeyCode percent_60;
    const WhbKeyCode percent_100;
    const WhbKeyCode lead;
    const WhbKeyCode undefined;
} WhbFeedButtonCodes;

typedef struct {
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

} WhbButtonCodes;

typedef struct {
	const char* name;
	const char* configSectionName;
	const uint16_t usbVendorId;
	const uint16_t usbProductId;
} WhbEntity;

typedef struct {
    WhbButtonCodes buttons;
	WhbAxisButtonCodes axis;
	WhbFeedButtonCodes feed;
} WhbKeyCodes;

typedef struct {
	const WhbEntity entity;
    const WhbKeyCodes codes;
    const WhbPackageInfo packageInfo;
    WhbSleepDetect sleepState;
} WhbContext;


WhbContext Whb = {
	.entity = {
		.name = "xhc-whb04b-6",
		.configSectionName = "XHC-WHB04B-6",
		.usbVendorId = 0x10ce,
		.usbProductId = 0xeb93
	},
	.codes = {
        .buttons = {
            .reset                  = {.code=0x01, .text="RESET", .altText= "Macro-11"},
            .stop                   = {.code=0x02, .text="STOP", .altText= "Macro-12"},
            .start                  = {.code=0x03, .text="Start", .altText= "Pause"},
            .feed_plus              = {.code=0x04, .text="Feed+", .altText= "Macro-1"},
            .feed_minus             = {.code=0x05, .text="Feed-", .altText= "Macro-2"},
            .spindle_plus           = {.code=0x06, .text="Spindle+", .altText= "Macro-3"},
            .spindle_minus          = {.code=0x07, .text="Spindle-", .altText= "Macro-4"},
            .machine_home           = {.code=0x08, .text="M-HOME", .altText= "Macro-5"},
            .safe_z                 = {.code=0x09, .text="Safe-Z", .altText= "Macro-6"},
            .workpiece_home         = {.code=0x0a, .text="W-HOME", .altText= "Macro-7"},
            .spindle_on_off         = {.code=0x0b, .text="S-ON/OFF", .altText= "Macro-8"},
            .function               = {.code=0x0c, .text="Fn", .altText= ""},
            .probe_z                = {.code=0x0d, .text="Probe-Z", .altText= "Macro-9"},
            .macro10                = {.code=0x10, .text="Macro-10", .altText= "Macro-13"},
            .manual_pulse_generator = {.code=0x0e, .text="MPG", .altText= "Macro-14"},
            .step_continuous        = {.code=0x0f, .text="STEP", .altText= "Continuous"},
            .undefined              = {.code=0x00, .text="", .altText= ""}
        },
		.axis = {
			.off = {.code=0x06, .text="OFF", .altText= ""},
			.x   = {.code=0x11, .text="X", .altText= ""},
			.y   = {.code=0x12, .text="Y", .altText= ""},
			.z   = {.code=0x13, .text="Z", .altText= ""},
			.a   = {.code=0x14, .text="A", .altText= ""},
			.b   = {.code=0x15, .text="B", .altText= ""},
			.c   = {.code=0x16, .text="C", .altText= ""},
            .undefined = {.code=0x00, .text="", .altText= ""}
		},
		.feed = {
			.speed_0_001 = {.code=0x0d, .text="0.001", .altText= "2%"},
			.speed_0_01  = {.code=0x0e, .text="0.01", .altText= "5%"},
			.speed_0_1   = {.code=0x0f, .text="0.1", .altText= "10%"},
			.speed_1     = {.code=0x10, .text="1", .altText= "30%"},
			.percent_60  = {.code=0x1a, .text="", .altText= "60%"},
			.percent_100 = {.code=0x1b, .text="", .altText= "100%"},
			.lead        = {.code=0x1c, .text="Lead", .altText= ""},
            .undefined   = {.code=0x00, .text="", .altText= ""}
		},
	},
    .packageInfo = {
        .in = {
            .positionToType = {
                WhbInPackageInfo::ByteType::NA1,
                WhbInPackageInfo::ByteType::NA2,
                WhbInPackageInfo::ByteType::Key1,
                WhbInPackageInfo::ByteType::Key2,
                WhbInPackageInfo::ByteType::Feed,
                WhbInPackageInfo::ByteType::Axis,
                WhbInPackageInfo::ByteType::Step,
                WhbInPackageInfo::ByteType::CRC
            },
            .expectedSize = (sizeof(WhbInPackageInfo::positionToType)/sizeof(WhbInPackageInfo::ByteType))
        }
    },
    .sleepState = {.dropNextInPackage = false }
};


int hal_comp_id;
bool simu_mode = true;

typedef struct {
	char pin_name[256];
	unsigned int code;
} xhc_button_t;


#define NB_MAX_BUTTONS 64

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

// alternate stepsize sequences (use STEPSIZE_DISPLAY_*), terminate with 0:
static const int  stepsize_sequence_1[] = {1,10,100,1000,0}; // default
static const int  stepsize_sequence_2[] = {1,5,10,20,0};
static const int* stepsize_sequence = stepsize_sequence_1; // use the default
static int stepsize_idx = 0; // start at initial (zeroth) sequence

typedef struct {
	hal_float_t *x_wc, *y_wc, *z_wc, *a_wc, *b_wc, *c_wc;
	hal_float_t *x_mc, *y_mc, *z_mc, *a_mc, *b_mc, *c_mc;

	hal_float_t *feedrate_override, *feedrate;
	hal_float_t *spindle_override, *spindle_rps;

	hal_bit_t *button_pin[NB_MAX_BUTTONS];

    hal_bit_t *jog_enable_off;
	hal_bit_t *jog_enable_x;
	hal_bit_t *jog_enable_y;
	hal_bit_t *jog_enable_z;
	hal_bit_t *jog_enable_a;
	hal_bit_t *jog_enable_b;
	hal_bit_t *jog_enable_c;
	//hal_bit_t *jog_enable_feedrate;
	//hal_bit_t *jog_enable_spindle;
	hal_float_t *jog_scale;
	hal_s32_t *jog_counts, *jog_counts_neg;

	hal_float_t *jog_velocity;
	hal_float_t *jog_max_velocity;
	hal_float_t *jog_increment;
	hal_bit_t *jog_plus_x, *jog_plus_y, *jog_plus_z, *jog_plus_a, *jog_plus_b, *jog_plus_c;
	hal_bit_t *jog_minus_x, *jog_minus_y, *jog_minus_z, *jog_minus_a, *jog_minus_b, *jog_minus_c;

	hal_bit_t *stepsize_up;
	hal_s32_t *stepsize;
	hal_bit_t *sleeping;
	hal_bit_t *connected;
	hal_bit_t *require_pendant;
} xhc_hal_t;

typedef struct {
    void* refs[256];
    uint16_t nextIndex;
} XhcCleanupRef;

typedef struct {
	xhc_hal_t *hal;
    uint8_t currentAxisCode;
    WhbSoftwareButton button[31];
    unsigned char button_code;

	unsigned char old_inc_step_status;
    //! used in simulation mode to handle the STEP increment
    unsigned char button_step;

    //! velocity computation
	hal_s32_t last_jog_counts;

    //! cleanup refernces
    XhcCleanupRef cleanup;

	struct timeval last_tv;
    struct timeval last_wakeup;
} xhc_t;

static xhc_t xhc =
{
    .hal = nullptr,
    .currentAxisCode = Whb.codes.axis.undefined.code,
    .button = {
            { .key = &Whb.codes.buttons.reset,          .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.reset,          .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.stop,           .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.stop,           .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.start,          .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.start,          .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.feed_plus,      .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.feed_plus,      .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.feed_minus,     .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.feed_minus,     .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.spindle_plus,   .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.spindle_plus,   .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.spindle_minus,  .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.spindle_minus,  .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.machine_home,   .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.machine_home,   .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.safe_z,         .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.safe_z,         .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.workpiece_home, .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.workpiece_home, .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.spindle_on_off, .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.spindle_on_off, .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.function,       .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.probe_z,        .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.probe_z,        .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.macro10,        .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.macro10,        .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.manual_pulse_generator, .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.manual_pulse_generator, .modifier = &Whb.codes.buttons.function},
            { .key = &Whb.codes.buttons.step_continuous, .modifier = &Whb.codes.buttons.undefined},
            { .key = &Whb.codes.buttons.step_continuous, .modifier = &Whb.codes.buttons.function}
    },
    .button_code = 0,
    .old_inc_step_status = 0,
    .button_step = 0,
    .last_jog_counts = -1,
    .cleanup = {.refs= {0}, .nextIndex = 0}
};

static bool do_exit = false;
static bool do_reconnect = false;
static bool wait_for_pendant_before_HAL = false;

unsigned char in_buf[32];
void cb_transfer_in(struct libusb_transfer *transfer);
void cb_response_in(struct libusb_transfer *transfer);

extern "C" const char *
iniFind(FILE *fp, const char *tag, const char *configSecitonName)
{
    IniFile                     f(false, fp);

    return(f.Find(tag, configSecitonName));
}

void init_xhc(xhc_t *xhc)
{
	//memset(xhc, 0, sizeof(*xhc));
	xhc->old_inc_step_status = -1;
	gettimeofday(&xhc->last_wakeup, nullptr);
}

int xhc_encode_float(float v, unsigned char *buf)
{
	unsigned int int_v = (int)rtapi_rint(rtapi_fabs(v) * 10000.0);
	unsigned short int_part = int_v / 10000;
	unsigned short fract_part = int_v % 10000;
	if (v < 0) fract_part = fract_part | 0x8000;
	*(short *)buf = int_part;
	*((short *)buf+1) = fract_part;
	return 4;
}

int xhc_encode_s16(int v, unsigned char *buf)
{
	*(short *)buf = v;
	return 2;
}

void xhc_display_encode(xhc_t *xhc, unsigned char *data, int len)
{
	unsigned char buf[6*7];
	unsigned char *p = buf;
	int i;
	int packet;

	assert(len == 6*8);

	memset(buf, 0, sizeof(buf));

	*p++ = 0xFE;
	*p++ = 0xFD;
	*p++ = 0x0C;

	if (xhc->currentAxisCode == Whb.codes.axis.a.code) p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->a_wc)) / 1000, p);
	else p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->x_wc)) / 1000, p);
	p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->y_wc)) / 1000, p);
	p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->z_wc)) / 1000, p);
	if (xhc->currentAxisCode == Whb.codes.axis.a.code) p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->a_mc)) / 1000, p);
	else p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->x_mc)) / 1000, p);
	p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->y_mc)) / 1000, p);
	p += xhc_encode_float(rtapi_rint(1000 * *(xhc->hal->z_mc)) / 1000, p);
	p += xhc_encode_s16((int)rtapi_rint(100.0 * *(xhc->hal->feedrate_override)), p);
	p += xhc_encode_s16((int)rtapi_rint(100.0 * *(xhc->hal->spindle_override)), p);
	p += xhc_encode_s16((int)rtapi_rint(60.0 * *(xhc->hal->feedrate)), p);
	p += xhc_encode_s16((int)rtapi_rint(60.0 * *(xhc->hal->spindle_rps)), p);

	switch (*(xhc->hal->stepsize)) {
	case    0: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0; break;
	case    1: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1; break;
	case    5: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_5; break;
	case   10: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_10; break;
	case   20: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_20; break;
	case   30: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_30; break;
	case   40: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_40; break;
	case   50: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_50; break;
	case  100: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_100; break;
	case  500: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_500; break;
	case 1000: buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_1000; break;
	default:   //stepsize not supported on the display:
			   buf[STEPSIZE_BYTE] = STEPSIZE_DISPLAY_0; break;
	}

	// Multiplex to 6 USB transactions

	p = buf;
	for (packet=0; packet<6; packet++) {
		for (i=0; i<8; i++) {
			if (i == 0) data[i+8*packet] = 6;
			else data[i+8*packet] = *p++;
		}
	}
}

void xhc_set_display(libusb_device_handle *dev_handle, xhc_t *xhc)
{
	unsigned char data[6*8];
	int packet;

	xhc_display_encode(xhc, data, sizeof(data));

	for (packet=0; packet<6; packet++) {
		int r = libusb_control_transfer(dev_handle,
		              LIBUSB_DT_HID, //bmRequestType 0x21
		              LIBUSB_REQUEST_SET_CONFIGURATION, //bRequest 0x09
		              0x0306,         //wValue
		              0x00,           //wIndex
		              data+8*packet,  //*data
		              8,              //wLength
		              0);             //timeout
		if (r < 0) {
			do_reconnect = true;
		}
	}
}

void printPushButtonText(uint8_t keyCode, uint8_t modifierCode)
{
    uint8_t indent = 10;
    const WhbKeyCode* keyCodeBase = (WhbKeyCode*)&Whb.codes.buttons;
    // no key
    if (keyCode == Whb.codes.buttons.undefined.code)
    {
        if (modifierCode == Whb.codes.buttons.function.code) {
            printf("%*s", indent, Whb.codes.buttons.undefined.altText);
        } else {
            printf("%*s", indent, Whb.codes.buttons.undefined.text);
        }
        return;
    }

    // key is modifier key itself
    if (keyCode == Whb.codes.buttons.function.code)
    {
        printf("%*s", indent, Whb.codes.buttons.function.text);
        return;
    }

    // find key code
    const WhbKeyCode* whbKeyCode = keyCodeBase;
    while (whbKeyCode->code != 0) {
        if (whbKeyCode->code == keyCode) {
            break;
        }
        whbKeyCode++;
    }

    if (modifierCode == Whb.codes.buttons.function.code) {
        printf("%*s", indent,whbKeyCode->altText);
    } else {
        printf("%*s", indent,whbKeyCode->text);
    }
}

void printTwistButtonText(const WhbKeyCode* keyCodeBase, uint8_t keyCode) {
    // find key code
    const WhbKeyCode* whbKeyCode = keyCodeBase;
    while (whbKeyCode->code != 0) {
        if (whbKeyCode->code == keyCode) {
            break;
        }
        whbKeyCode++;
    }
    printf("%*s (%*s)", 5, whbKeyCode->text, 4, whbKeyCode->altText);
}


void printData(const unsigned char* data, int length)
{
    if (length != Whb.packageInfo.in.expectedSize) return;

    for (uint8_t idx = 0; idx < Whb.packageInfo.in.expectedSize; idx++) {
        WhbInPackageInfo::ByteType dataType = Whb.packageInfo.in.positionToType[idx];
        switch (dataType)
        {
            case WhbInPackageInfo::ByteType::NA1:
                printf("| %02X | ", data[idx]);
                break;
            case WhbInPackageInfo::ByteType::NA2:
                printf("%02X | ", data[idx]);
                break;
            case WhbInPackageInfo::ByteType::Key1:
                printPushButtonText((uint8_t)data[idx], (uint8_t)data[idx+1]);
                printf(" | ");
                break;
            case WhbInPackageInfo::ByteType::Key2:
                printPushButtonText((uint8_t)data[idx], (uint8_t)data[idx-1]);
                printf(" | ");
                break;
            case WhbInPackageInfo::ByteType::Feed:
                printTwistButtonText((WhbKeyCode*)&Whb.codes.feed, (uint8_t)data[idx]);
                printf(" | ");
                break;
            case WhbInPackageInfo::ByteType::Axis:
                printTwistButtonText((WhbKeyCode*)&Whb.codes.axis, (uint8_t)data[idx]);
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

void hexdump(unsigned char *data, int len)
{
	int i;

	for (i=0; i<len; i++) printf("%02X ", data[i]);
}

void linuxcnc_simu(xhc_t *xhc)
{
	static int last_jog_counts = 0;
	xhc_hal_t *hal = xhc->hal;

	*(hal->stepsize_up) = (xhc->button_step && xhc->button_code == xhc->button_step);

	if (*(hal->jog_counts) != last_jog_counts) {
		int delta_int = *(hal->jog_counts) - last_jog_counts;
		float delta = delta_int * *(hal->jog_scale);
		if (*(hal->jog_enable_x)) {
			*(hal->x_mc) += delta;
			*(hal->x_wc) += delta;
		}

		if (*(hal->jog_enable_y)) {
			*(hal->y_mc) += delta;
			*(hal->y_wc) += delta;
		}

		if (*(hal->jog_enable_z)) {
			*(hal->z_mc) += delta;
			*(hal->z_wc) += delta;
		}

        if (*(hal->jog_enable_a)) {
            *(hal->a_mc) += delta;
            *(hal->a_wc) += delta;
        }

        if (*(hal->jog_enable_b)) {
            *(hal->b_mc) += delta;
            *(hal->b_wc) += delta;
        }

        if (*(hal->jog_enable_c)) {
            *(hal->c_mc) += delta;
            *(hal->c_wc) += delta;
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

		last_jog_counts = *(hal->jog_counts);
	}
}

void compute_velocity(xhc_t *xhc)
{
	timeval now, delta_tv;
	gettimeofday(&now, nullptr);

	if (xhc->last_tv.tv_sec == 0) xhc->last_tv = now;
	timersub(&now, &xhc->last_tv, &delta_tv);
	float elapsed = delta_tv.tv_sec + 1e-6f*delta_tv.tv_usec;
	if (elapsed <= 0) return;

	float delta_pos = (*(xhc->hal->jog_counts) - xhc->last_jog_counts) * *(xhc->hal->jog_scale);
	float velocity = *(xhc->hal->jog_max_velocity) * 60.0f * *(xhc->hal->jog_scale);
	float k = 0.05f;

	if (delta_pos) {
		*(xhc->hal->jog_velocity) = (1 - k) * *(xhc->hal->jog_velocity) + k * velocity;
		*(xhc->hal->jog_increment) = rtapi_fabs(delta_pos);
		*(xhc->hal->jog_plus_x) = (delta_pos > 0) && *(xhc->hal->jog_enable_x);
		*(xhc->hal->jog_minus_x) = (delta_pos < 0) && *(xhc->hal->jog_enable_x);
		*(xhc->hal->jog_plus_y) = (delta_pos > 0) && *(xhc->hal->jog_enable_y);
		*(xhc->hal->jog_minus_y) = (delta_pos < 0) && *(xhc->hal->jog_enable_y);
		*(xhc->hal->jog_plus_z) = (delta_pos > 0) && *(xhc->hal->jog_enable_z);
		*(xhc->hal->jog_minus_z) = (delta_pos < 0) && *(xhc->hal->jog_enable_z);
        *(xhc->hal->jog_plus_a) = (delta_pos > 0) && *(xhc->hal->jog_enable_a);
        *(xhc->hal->jog_minus_a) = (delta_pos < 0) && *(xhc->hal->jog_enable_a);
        *(xhc->hal->jog_plus_b) = (delta_pos > 0) && *(xhc->hal->jog_enable_b);
        *(xhc->hal->jog_minus_b) = (delta_pos < 0) && *(xhc->hal->jog_enable_b);
        *(xhc->hal->jog_plus_c) = (delta_pos > 0) && *(xhc->hal->jog_enable_c);
        *(xhc->hal->jog_minus_c) = (delta_pos < 0) && *(xhc->hal->jog_enable_c);
		xhc->last_jog_counts = *(xhc->hal->jog_counts);
		xhc->last_tv = now;
	}
	else {
		*(xhc->hal->jog_velocity) = (1 - k) * *(xhc->hal->jog_velocity);
		if (elapsed > 0.25) {
			*(xhc->hal->jog_velocity) = 0;
			*(xhc->hal->jog_plus_x) = 0;
			*(xhc->hal->jog_minus_x) = 0;
			*(xhc->hal->jog_plus_y) = 0;
			*(xhc->hal->jog_minus_y) = 0;
			*(xhc->hal->jog_plus_z) = 0;
			*(xhc->hal->jog_minus_z) = 0;
            *(xhc->hal->jog_plus_a) = 0;
            *(xhc->hal->jog_minus_a) = 0;
            *(xhc->hal->jog_plus_b) = 0;
            *(xhc->hal->jog_minus_b) = 0;
            *(xhc->hal->jog_plus_c) = 0;
            *(xhc->hal->jog_minus_c) = 0;
		}
	}
}

void handle_step(xhc_t *xhc)
{
	int inc_step_status = *(xhc->hal->stepsize_up);
    //! Use a local variable to avoid STEP display as 0 on pendant during transitions
    int stepSize = *(xhc->hal->stepsize);

	if (inc_step_status  &&  ! xhc->old_inc_step_status) {
		stepsize_idx++;
		// restart idx when 0 terminator reached:
		if (stepsize_sequence[stepsize_idx] == 0) stepsize_idx = 0;
		stepSize = stepsize_sequence[stepsize_idx];
	}

	xhc->old_inc_step_status = inc_step_status;

	*(xhc->hal->stepsize) = stepSize;
	*(xhc->hal->jog_scale) = *(xhc->hal->stepsize) * 0.001f;
}

static void quit(int sig)
{
    do_exit = true;
}


void setup_async_transfer(libusb_device_handle *dev_handle)
{
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    assert(transfer != nullptr);
    libusb_fill_bulk_transfer(transfer,
                              dev_handle,
                              (0x1 | LIBUSB_ENDPOINT_IN),
                              in_buf,
                              sizeof(in_buf),
                              cb_response_in, nullptr, // no user data
                              750); // timeout
    assert(0 == libusb_submit_transfer(transfer));
}


void cb_response_in(struct libusb_transfer *transfer)
{
    switch (transfer->status) {
        case(LIBUSB_TRANSFER_COMPLETED):
            // detetec sleep mode, truncate subsequent package once
            if (Whb.sleepState.dropNextInPackage) {
                Whb.sleepState.dropNextInPackage = false;
                goto ___TRUNCATE_PACKAGE;
            }

            // clarify modifier and key
            if (transfer->actual_length > 0) {
                if (simu_mode) hexdump(in_buf, transfer->actual_length);

                uint8_t buttonCode1 = in_buf[WhbInPackageInfo::ByteType::Key1];
                uint8_t buttonCode2 = in_buf[WhbInPackageInfo::ByteType::Key2];

                uint8_t modifierCode = Whb.codes.buttons.undefined.code;
                uint8_t keyCode = Whb.codes.buttons.undefined.code;

                //! found modifier on key1, key2 is the key
                if (buttonCode1 == Whb.codes.buttons.function.code) {
                    modifierCode = Whb.codes.buttons.function.code;
                    keyCode = buttonCode2;
                }
                    //! found modifier on key2, key1 is the key
                else if (buttonCode2 == Whb.codes.buttons.function.code) {
                    modifierCode = Whb.codes.buttons.function.code;
                    keyCode = buttonCode1;
                }
                    //! no modifier, key1 and key2 are defined, fallback to key2
                else if (buttonCode2 != Whb.codes.buttons.undefined.code) {
                    keyCode = buttonCode2;
                }
                    //! fallback to whatever key1 is
                else {
                    keyCode = buttonCode1;
                }

                xhc.currentAxisCode = in_buf[WhbInPackageInfo::ByteType::Axis];
                *(xhc.hal->jog_counts) += ((signed char) in_buf[WhbInPackageInfo::ByteType::Step]);
                *(xhc.hal->jog_counts_neg) = -*(xhc.hal->jog_counts);
                *(xhc.hal->jog_enable_off) = (xhc.currentAxisCode == Whb.codes.axis.off.code);
                *(xhc.hal->jog_enable_x) = (xhc.currentAxisCode == Whb.codes.axis.x.code);
                *(xhc.hal->jog_enable_y) = (xhc.currentAxisCode == Whb.codes.axis.y.code);
                *(xhc.hal->jog_enable_z) = (xhc.currentAxisCode == Whb.codes.axis.z.code);
                *(xhc.hal->jog_enable_a) = (xhc.currentAxisCode == Whb.codes.axis.a.code);
                *(xhc.hal->jog_enable_b) = (xhc.currentAxisCode == Whb.codes.axis.b.code);
                *(xhc.hal->jog_enable_c) = (xhc.currentAxisCode == Whb.codes.axis.c.code);

                //*(xhc.hal->jog_enable_feedrate) = (xhc.currentAxisCode == axis_feed);
                //*(xhc.hal->jog_enable_spindle) = (xhc.currentAxisCode == axis_spindle);

                //		for (i=0; i<NB_MAX_BUTTONS; i++) {
                //			if (!xhc.hal->button_pin[i]) continue;
                //			*(xhc.hal->button_pin[i]) = (xhc.button_code == xhc.buttons[i].code);
                //			if (simu_mode && *(xhc.hal->button_pin[i])) {
                //				printf("%s pressed", xhc.buttons[i].pin_name);
                //			}
                //		}


                if (simu_mode) {
                    if ((char) in_buf[4] != 0) printf(" delta %+3d", (char) in_buf[4]);
                    printf(" => ");
                    printData(in_buf, transfer->actual_length);
                    printf("\n");
                }

                // update button state to hal
                int buttonsCount = sizeof(xhc.button) / sizeof(WhbSoftwareButton);
                for (int idx = 0; idx < buttonsCount; idx++) {
                    if ((xhc.button[idx].key->code == keyCode) && (xhc.button[idx].modifier->code == modifierCode)) {
                        *(xhc.hal->button_pin[idx]) = true;
                        if (simu_mode) {
                            printPushButtonText(keyCode, modifierCode);
                            printf(" pressed\n");
                        }
                    } else {
                        *(xhc.hal->button_pin[idx]) = false;
                    }
                }

                //! detect pendant going to sleep:
                //! when powering off pedant sends two packages
                //! 1st: 0x4 0x? 0x0 0x0 0x0 0x0 0x?
                //! 2nd; 0x4 0x? 0x? 0x? 0x? 0x? 0x?

                if (in_buf[WhbInPackageInfo::ByteType::NA1] == 0x04 &&
                    in_buf[WhbInPackageInfo::ByteType::Key1] == 0 &&
                    in_buf[WhbInPackageInfo::ByteType::Key2] == 0 &&
                    in_buf[WhbInPackageInfo::ByteType::Feed] == 0 &&
                    in_buf[WhbInPackageInfo::ByteType::Axis] == 0 &&
                    in_buf[WhbInPackageInfo::ByteType::Step] == 0) {
                    Whb.sleepState.dropNextInPackage = true;
                    *(xhc.hal->sleeping) = 1;
                    if (simu_mode) {
                        struct timeval now;
                        gettimeofday(&now, nullptr);
                        fprintf(stderr, "Sleep, %s was idle for %ld seconds\n",
                                Whb.entity.name, now.tv_sec - xhc.last_wakeup.tv_sec);
                    }
                } else {
                    gettimeofday(&xhc.last_wakeup, nullptr);
                    if (*(xhc.hal->sleeping)) {
                        if (simu_mode) {
                            fprintf(stderr, "Wake\n");
                        }
                    }
                    *(xhc.hal->sleeping) = 0;
                }
            }

            if (!do_exit) {
                setup_async_transfer(transfer->dev_handle);
            }

        break;

        ___TRUNCATE_PACKAGE:
        case(LIBUSB_TRANSFER_TIMED_OUT):
            if (!do_exit) {
                setup_async_transfer(transfer->dev_handle);
            }
            break;
        case(LIBUSB_TRANSFER_CANCELLED):
            break;
        case(LIBUSB_TRANSFER_STALL):
        case(LIBUSB_TRANSFER_NO_DEVICE):
        case(LIBUSB_TRANSFER_OVERFLOW):
        case(LIBUSB_TRANSFER_ERROR):
            printf("transfer error: %d", transfer->status);
            quit(0);
            break;
        default:
            printf("unknown transfer status %d\n", transfer->status);
            quit(0);
            break;
    }

    libusb_free_transfer(transfer);
}



static int hal_pin_simu(char* pin_name, void** ptr, int s)
{
    *ptr = calloc(s, 1);
    assert(*ptr != nullptr);
    memset(*ptr, 0, s);
    xhc.cleanup.refs[xhc.cleanup.nextIndex++] = *ptr;
    return 0;
}

int _hal_pin_float_newf(hal_pin_dir_t dir, hal_float_t** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, (void**)data_ptr_addr, sizeof(hal_float_t));
    }
    else {
    	return hal_pin_float_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

int _hal_pin_s32_newf(hal_pin_dir_t dir, hal_s32_t** data_ptr_addr, int comp_id, const char* fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, (void**)data_ptr_addr, sizeof(hal_s32_t));
    }
    else {
    	return hal_pin_s32_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

int _hal_pin_bit_newf(hal_pin_dir_t dir, hal_bit_t** data_ptr_addr, int comp_id, const char *fmt, ...)
{
	char pin_name[256];
    va_list args;
    va_start(args,fmt);
	vsprintf(pin_name, fmt, args);
	va_end(args);

    if (simu_mode) {
    	return hal_pin_simu(pin_name, (void**)data_ptr_addr, sizeof(hal_bit_t));
    }
    else {
    	return hal_pin_bit_new(pin_name, dir, data_ptr_addr, comp_id);
    }
}

static void hal_teardown() {
    if (simu_mode) {
        if (xhc.hal != nullptr) {
            for (uint16_t idx = 0; idx < xhc.cleanup.nextIndex; idx++)
            {
                free(xhc.cleanup.refs[idx]);
            }
            free(xhc.hal);
        }

        xhc.cleanup.nextIndex = 0;
        xhc.hal = nullptr;
    }
}

static void hal_setup()
{
	const char* modname = Whb.entity.name;

	if (!simu_mode) {
		hal_comp_id = hal_init(modname);
		if (hal_comp_id < 1) {
			fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
			exit(1);
		}

		xhc.hal = (xhc_hal_t*)hal_malloc(sizeof(xhc_hal_t));
		if (xhc.hal == nullptr) {
			fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
            hal_exit(hal_comp_id);
			exit(1);
		}
	}
	else {
		xhc.hal = (xhc_hal_t*)calloc(sizeof(xhc_hal_t), 1);
        memset(xhc.hal, 0, sizeof(xhc_hal_t));
	}

    // register all known whb04b-6 buttons
    int buttonsCount = sizeof(xhc.button) / sizeof(WhbSoftwareButton);
    for (int idx = 0; idx < buttonsCount; idx++) {
        const char* buttonName = nullptr;
        if ( xhc.button[idx].modifier == &Whb.codes.buttons.undefined)
        {
            buttonName = xhc.button[idx].key->text;
        } else
        {
            buttonName = xhc.button[idx].key->altText;
        }
        assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->button_pin[idx]), hal_comp_id,
                               "%s.%s", modname, buttonName));
    }

    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->x_mc), hal_comp_id, "%s.x.pos-absolute", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->y_mc), hal_comp_id, "%s.y.pos-absolute", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->z_mc), hal_comp_id, "%s.z.pos-absolute", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->a_mc), hal_comp_id, "%s.a.pos-absolute", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->b_mc), hal_comp_id, "%s.b.pos-absolute", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->c_mc), hal_comp_id, "%s.c.pos-absolute", modname));

    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->x_wc), hal_comp_id, "%s.x.pos-relative", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->y_wc), hal_comp_id, "%s.y.pos-relative", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->z_wc), hal_comp_id, "%s.z.pos-relative", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->a_wc), hal_comp_id, "%s.a.pos-relative", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->b_wc), hal_comp_id, "%s.b.pos-relative", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->c_wc), hal_comp_id, "%s.c.pos-relative", modname));

    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->feedrate),          hal_comp_id, "%s.feed-value", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->feedrate_override), hal_comp_id, "%s.feed-override", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->spindle_rps),       hal_comp_id, "%s.spindle-rps", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->spindle_override),  hal_comp_id, "%s.spindle-override", modname));

    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->sleeping),        hal_comp_id, "%s.sleeping", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->connected),       hal_comp_id, "%s.connected", modname));
    assert(0 == _hal_pin_bit_newf(HAL_IN,  &(xhc.hal->stepsize_up),     hal_comp_id, "%s.stepsize-up", modname));
    assert(0 == _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->stepsize),        hal_comp_id, "%s.stepsize", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->require_pendant), hal_comp_id, "%s.require_pendant", modname));

    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_off), hal_comp_id, "%s.jog.enable-off", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_x),   hal_comp_id, "%s.jog.enable-x", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_y),   hal_comp_id, "%s.jog.enable-y", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_z),   hal_comp_id, "%s.jog.enable-z", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_a),   hal_comp_id, "%s.jog.enable-a", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_b),   hal_comp_id, "%s.jog.enable-b", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_c),   hal_comp_id, "%s.jog.enable-c", modname));

    //r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_feedrate), hal_comp_id, "%s.jog.enable-feed-override", modname);
    //r |= _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_enable_spindle), hal_comp_id, "%s.jog.enable-spindle-override", modname);

    assert(0 == _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_scale),    hal_comp_id, "%s.jog.scale", modname));
    assert(0 == _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->jog_counts),     hal_comp_id, "%s.jog.counts", modname));
    assert(0 == _hal_pin_s32_newf(HAL_OUT, &(xhc.hal->jog_counts_neg), hal_comp_id, "%s.jog.counts-neg", modname));

    assert(0 == _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_velocity),    hal_comp_id, "%s.jog.velocity", modname));
    assert(0 == _hal_pin_float_newf(HAL_IN, &(xhc.hal->jog_max_velocity), hal_comp_id, "%s.jog.max-velocity", modname));
    assert(0 == _hal_pin_float_newf(HAL_OUT, &(xhc.hal->jog_increment),   hal_comp_id, "%s.jog.increment", modname));

    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_x), hal_comp_id, "%s.jog.plus-x", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_y), hal_comp_id, "%s.jog.plus-y", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_z), hal_comp_id, "%s.jog.plus-z", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_a), hal_comp_id, "%s.jog.plus-a", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_b), hal_comp_id, "%s.jog.plus-b", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_plus_c), hal_comp_id, "%s.jog.plus-c", modname));

    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_x), hal_comp_id, "%s.jog.minus-x", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_y), hal_comp_id, "%s.jog.minus-y", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_z), hal_comp_id, "%s.jog.minus-z", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_a), hal_comp_id, "%s.jog.minus-a", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_b), hal_comp_id, "%s.jog.minus-b", modname));
    assert(0 == _hal_pin_bit_newf(HAL_OUT, &(xhc.hal->jog_minus_c), hal_comp_id, "%s.jog.minus-c", modname));

	return;
}

/*
int read_ini_file(char *filename)
{
	FILE *fd = fopen(filename, "r");
	const char *bt;
	int nb_buttons = 0;
	if (!fd) {
		perror(filename);
		return -1;
	}

	IniFile f(false, fd);

	while ( (bt = f.Find("BUTTON", WHB.entity.configSectionName, nb_buttons+1)) && nb_buttons < NB_MAX_BUTTONS) {
		if (sscanf(bt, "%x:%s", &xhc.buttons[nb_buttons].code, xhc.buttons[nb_buttons].pin_name) !=2 ) {
			fprintf(stderr, "%s: syntax error\n", bt);
			return -1;
		}
		nb_buttons++;
	}

	return 0;
}
*/

#define STRINGIFY_IMPL(S) #S
#define STRINGIFY(s) STRINGIFY_IMPL(s)

static void Usage(char *name)
{
	fprintf(stderr, "%s version %s by Frederic RIBLE (frible@teaser.fr)\n", name, PACKAGE_VERSION);
    fprintf(stderr, "Usage: %s [-I ini-file] [-h] [-H] [-s 1|2]\n", name);
    //fprintf(stderr, " -I ini-file: configuration file defining the MPG keyboard layout\n");
    fprintf(stderr, " -h: usage (this)\n");
    fprintf(stderr, " -H: run in real-time HAL mode (run in simulation mode by default)\n");
    fprintf(stderr, " -x: wait for pendant detection before creating HAL pins\n");
    fprintf(stderr, " -s: step sequence (*.001 unit):\n");
    fprintf(stderr, "     1: 1,10,100,1000 (default)\n");
    fprintf(stderr, "     2: 1,5,10,20\n\n");
    fprintf(stderr, "Configuration file section format:\n");
    fprintf(stderr, "[%s]\n", Whb.entity.configSectionName);
    fprintf(stderr, "BUTTON=XX:button-thename\n");
    fprintf(stderr, "...\n");
    fprintf(stderr, "    where XX=hexcode, thename=nameforbutton\n");
}

int main (int argc,char **argv)
{
	libusb_device **devs;
    //libusb_device_handle *dev_handle;
	//libusb_context *ctx = nullptr;

    XhbUsb usb = {
        .context = nullptr,
        .deviceHandle = nullptr
    };


	int r;
	ssize_t cnt;
#define MAX_WAIT_SECS 10
	int wait_secs = 0;

    int opt;
    bool hal_ready_done = false;

    init_xhc(&xhc);

    while ((opt = getopt(argc, argv, "HhI:xs:")) != -1) {
        switch (opt) {
        /*case 'I':
            if (read_ini_file(optarg)) {
                printf("Problem reading ini file: %s\n\n",optarg);
                Usage(argv[0]);
                exit(EXIT_FAILURE);
            }
            break;*/
        case 'H':
        	simu_mode = false;
        	break;
        case 's':
            switch (optarg[0]) {
              case '1': stepsize_sequence = stepsize_sequence_1;break;
              case '2': stepsize_sequence = stepsize_sequence_2;break;
              default:
                printf("Unknown sequence: %s\n\n",optarg);
                Usage(argv[0]);
                exit(EXIT_FAILURE);
                break;
            }
            break;
        case 'x':
        	wait_for_pendant_before_HAL = true;
        	break;
        default:
        	Usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }

	hal_setup();

    signal(SIGINT, quit);
	signal(SIGTERM, quit);

    if (!wait_for_pendant_before_HAL && !simu_mode) {
    	hal_ready(hal_comp_id);
    	hal_ready_done = true;
    }

	while (!do_exit) {
    	//on reconnect wait for device to be gone
    	if (do_reconnect == true) {
    		sleep(5);
    		do_reconnect = false;
    	}
    
		r = libusb_init(&usb.context);

		if(r != 0) {
			perror("libusb_init");
			return 1;
		}
		libusb_set_debug(usb.context, 3);

		printf("%s: waiting for %s device\n", Whb.entity.name, Whb.entity.configSectionName);
		*(xhc.hal->connected) = 0;
		wait_secs = 0;
		*(xhc.hal->require_pendant) = wait_for_pendant_before_HAL;
		*(xhc.hal->stepsize) = stepsize_sequence[0];

		do {
			cnt = libusb_get_device_list(usb.context, &devs);
			if (cnt < 0) {
				perror("libusb_get_device_list");
				return 1;
			}

			usb.deviceHandle = libusb_open_device_with_vid_pid(usb.context,
                                                               Whb.entity.usbVendorId,
                                                               Whb.entity.usbProductId);
			libusb_free_device_list(devs, 1);
			if (usb.deviceHandle == nullptr) {
				if (wait_for_pendant_before_HAL) {
					wait_secs++;
					if (wait_secs >= MAX_WAIT_SECS/2) {
						printf("%s: waiting for %s device (%d)\n",  Whb.entity.name,
							   Whb.entity.configSectionName, wait_secs);
					}
					if (wait_secs > MAX_WAIT_SECS) {
						printf("%s: MAX_WAIT_SECS exceeded, exiting\n",  Whb.entity.name);
						exit(1);
					}
				}
				sleep(1);
			}
		} while(usb.deviceHandle == nullptr && !do_exit);

		printf("%s: found %s device\n", Whb.entity.name, Whb.entity.configSectionName);

		if (usb.deviceHandle != nullptr) {
			if 	(libusb_kernel_driver_active(usb.deviceHandle, 0) == 1) {
				assert(0 == libusb_detach_kernel_driver(usb.deviceHandle, 0));
			}

			r = libusb_claim_interface(usb.deviceHandle, 0);
			if (r != 0) {
				perror("libusb_claim_interface");
				return 1;
			}
		}

		*(xhc.hal->connected) = 1;

	    if (!hal_ready_done && !simu_mode) {
	    	hal_ready(hal_comp_id);
	    	hal_ready_done = true;
	    }

		if (usb.deviceHandle != nullptr) {
			setup_async_transfer(usb.deviceHandle);
			xhc_set_display(usb.deviceHandle, &xhc);
		}

		if (usb.deviceHandle != nullptr) {
			while (!do_exit && !do_reconnect) {
				struct timeval tv;
				tv.tv_sec  = 0;
				tv.tv_usec = 30000;
				libusb_handle_events_timeout_completed(usb.context, &tv, nullptr);
				compute_velocity(&xhc);
			    if (simu_mode) linuxcnc_simu(&xhc);
				handle_step(&xhc);
				xhc_set_display(usb.deviceHandle, &xhc);
			}
			*(xhc.hal->connected) = 0;
            printf("%s: connection lost, cleaning up\n",Whb.entity.name);
            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 750000;
            assert(0 == libusb_handle_events_timeout_completed(usb.context, &tv, nullptr));
			assert(0 == libusb_release_interface(usb.deviceHandle, 0));
			libusb_close(usb.deviceHandle);
            usb.deviceHandle = nullptr;
		}
		else {
			while (!do_exit) usleep(70000);
		}
		libusb_exit(usb.context);
        usb.context = nullptr;
    }
    hal_teardown();
}
