# XHC WHB04B-6 pendant HAL module

## Pendant layout

The device provides 16 buttons. One button - "Fn" - is used as key
modifier. Any button pressed in combination with the modifier button
generates an alternative event - Macro-n (with n being the macro
number). This holds for any button, even if Macro-n is not written
on the button. We have extended the macros beyond Macro-10 until
Macro-16. The keypad layout is as follows:

|                             |                  |                        |                    |                            |
|:---------------------------:|:----------------:|:----------------------:|:------------------:|:--------------------------:|
| RESET (Macro-11)            | Stop  (Macro-12) | Start/Pause (Macro-13) |                    | &lt;On/Off Sw.&gt;         |
| Feed + (Macro-1)            | Feed-  (Macro-2) | Spindle+ (Macro-3)     | Spindle- (Macro-4) |                            |
| M-HOME (Macro-5)            | Safe-Z (Macro-6) | W-HOME   (Macro-7)     | S-ON/OFF (Macro-8) | Fn                         |
|                             |                  | Probe-Z (Macro-9)      |                    |                            |
| &lt;Axis Rotary Button&gt;  |                  | Macro-10 (Macro-14)    |                    | &lt;Feed Rotary Button&gt; |
| MPG (Macro-15)              |                  |                        |                    | Step (Macro-16)            |
|                             |                  | &lt;Jog Dial&gt;       |                    |                            |

* Caution: The buttons' naming and user manual are sohewhat unluckily translated. The **MPG** button puts the device into **continuous**, whereas the **Step/Continuous** button into **step mode**. Continuous mode means n jog dial events are translated to "move joint n times with speed x" whereas step mode to "move joint n steps" with a predefined speed.

### Pendant button naming

The HAL pin names are derived from text written on the respecitve button. For example:

```
$ xhc-whb04b-6 -p
init  starting in simulation mode
hal   new pin xhc-whb04b-6.out.button.reset
hal   new pin xhc-whb04b-6.out.button.macro-11
hal   new pin xhc-whb04b-6.out.button.stop
hal   new pin xhc-whb04b-6.out.button.macro-12
hal   new pin xhc-whb04b-6.out.button.start-pause
hal   new pin xhc-whb04b-6.out.button.macro-13
hal   new pin xhc-whb04b-6.out.button.feed-plus
hal   new pin xhc-whb04b-6.out.button.macro-1
hal   new pin xhc-whb04b-6.out.button.feed-minus
hal   new pin xhc-whb04b-6.out.button.macro-2
hal   new pin xhc-whb04b-6.out.button.spindle-plus
hal   new pin xhc-whb04b-6.out.button.macro-3
hal   new pin xhc-whb04b-6.out.button.spindle-minus
hal   new pin xhc-whb04b-6.out.button.macro-4
hal   new pin xhc-whb04b-6.out.button.m-home
hal   new pin xhc-whb04b-6.out.button.macro-5
hal   new pin xhc-whb04b-6.out.button.safe-z
hal   new pin xhc-whb04b-6.out.button.macro-6
hal   new pin xhc-whb04b-6.out.button.w-home
hal   new pin xhc-whb04b-6.out.button.macro-7
hal   new pin xhc-whb04b-6.out.button.s-on-off
hal   new pin xhc-whb04b-6.out.button.macro-8
hal   new pin xhc-whb04b-6.out.button.fn
hal   new pin xhc-whb04b-6.out.button.probe-z
hal   new pin xhc-whb04b-6.out.button.macro-9
hal   new pin xhc-whb04b-6.out.button.macro-10
hal   new pin xhc-whb04b-6.out.button.macro-14
hal   new pin xhc-whb04b-6.out.button.mode-continuous
hal   new pin xhc-whb04b-6.out.button.macro-15
hal   new pin xhc-whb04b-6.out.button.mode-step
hal   new pin xhc-whb04b-6.out.button.macro-16

...
```

#### For an extensive list of HAL pins consider running:

```
xhc-whb04b-6 -p
```

#### For even more details:

```
$ ./xhc-whb04b-6 -h
xhc-whb04b-6 version 0.1 Sep 16 2017 14:41:32

SYNOPSIS
    xhc-whb04b-6 [-h] | [-H] [OPTIONS] 

NAME
    xhc-whb04b-6 - jog dial HAL module for the XHC-WHB04B-6 device

DESCRIPTION
    xhc-whb04b-6 is a HAL module that receives events from the XHC-WHB04B-6 device and exposes them to HAL via HAL pins.

OPTIONS
 -h 
    Prints the synonpsis and the most commonly used commands.

 -H 
    run xhc-whb04b-6 in HAL-mode instead of interactive mode. When in HAL mode commands from device will be exposed to HAL's shred memory. Interactive mode is useful for testing device connectivity and debugging.

 -t 
    Wait with timeout for USB device then proceed, exit otherwise. Without -t the timeout is ipmlicitely infinite.

 -u, -U 
    Show received data from device. With -U received and transmitted data will be printed. Output is prefixed with "usb".

 -p 
    Show HAL pins and HAL related messages. Output is prefixed with "hal".

 -e 
    Show captured events such as button pressed/released, jog dial, axis rotary button, and feed rotary button event. Output is prefixed with "event".and in case.

 -a 
    Enable all logging facilities without explicitly specifying each.

 -c 
    Enable checksum output which is necessary for debugging the checksum generator function. Do not rely on this featue since it will be removed once the generator is implemented.

 -n 
    Force being silent and not printing any output except of errors. This will also inhibit messages prefixed with "init".

 -s <scale>
    Specifies the number of pulses that corresponds to a move of one machine unit in [mm] or [inch]. Default is 80.

 -v <max_velocity>
    The maximum velocity for any axis in machine units per second (same unit as -s). Default is 800.

EXAMPLES
xhc-whb04b-6 -ue
    Prints incoming USB data transfer and generated key pressed/released events.

xhc-whb04b-6 -p
    Prints hal pin names and events distributed to HAL memory.

xhc-whb04b-6 -Ha
    Start in HAL mode and avoid output, except of errors.

AUTHORS
    This module was started by Raoul Rubien (github.com/rubienr) based on predecessor device's module xhc-hb04.cc. https://github.com/machinekit/machinekit/graphs/contributors gives you a more complete list of contributors.
 ```

## Protocol description
Since the manufacturer's ([Chengdu Xinhecheng Technology Co.,Ltd.](http://cdxhctech.com/)) developers refuse to release any protocol information, we had to reverse engineer the protocol.
After lots of begging we received at least some sort of source code for PHB04 - it was hard to believe that this was serious productive code in terms of programming capability. 
However here we list findings and thoughts on the USB communication protocol.

**Any discussion regarding this topic is welcome!**

### Findings

#### Received data structure

| Byte# | Width | Data                        | Value                    | Clarification Needed | 
|:------|:------|:----------------------------|:-------------------------|:-:|
| 0x00  | [0:7] | retport ID                  | constant 0x04            |   |
| 0x01  | [0:7] | random                      |                          |   | 
| 0x02  | [0:7] | button 1 key code           | 0x00-0x10                |   |
| 0x03  | [0:7] | button 2 key code           | 0x00-0x10                |   |
| 0x04  | [0:7] | feed rotary button key code | 0x0d-0x10, 0x1a-0x1c     |   | 
| 0x05  | [0:7] | axis rotary button key code, **if axis in state OFF, display cannot be updated** | 0x11-0x16, 0x06          | * | 
| 0x06  | [0:7] | jog dial delta              | int8_t                   |   | 
| 0x07  | [0:7] | checksum                    |                          | * |

##### Checksum investigation

* On jog dial, 
* on rotary button or 
* on button released event:
```
checksum == random & seed
```

* On button pressed event:
```
//! Works if seed is 0xfe, 0xff, otherwise not reliable. Some equation part must be missing.
//! Not sure whether crc or hand-crafted algorithm is applied.
checksum == random - (keyCode ^ (~seed & random)) 
```

#### Transmission data structure
```
USB vendor  ID = 0x10ce
USB product ID = 0xeb93
```

Data transmitted is packed as 7 bytes plus a constant leading byte 0x06
which is the report ID. The data **exclusive report ID** reads as follows:

| Byte# | Width   | Data                                                                 | Value               | Clarification Needed | 
|:------|:--------|:---------------------------------------------------------------------|:--------------------|:-:|
| 0x00  | [0:15]  | header, **unclear if different headers (commands) can be sent**      | constant 0xfdfe     | * |
| 0x02  | [0:7]   | seed                                                                 |                     | * |
| 0x03  | [0:1]   | display indicator flags: step mode                                   |                     |   |
| 0x03  | [2:5]   | display indicator flags: **unknown**                                 |                     | * |
| 0x03  | [6:6]   | display indicator flags: reset                                       |                     |   |
| 0x03  | [7:7]   | display indicator flags: machine coordinate                          |                     |   |
| 0x04  | [0:15]  | axis coordinate on display line 1: integer value                     |                     |   |
| 0x06  | [0:14]  | axis coordinate on display line 1: fraction value                    | 15bit width but device cuts off to 4 digits, **seems to be a firmware bug** | |
| 0x06  | [15:15] | axis coordinate on display line 1: sign                              |                     |   |
| 0x08  | [0:15]  | axis coordinate on display line 2: integer value                     |                     |   |
| 0x10  | [0:14]  | axis coordinate on display line 2: fraction value                    | **same as axis 1**  |   |
| 0x10  | [15:15] | axis coordinate on display line 2: sign                              |                     |   |
| 0x12  | [0:15]  | axis coordinate on display line 3: integer value                     |                     |   |
| 0x14  | [0:14]  | axis coordinate on display line 3: fraction value                    |                     |   |
| 0x14  | [15:15] | axis coordinate on display line 3: sign                              | **same as axis 1**  |   |
| 0x16  | [0:15]  | feed rate                                                            |                     |   |
| 0x18  | [0:15]  | spindle speed                                                        |                     |   |
| 0x20  | [0:x]   | **unclear if axis A, B, C, X1, Y1, Z1, A1, B1, C1 are also transmitted or just coordinates for lines 1-3**   |                     | * |
| 0x20  | [0:x]   | unclear if the device interprets subsequent bytes                    |                     | * |
| 0xn   | [0:x]   | the **maximum length** is also **unclear**                           |                     | * |

### What we did so far
* Searched the web and found not 100% related but interesting information on this [site](http://forum.planet-cnc.com/viewtopic.php?f=12&t=1125), and
* this [site](http://wiki.linuxcnc.org/cgi-bin/wiki.pl?Using_A_XHC-HB04_Wireless_MPG_Pendant).
* Politely contacted the manufacturer and requested an interface controld document or equivalent information (without success).

### What we didn't
* Did not install the driver and Mach3 on Windows guest VM and sniff the USB protocol using SOTA tools such as
    * Wireshark
    * usbmon
    
**Any help in that regard is appreciated.**

### Issues
* if de device is powered on it does not (always) send data on its own to disclose the current rotary buttons' state
    * does send most likely if the rotary buttons' state has changed during power-off, but not always
* if the axis rotary button is in "OFF" state, the device does not refresh coordinates on display

### Key codes in detail

|Button Name              | Key Code | Button Text | Button Alternative Text |
|:------------------------|:---------|:------------|:------------------------|
|reset                    | 0x01     | RESET       | *Macro-11*              |
|stop                     | 0x02     | STOP        | *Macro-12*              |
|start                    | 0x03     | Start       | Pause                   |
|feed_plus                | 0x04     | Feed+       | Macro-1                 |
|feed_minus               | 0x05     | Feed-       | Macro-2                 |
|spindle_plus             | 0x06     | Spindle+    | Macro-3                 |
|spindle_minus            | 0x07     | Spindle-    | Macro-4                 |
|machine_home             | 0x08     | M-HOME      | Macro-5                 |
|safe_z                   | 0x09     | Safe-Z      | Macro-6                 |
|workpiece_home           | 0x0a     | W-HOME      | Macro-7                 |
|spindle_on_off           | 0x0b     | S-ON/OFF    | Macro-8                 |
|function                 | 0x0c     | Fn          | Fn                      |
|probe_z                  | 0x0d     | Probe-Z     | Macro-9                 |
|macro10                  | 0x10     | Macro-10    | *Macro-13*              |
|manual_pulse_generator   | 0x0e     | MPG         | *Macro-14*              |
|step_continuous          | 0x0f     | STEP        | Continuous              | 
|&lt;no button pressed&gt;| 0x00     | &lt;NA&gt;  | &lt;NA&gt;              |

|  Feed Rotary Button Name    | Key Code | Button Text | Button Alternative Text |
|:----------------------------|:---------|:------------|:------------------------|
| speed_0_001                 | 0x0d     | 0.001       | 2%                      |
| speed_0_01                  | 0x0e     | 0.01        | 5%                      |
| speed_0_1                   | 0x0f     | 0.1         | 10%                     |
| speed_1                     | 0x10     | 1           | 30%                     |
| percent_60                  | 0x1a     | &lt;NA&gt;  | 60%                     |
| percent_100                 | 0x1b     |   | 100%                    |
| lead                        | 0x1c     | Lead        | &lt;NA&gt;              |
| &lt;no button pressed&gt;   | 0x00     | &lt;NA&gt;  | &lt;NA&gt;              |

|  Feed Rotary Button Name    | Key Code | Button Text | Button Alternative Text |
|:----------------------------|:---------|:------------|:------------------------|
| off                         | 0x06     | OFF         | &lt;NA&gt;              |
| x                           | 0x11     | X           | &lt;NA&gt;              |
| y                           | 0x12     | Y           | &lt;NA&gt;              |
| z                           | 0x13     | Z           | &lt;NA&gt;              |
| a                           | 0x14     | A           | &lt;NA&gt;              |
| b                           | 0x15     | B           | &lt;NA&gt;              |
| c                           | 0x16     | C           | &lt;NA&gt;              |
| &lt;no button pressed&gt;   | 0x00     | &lt;NA&gt;  | &lt;NA&gt;              |
