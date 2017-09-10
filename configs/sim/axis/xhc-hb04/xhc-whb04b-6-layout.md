# XHC WHB04B-6 layout

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


# XHC WHB04B-6 button naming

The HAL pin names are derived from the case sensitive text as written on
the respecitve button. For example:

```
xhc-whb04b-6.RESET
xhc-whb04b-6.Macro-11
xhc-whb04b-6.STOP
xhc-whb04b-6.Macro-12
xhc-whb04b-6.Start
xhc-whb04b-6.Pause
xhc-whb04b-6.Feed+
xhc-whb04b-6.Macro-1
xhc-whb04b-6.Feed-
xhc-whb04b-6.Macro-2
xhc-whb04b-6.Spindle+
...
```

# For an extensive list of HAL pins consider running:

```
xhc-whb04b-6 -p
```

# For even more details:

```
$ ./xhc-whb04b-6 -h
xhc-whb04b-6 version 0.1 Sep 10 2017 08:57:04

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
    run XHC-WHB04B-6 in HAL-mode instead of interactive mode. When in HAL mode commands from device will be exposed to HAL's shred memory. Interactive mode is useful for testing device connectivity and debugging.

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
    
 -s 
    Force being silent and not printing any output except of errors.

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
