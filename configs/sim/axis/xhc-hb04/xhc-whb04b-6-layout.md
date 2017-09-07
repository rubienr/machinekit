# XHC WHB04B-6 Layout

The device provides 16 buttons. One button - "Fn" - is used as key
modifier. Any button pressed in combination with the modifier button
generates an alternative event - Macro-n (with n being the macro
number). This holds for any button, even if Macro-n is not written
on the button. We have extended the macros beyond Macro-10 until
Macro-14. The keypad layout is as follows:

|                             |                  |                    |                    |                            |
|:---------------------------:|:----------------:|:------------------:|:------------------:|:--------------------------:|
| RESET (Macro-11)            | Stop  (Macro-12) | Start/Pause        |                    | &lt;On/Off Sw.&gt;         |
| Feed + (Macro-1)            | Feed-  (Macro-2) | Spindle+ (Macro-3) | Spindle- (Macro-4) |                            |
| M-HOME (Macro-5)            | Safe-Z (Macro-6) | W-HOME   (Macro-7) | S-ON/OFF (Macro-8) | Fn                         |
|                             |                  | Probe-Z (Macro-9)  |                    |                            |
| &lt;Axis Rotary Button&gt;  |                  | Macro-10           |                    | &lt;Feed Rotary Button&gt; |
| MPG (Macro-13)              |                  |                    |                    | Step (Macro-14)            |
|                             |                  | &lt;Jog Dial&gt;   |                    |                            |


# XHC WHB04B-6 Button Naming

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

# For more details:

```
Usage: xhc-whb04b-6 [-h] | [-H] [-x] [[-u|-U] [-p] | [-a] | [-s]] 
 -h usage help text
 -H run XHC-WHB04B-6in HAL-mode instead of interactive mode
 -t wait for USB device before processing with HAL initialization
 -u print received data
 -U print received and transmitted data
 -p print initialized HAL pins 
 -e print key events
 -a enable all verbose facilities
 -s be silent
 ```
