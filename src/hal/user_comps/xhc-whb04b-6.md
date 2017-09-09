# XHC WHB04B-6 Protocol description
Since the manufacturer's ([Chengdu Xinhecheng Technology Co.,Ltd.](http://cdxhctech.com/)) developers refuse to release any protocol information, we had to reverse engineer the protocol.
After lots of begging we received at least some sort of source code for PHB04 - it was hard to believe that this was serious productive code in terms of programming capability. 
However here we list findings and thoughts on the USB communication protocol.

**Any discussion regarding this topic is welcome!**

## Findings

### Received data structure

| Byte# | Width | Data                        | Value                    | Clarification Needed | 
|:------|:------|:----------------------------|:-------------------------|:-:|
| 0x00  | [0:7] | retport ID                  | constant 0x04            |   |
| 0x01  | [0:7] | random                      |                          |   | 
| 0x02  | [0:7] | button 1 key code           | 0x00-0x10                |   |
| 0x03  | [0:7] | button 2 key code           | 0x00-0x10                |   |
| 0x04  | [0:7] | feed rotary button key code | 0x0d-0x10, 0x1a-0x1c     |   | 
| 0x05  | [0:7] | axis rotary button key code | 0x11-0x16, 0x06          |   | 
| 0x06  | [0:7] | jog dial delta | int8_t     |                          |   | 
| 0x07  | [0:7] | checksum                    |                          | * |

* On jog dial, 
* on rotary button or 
* on button released event:
```
checksum == random & seed ==
```

* On button pressed event:
```
//! works most of the cases, some quation part is missing
checksum == random - (keyCode ^ (~seed & random)) 
```

### Transmission data structure
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
| 0x20  | [0:x]   | **unclear if axis A, B, C are also transmitted or just lines 1-3**   |                     | * |
| 0x20  | [0:x]   | unclear if the device interprets subsequent bytes                    |                     | * |
| 0xn   | [0:x]   | the **maximum length** is also **unclear**                           |                     | * |

## What we did so far
* Searched the web and found not 100% related but interesting information on this [site](http://forum.planet-cnc.com/viewtopic.php?f=12&t=1125).
* Politely contacted the manufacturer and requested an interface controld document or equivalent information (without success).

## What we didn't
* Did not install the driver and Mach3 on Windows guest VM and sniff the USB protocol using SOTA tools such as
    * Wireshark
    * usbmon
    
**Any help in that regard is appreciated.**

## Issues
* if de device is powered on it does not (always) send data on its own to disclose the current rotary buttons' state
    * does send most likely if the rotary buttons' state has changed during power-off, but not always
* if the axis rotary button is in "OFF" state, the device does not refresh coordinates on display

## Key codes in detail

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
