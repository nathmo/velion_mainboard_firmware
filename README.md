# velion_mainboard_firmware
This is the firmware that control the main board and control pannel of the vehicle.

# General logic
Use ESP32 FreeRTOS on Arduino  IDE (ESP32 by Espressif, V 2.0.17)

there is a mainloop which first read the input, generate event. then iterrate over all state machine and feed them the event. then there the output stage which run the PID step and the output monitoring.

the following loop should run at least at 1 Khz :

INPUTS 
Call AceButton and AceRotary to handle buttons and give event
read all the sensor
process CAN message

Event generation
process state and generate event

LOGIC FSMs
execute each FSM with the event table

OUTPUT 
PID V1
CAN message broadcasting
PWM ouput
on/off toggling

# Pin mapping  (HAL)
GPIO 36 : Analog input  (THROTTLE_INPUT)
GPIO 39 : Digital input (InterruptEXTGPIO)
GPIO 34 : rotary encoder 1, channel A (POT_SPEED_A)
GPIO 35 : rotary encoder 1, channel B (POT_SPEED_B)
GPIO 32 : rotary encoder 2, channel A (POT_SEATHEATER_A)
GPIO 33 : rotary encoder 2, channel B (POT_SEATHEATER_B)
GPIO 25 : rotary encoder 3, channel A (POT_HANDHEATER_A)
GPIO 26 : rotary encoder 3, channel B (POT_HANDHEATER_B)
GPIO 13 : mosfet gate for defroster load (DEFROSTER_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 15 : mosfet gate for interrior light (INTERRIOR_LIGHT_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 2 : mosfet gate for Blinker LEFT (BLIKER_LEFT_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 0 : mosfet gate for Blinker RIGHT (BLIKER_RIGHT_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 4 : mosfet gate for hand heater load (HANDHEATER_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 16 : mosfet gate for DRL light RIGHT (DRL_RIGHT_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 17 : mosfet gate for DRL light LEFT (DRL_LEFT_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 5 : mosfet gate for SEAT heater load (SEATHEATER_MOSFET_GATE) (drive with PWM duty cycle and ON / OFF)
GPIO 21 : RGBLED_DIN (WS2812B, 5 led, drive them from this pin)
GPIO 23 : SDA (has a pullup of 4k7 aldready )
GPIO 22 : SCL (has a pullup of 4k7 aldready )

I have two MCP23017_SO over I2C whose IO I also want to control over serial like if it was a GPIO of the ESP32.

the one with address 0100001 = 0x21 has : 
GPB0 : Warning_LED
GPB1 : CABLIGHT_LED
GPB2 : DEFROST_LED
GPB3 : TRUNK_LED
GPB4 : BROUILLARD_LED
GPB5 : MAP_LED
GPB6 : DEFROST_INPUT (button, enable the pull-up)
GPB7 : TRUNK_INPUT (button, enable the pull-up)

GPA0 : AUXAUDIO_MOSFET_GATE (drive with ON / OFF)
GPA1 : AUXUSB_MOSFET_GATE (drive with ON / OFF)
GPA2 : LATCH_TRUNK_MOSFTET_GATE (drive with ON / OFF)
GPA3 : POWER_MOSFET_GATE (drive with ON / OFF)

the second MCP with address 0100000 = 0x20

GPB0 : Defroster_ALERT (input that must be configured with interrupt on INTA)
GPB1 : HANDHEATER_ALERT (input that must be configured with interrupt on INTA)
GPB2 : SEATHEATER_ALERT (input that must be configured with interrupt on INTA)
GPB3 : POWERINPUT_ALERT (input that must be configured with interrupt on INTA)
GPB4 : POT_SEATHEATER_X (input switch for the rotary encoder that connect when pressed, dont require a pullup)
GPB5 : POT_HANDHEATER_X (input switch for the rotary encoder that connect when pressed, dont require a pullup)
GPB6 : POT_SPEED_X (input switch for the rotary encoder that connect when pressed, dont require a pullup)
GPB7 : SEAT_INPUT (button, enable the pull-up)

GPA0 : BRAKE_RIGHT_INPUT (button, enable the pull-up)
GPA1 : BRAKE_LEFT_INPUT (button, enable the pull-up)
GPA2 : REVERSE_INPUT (button, enable the pull-up)
GPA3 : FORWARD_INPUT (button, enable the pull-up)
GPA4 : BROUILLARD_INPUT (button, enable the pull-up)
GPA5 : CABLIGHT_INPUT (button, enable the pull-up)
GPA6 : WARNING_INPUT (button, enable the pull-up)
GPA7 : MAP_INPUT (button, enable the pull-up)

I also have a GY-91 with MPU-9250 and BMP280 as a 10 DOF IMU. address for the  MPU9250 shoud be 1101000 = 0x68, BMP : 1110110 = 0x76

there is also four INA228 with 1 mOhm resistor. the current measured should never exceed 50 A.
0x40 is the address of the INA measuring the total current consumption
0x41 is the address of the INA measuring the hand heater current consumption
0x44 is the address of the INA measuring the seat heater current consumption
0x45 is the address of the INA measuring the defroster current consumption

there are also all the CAN packet that can set virtual input (for instance, the luminisity level from the front and rear board or the error status of the lightning system or the battery SOC, SOH, the PAS cadense broadcasted by the motor controller, the front and rear proximity sensor) or act as virtual output (blinker left / right, fog beam, ...)

TIMER, for each FSM as needed. they can be implemented in software / hardware and thus have a HAL

# CAN messages


| System            | ID Range Used |
| ----------------- | ------------- |
| mainboard         | `0x420–0x480` |
| rearboard         | `0x481–0x490` |
| frontboard        | `0x491–0x500` |
| siliXcon Request  | `0x630–0x637` |
| siliXcon Response | `0x640–0x647` |
| siliXcon Control  | `0x5FF`       |
| VDS Button Event  | `0x5FE`       |
| BMS Telemetry     | `0x610–0x619` |
| BMS Heartbeat RX  | `0x601`       |
| Throttle Input    | `0x407`       |

## RX: Throttle Input (`0x407`)

Received from the control board. Contains the throttle 8-bit ADC value.

| Byte | Type   | Description          |
| ---- | ------ | -------------------- |
| 0    | UINT_8 | Throttle ADC (0–255) |

- Stored in `canThrottleRx`.
- If no 0x407 message is received within 500 ms, the throttle is considered stale and reported as invalid (32767) to the siliXcon.

## TX: siliXcon Control Input (`0x5FF`)

Sent every 50 ms (20 Hz). Follows the [siliXcon LYNX CAN Control Input](https://docs.silixcon.com/docs/fw/apps/esc/lynx/can/control_input) protocol. All values little-endian, DLC = 8 bytes. If not received within 200 ms the controller enters MODE 20 (CAN timeout).

| Byte | Type    | Description                  | siliXcon Input Param | Source                                               |
| ---- | ------- | ---------------------------- | -------------------- | ---------------------------------------------------- |
| 0-1  | INT_16  | CAN Level 1 — **Throttle**   | 1,255                | 0x407 ADC scaled 0→32766 (32767 = invalid/stale)     |
| 2-3  | INT_16  | CAN Level 2 — **Brake level**| 2,255                | Brake FSM: 0%→0, 25%→8191, 50%→16383, 100%→32766    |
| 4-5  | INT_16  | CAN Level 3 — *unused*       | 3,255                | 32767 (invalid)                                      |
| 6    | UINT_8  | Digital Inputs + Map switch  | see below            | See bit mapping                                      |
| 7    | UINT_8  | Commands                     | —                    | Bit 0 = disarm/seatswitch                            |

### Byte 6 — Digital inputs & map switching

| Bit | Signal            | siliXcon Input Param | Description                                |
| --- | ----------------- | -------------------- | ------------------------------------------ |
| 0   | Seat sensor       | 10,255               | 1 = seat occupied                          |
| 1   | Brake left        | 11,255               | 1 = left brake pressed                     |
| 2   | Brake right       | 12,255               | 1 = right brake pressed                    |
| 3   | Reverse           | 13,255               | 1 = drive FSM in reverse state             |
| 4   | Map switch (2x)   | 20,255               | Auto-pulse when mapping button is pressed  |
| 5-7 | Reserved          | —                    | 0                                          |

### Byte 7 — Commands

| Bit | Signal  | Description                              |
| --- | ------- | ---------------------------------------- |
| 0   | Disarm  | 1 = seat released → activates seatswitch |
| 1-7 | Reserved| 0                                        |

### Data flow summary

```
[Control board] --0x407 (throttle ADC)--> [Mainboard] --0x5FF--> [siliXcon]
                                             |
                  Brake left/right buttons --+-- CAN Level 1 = throttle
                  Reverse FSM state       --+-- CAN Level 2 = brake analog
                  Seat sensor             --+-- Byte 6 bits = digital in
                  Map button              --+-- Byte 7 bit 0 = disarm
```

# input conditionning
read the value from HAL and update the local state or event (ace rotatry and button generate directly the cleaned event too)
(just a list of function with simple conversion)

# output conditionning
apply the local state to the physical system (mosftet on / off / duty cycle, CAN broadcast at correct interval, PID computation)
(just a list of function with simple conversion)

# local state (ST)
this is a local and up to date copy of the local state used to compute the event. it also have the "history" of the last N state to be able to compute the EVENT on state change.

temperature
acceleration (lateral ,longitudinal, vertical, yaw, pitch, roll)
altitude
current consumtion at the mainboard input
current consuption for the heaters, defroster
output state
PID setpoint
PID current value
PID past value
FSM current state
timer value and state (running, stopped)

...

# Input event (EV)
this is our logic filter to explicitly define what condition are required to generate an event.
theses event should be "state less"/ not require information outside the ST table. if I need N sample to compute a derivate then I store the N last sample in the ST table.
it work like an interrupt bit list. Event can be set and are cleared after each cycle. 

button/switch pressed
button/switch released short press
button/switch releasd long press
button/switch releasd (dont care if it was long or short)
rotary increment / decrement
analog value exceed a threshold
can packet timeout
lateral acceleration exceed a threshold
longitudinal acceleration excced a threshold
current above threshold
timer timeout
composite event that arrise when some conditions are met.

## EVENT list
EV_SEATSENSOR_RELEASED
EV_SEATSENSOR_PRESSED
EV_DRIVING
EV_STATIONNARY

EV_VEHICLE_SPEED_ABOVE_5KPH
EV_VEHICLE_SPEED_BELOW_5KPH


EV_BTN_BROUILLARD_SHORT
EV_BTN_BROUILLARD_LONG
EV_FOG_ON_TIMER_TIMEOUT

EV_BTN_DEFROSTER_SHORT_AND_BATTERY_OK
EV_BTN_DEFROSTER_LONG_AND_BATTERY_OK
EV_BTN_DEFROSTER_SHORT
EV_BTN_DEFROSTER_LONG
EV_DEFROSTER_ON_TIMER_TIMEOUT
EV_DEFROSTER_FAULT_TIMER_TIMEOUT
EV_DEFROSTER_BLINK_TIMER_TIMEOUT
EV_DEFROSTER_ON_TIMER_RUNNING
EV_DEFROSTER_FAULT_TIMER_RUNNING
EV_DEFROSTER_BLINK_TIMER_RUNNING

EV_BTN_CABLIGHT_AND_STATIONNARY
EV_BTN_CABLIGHT_AND_DRIVING
EV_BTN_CABLIGHT_RELEASED
EV_CABINLIGHT_TIMER_LONG_TIMEOUT
EV_CABINLIGHT_TIMER_SHORT_TIMEOUT

EV_BTN_TRUNK_RELEASED_AND_STATIONNARY
EV_BTN_TRUNK_RELEASED_AND_DRIVING
EV_TRUNK_ACTION_TIMER_TIMEOUT
EV_TRUNK_BLINK_TIMER_TIMEOUT
EV_TRUNK_ERROR_TIMER_TIMEOUT

EV_BTN_FORWARD_SHORT
EV_BTN_FORWARD_LONG
EV_BTN_REVERSE_SHORT
EV_BTN_REVERSE_LONG
EV_DRIVE_CHANGE_TIMER_TIMEOUT

# State machines
they can only change state on INPUT event.
FSM state change only happen on event. no logic should be implemented there like event A and B should arrise. this is handled in the input event. but we can have multiple event that can trigger a state change (OR but no AND)

general rules about timer, if set while running, is restart again at the set time but will not generate more than ONE timeout event.

only ONE state machine can exist at most to change an output.
(for instance, for blinker and warning, we have 9 output (left + right blinker on the front, read, center of the vehicle, we have the visual feedback on the warning button, we have visual and audio feedback on the VDS display (left + right)) and thus all theses are controlled by only ONE FSM to explicit what happen and avoid edge case like for instance when the warning button is pressed but the blinker is also toggled and both try to interract with the blinker light at the same time.

## STATE list

ST_FSM_FOG_OFF | ST_FSM_FOG_ON | ST_FSM_FOG_ON_GRACE
ST_FSM_DEFROSTER_OFF | ST_FSM_DEFROSTER_ON | ST_FSM_DEFROSTER_FAULT_BLINK_OFF | ST_FSM_DEFROSTER_FAULT_BLINK_ON
ST_CABINLIGHT_OFF | ST_CABINLIGHT_ON | ST_CABINLIGHT_ON_TRUNK | ST_CABINLIGHT_ON_DRV
ST_TRUNK_LATCHED | ST_TRUNK_OPENING | ST_TRUNK_BLINK_ON | ST_TRUNK_BLINK_OFF

## GLOBAL finite state machines RULES (FSM DEFINITIONS)

* FSMs consume events from a **global event queue**
* Each FSM has **its own timer(s)**
  * If a timer is started while already running, it **restarts**
  * A timer **emits only ONE timeout event**
* `ST_` = current state
* `ST_FSM` = FSM state
* `EV_` = discrete event
* Transitions are **event-driven only**
* Outputs are applied in the **output stage**, we just update the state

---

## power on / off
when the VDS display button is pressed for N seconds, a message is sent on the CAN bus.
we must toggle the KEY/POWER pin of the controller when that message is received.

we are also responsible for turning it off if the button is long pressed or if the vehicle is IDLE for more than 60 minute

### States
```
ST_POWER_OFF      (default / safe)
ST_POWER_ON
ST_POWER_ON_GRACE
```

### Transitions

```
ST_POWER_OFF -> ST_POWER_ON
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN 
  
ST_POWER_ON -> ST_POWER_OFF
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN 

ST_POWER_ON -> ST_POWER_ON_GRACE
  on EV_SEATSENSOR_RELEASED
  start PWR_IDLE_TIMER (60 minute)

ST_POWER_ON_GRACE -> ST_POWER_OFF
  on EV_PWR_IDLE_TIMER_TIMEOUT

```

### Outputs

```
ST_POWER_OFF -> OUT_KEY_POWER_PIN = LOW
ST_POWER_ON  -> OUT_KEY_POWER_PIN = HIGH

```

---

## DRL
should turn on when the vehicle is powered on via CAN using the button on the VDS display.
only turn off when the vehicle is powered off via the button / after 1h timeout.

### States
```
ST_DRL_OFF      (default / safe)
ST_DRL_ON
ST_DRL_ON_GRACE
```

### Transitions

```
ST_DRL_OFF -> ST_DRL_ON
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN 

ST_DRL_ON -> ST_DRL_OFF
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN 

ST_DRL_ON -> ST_DRL_ON_GRACE
  on EV_SEATSENSOR_RELEASED
  start DRL_IDLE_TIMER (60 minute)

ST_DRL_ON_GRACE -> ST_DRL_OFF
  on EV_DRL_IDLE_TIMER_TIMEOUT
```

### Outputs

```
ST_DRL_OFF:
  CAN_DRL_LEFT  = OFF
  CAN_DRL_RIGHT = OFF

ST_DRL_ON:
  CAN_DRL_LEFT  = ON
  CAN_DRL_RIGHT = ON
```

---

## low beam
should turn on when the vehicle is driving via CAN using the button on the VDS display.
only turn off when the vehicle is stationnary for more than 5 minute or that the VDS display button turn of the vehicle

### States
```
ST_LOWBEAM_OFF      (default / safe)
ST_LOWBEAM_ON
ST_LOWBEAM_ON_GRACE
```

### Transitions

```
ST_LOWBEAM_OFF -> ST_LOWBEAM_ON
  on EV_DRIVING


ST_LOWBEAM_ON -> ST_LOWBEAM_OFF
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN


ST_LOWBEAM_ON -> ST_LOWBEAM_ON_GRACE
  on EV_STATIONNARY
  start LOWBEAM_IDLE_TIMER (5 minute)


ST_LOWBEAM_ON_GRACE -> ST_LOWBEAM_ON
  on EV_DRIVING


ST_LOWBEAM_ON_GRACE -> ST_LOWBEAM_OFF
  on EV_LOWBEAM_IDLE_TIMER_TIMEOUT


ST_LOWBEAM_ON_GRACE -> ST_LOWBEAM_OFF
  on EV_POWER_BUTTON_SHORT_CAN
  on EV_POWER_BUTTON_LONG_CAN
```

### Outputs

```
ST_LOWBEAM_OFF:
  CAN_LOWBEAM_LEFT  = OFF
  CAN_LOWBEAM_RIGHT = OFF

ST_LOWBEAM_ON:
  CAN_LOWBEAM_LEFT  = ON
  CAN_LOWBEAM_RIGHT = ON

ST_LOWBEAM_ON_GRACE:
  CAN_LOWBEAM_LEFT  = ON
  CAN_LOWBEAM_RIGHT = ON
```

---

## FOG LIGHT FSM
on as un bouton qui nous permet d'allumer/éteidne le phare a brouillard. Il s'agit d'un simple bouton et chaque pression change l'état de on à off.
Le bouton possede une LED qui indique l'etat actuel.
l'etat est partagé via CAN pour les lightboard qui allume les lumières exterieur.
on lis le seat sensor avec un timeout a 2 minute si on quitte le vehicle en oubliant le fog light, il seteint tout seul.

### States

```
ST_FSM_FOG_OFF        (default / safe)
ST_FSM_FOG_ON
ST_FSM_FOG_ON_GRACE
```

### Transitions

```
ST_FSM_FOG_OFF -> ST_FSM_FOG_ON
  on EV_BTN_BROUILLARD_SHORT
  on EV_BTN_BROUILLARD_LONG

ST_FSM_FOG_ON -> ST_FSM_FOG_ON_GRACE
  on EV_SEATSENSOR_RELEASED
  start FOG_ON_TIMER (2 min)

ST_FSM_FOG_ON_GRACE -> ST_FSM_FOG_ON
  on EV_SEATSENSOR_PRESSED

ST_FSM_FOG_ON_GRACE -> ST_FSM_FOG_OFF
  on EV_FOG_ON_TIMER_TIMEOUT

ST_FSM_FOG_ON -> ST_FSM_FOG_OFF
  on EV_BTN_BROUILLARD_SHORT
  on EV_BTN_BROUILLARD_LONG
```

### Outputs

on envoie sur le can periodiquement l'etat. on allume / eteint la led sur le bouton.

---

## DEFROSTER FSM

Le defroster s'allume pendant 5 minute quand le bouton est pressé, la led du bouton s'allume aussi.
si le bouton est pressé pendant que le defroster est actif, alors on arrete le defroster.
si la batterie est presque vide, le defroster ne s'allume pas/tombe en etat off et le bouton clignote qq fois rapidement pendat 3 secondes.

### States

```
ST_FSM_DEFROSTER_OFF        (default / safe)
ST_FSM_DEFROSTER_ON
ST_FSM_DEFROSTER_FAULT_BLINK_OFF
ST_FSM_DEFROSTER_FAULT_BLINK_ON
```

### Transitions

```
ST_FSM_DEFROSTER_OFF -> ST_FSM_DEFROSTER_ON
  on EV_BTN_DEFROSTER_SHORT_AND_BATTERY_OK
  on EV_BTN_DEFROSTER_LONG_AND_BATTERY_OK
  start DEFROSTER_ON_TIMER (5 min)

ST_FSM_DEFROSTER_ON -> ST_FSM_DEFROSTER_OFF
  on EV_BTN_DEFROSTER_SHORT
  on EV_BTN_DEFROSTER_LONG
  on EV_DEFROSTER_ON_TIMER_TIMEOUT

ST_FSM_DEFROSTER_ON -> ST_FSM_DEFROSTER_FAULT_BLINK_ON
  on EV_BATTERY_BELOW_30_PERCENT
  on EV_DEFROSTER_FAULT
  start DEFROSTER_FAULT_TIMER (3 sec)
  start DEFROSTER_BLINK_TIMER (0.4 sec)

ST_FSM_DEFROSTER_FAULT_BLINK_ON -> ST_FSM_DEFROSTER_FAULT_BLINK_OFF
  on EV_DEFROSTER_BLINK_TIMER_TIMEOUT
  start DEFROSTER_BLINK_TIMER (0.4 sec)

ST_FSM_DEFROSTER_FAULT_BLINK_OFF -> ST_FSM_DEFROSTER_FAULT_BLINK_ON
  on EV_DEFROSTER_BLINK_TIMER_TIMEOUT
  start DEFROSTER_BLINK_TIMER (0.4 sec)

ST_FSM_DEFROSTER_FAULT_BLINK_ON -> ST_FSM_DEFROSTER_OFF
  on EV_DEFROSTER_FAULT_TIMER_TIMEOUT

ST_FSM_DEFROSTER_FAULT_BLINK_OFF -> ST_FSM_DEFROSTER_OFF
  on EV_DEFROSTER_FAULT_TIMER_TIMEOUT


```

### Outputs

```
ST_DEFROSTER_OFF         : OUT_DEFROSTER_MOSFET = OFF, LED OFF
ST_DEFROSTER_ON          : OUT_DEFROSTER_MOSFET = ON,  LED ON
ST_FSM_DEFROSTER_FAULT_BLINK_OFF : OUT_DEFROSTER_MOSFET = OFF, LED OFF
ST_FSM_DEFROSTER_FAULT_BLINK_ON : OUT_DEFROSTER_MOSFET = OFF, LED ON
```

---

## CABIN LIGHT FSM
the light stay on for 1h when the vehicle is stationnary. pressing the button again toggle back the state to off.
the light turn off if the vehicle speed goes above 5kmh
if the light is turned on / button pressed while speed is above 5kmh. we keep the  light on as long as the speed is above 5kmh.
once we cross bellow 5kmh / stationnary, we have a 1h timeout. If the vehicle restart after that, the light turn off.
pressing the trunk button while stationnary will also turn on the light for a small delay of 5 minute. (if not stationnary, the command is ignored as we dont want to open the trunk while driving.)

### States

```
ST_CABINLIGHT_OFF        (default)
ST_CABINLIGHT_ON
ST_CABINLIGHT_ON_TRUNK
ST_CABINLIGHT_ON_DRV
```

### Transitions

```
ST_CABINLIGHT_OFF -> ST_CABINLIGHT_ON
  on EV_BTN_CABLIGHT_AND_STATIONNARY
  start CABINLIGHT_TIMER_LONG (60 min)

ST_CABINLIGHT_OFF -> ST_CABINLIGHT_ON_DRV
  on EV_BTN_CABLIGHT_AND_DRIVING

ST_CABINLIGHT_OFF -> ST_CABINLIGHT_ON_TRUNK
  on EV_BTN_TRUNK_AND_STATIONNARY
  start CABINLIGHT_TIMER_SHORT (5 min)

ST_CABINLIGHT_ON -> ST_CABINLIGHT_OFF
  on EV_BTN_CABLIGHT_RELEASED
  on EV_CABINLIGHT_TIMER_LONG_TIMEOUT
  on EV_DRIVING

ST_CABINLIGHT_ON_DRV -> ST_CABINLIGHT_ON
  on EV_STATIONNARY

ST_CABINLIGHT_ON_DRV -> ST_CABINLIGHT_OFF
  on EV_BTN_CABLIGHT_RELEASED

ST_CABINLIGHT_ON_TRUNK -> ST_CABINLIGHT_OFF
  on EV_BTN_CABLIGHT_RELEASED
  on EV_CABINLIGHT_TIMER_SHORT_TIMEOUT
```

### Outputs

```
ST_CABINLIGHT_OFF,      cablight and led feedback are OFF
ST_CABINLIGHT_ON,       cablight and led feedback are ON
ST_CABINLIGHT_ON_TRUNK, cablight and led feedback are ON
ST_CABINLIGHT_ON_DRV,   cablight and led feedback are ON
```

---

## TRUNK FSM
we can press the TRUNK button. if we do so while the vehicle is driving, the trunk will not open and the button will blink. otherwise, the button will be powered on for a few seconds while the latch release the trunk.

### States

```
ST_TRUNK_LATCHED        (default)
ST_TRUNK_OPENING
ST_TRUNK_BLINK_ON
ST_TRUNK_BLINK_OFF
```

### Transitions

```
ST_TRUNK_LATCHED -> ST_TRUNK_OPENING
  on EV_BTN_TRUNK_RELEASED_AND_STATIONNARY
  start TRUNK_ACTION_TIMER (2 sec)

ST_TRUNK_OPENING -> ST_TRUNK_LATCHED 
  on EV_TRUNK_ACTION_TIMER_TIMEOUT

ST_TRUNK_LATCHED -> ST_TRUNK_BLINK_ON
  on EV_BTN_TRUNK_RELEASED_AND_DRIVING
  start TRUNK_ERROR_TIMER (3 sec)
  start TRUNK_BLINK_TIMER (0.4 sec)

ST_TRUNK_BLINK_ON -> ST_TRUNK_BLINK_OFF
  on EV_TRUNK_BLINK_TIMER_TIMEOUT
  start TRUNK_BLINK_TIMER (0.4 sec)

ST_TRUNK_BLINK_OFF -> ST_TRUNK_BLINK_ON
  on EV_TRUNK_BLINK_TIMER_TIMEOUT
  start TRUNK_BLINK_TIMER (0.4 sec)

ST_TRUNK_BLINK_ON -> ST_TRUNK_LATCHED
  on EV_TRUNK_ERROR_TIMER_TIMEOUT

ST_TRUNK_BLINK_OFF -> ST_TRUNK_LATCHED
  on EV_TRUNK_ERROR_TIMER_TIMEOUT

```
### Outputs

```
ST_TRUNK_LATCHED,   coil is not powered, feedback LED is also off
ST_TRUNK_OPENING,   coil is powered, feedback LED is on
ST_TRUNK_BLINK_ON,  coil is not powered, feedback LED is on
ST_TRUNK_BLINK_OFF, coil is not powered, feedback LED is off
```

---

## BLINKER AND WARNING FSM
when the warning button is pressed, both side blinker should blink at 4Hz, we press it again to turn off the warnings.
when left / right blinker is toggled, we blink at 75 bpm
when in warning, the warning button should blink and the binker feedback on the display + rythmic beeping.
when the left / right blinker are active, only the correct blinker feedback should blink + rythmic beeping.
when the seat sensor is released, the blinker should stop but the warning button should keep on until the battery run out.

### States

```
ST_BLINKER_IDLE (default)
ST_WARNING_ON
ST_WARNING_OFF
ST_BLINKER_RIGHT_OFF
ST_BLINKER_LEFT_OFF
ST_BLINKER_RIGHT_ON
ST_BLINKER_LEFT_ON
```

### Transitions

```
ST_BLINKER_IDLE -> ST_BLINKER_LEFT_ON
  on EV_BTN_BLINKER_LEFT_RELEASED
  start BLINKER_CYCLE_TIMER (400 ms)

ST_BLINKER_IDLE -> ST_BLINKER_RIGHT_ON
  on EV_BTN_BLINKER_RIGHT_RELEASED
  start BLINKER_CYCLE_TIMER (400 ms)

ST_BLINKER_IDLE -> ST_WARNING_ON
  on EV_BTN_WARNING_RELEASED
  start WARNING_CYCLE_TIMER (125 ms)


ST_BLINKER_LEFT_ON -> ST_BLINKER_LEFT_OFF
  on EV_BLINKER_CYCLE_TIMEOUT
  start BLINKER_CYCLE_TIMER

ST_BLINKER_LEFT_OFF -> ST_BLINKER_LEFT_ON
  on EV_BLINKER_CYCLE_TIMEOUT
  start BLINKER_CYCLE_TIMER

ST_BLINKER_LEFT_ON -> ST_BLINKER_IDLE
  on EV_BTN_BLINKER_LEFT_RELEASED

ST_BLINKER_LEFT_OFF -> ST_BLINKER_IDLE
  on EV_BTN_BLINKER_LEFT_RELEASED

ST_BLINKER_LEFT_ON -> ST_WARNING_ON
  on EV_BTN_WARNING_RELEASED
  start WARNING_CYCLE_TIMER

ST_BLINKER_LEFT_OFF -> ST_WARNING_ON
  on EV_BTN_WARNING_RELEASED
  start WARNING_CYCLE_TIMER

ST_BLINKER_LEFT_ON -> ST_BLINKER_IDLE
  on EV_SEATSENSOR_RELEASED

ST_BLINKER_LEFT_OFF -> ST_BLINKER_IDLE
  on EV_SEATSENSOR_RELEASED


ST_BLINKER_RIGHT_ON -> ST_BLINKER_RIGHT_OFF
  on EV_BLINKER_CYCLE_TIMEOUT
  start BLINKER_CYCLE_TIMER

ST_BLINKER_RIGHT_OFF -> ST_BLINKER_RIGHT_ON
  on EV_BLINKER_CYCLE_TIMEOUT
  start BLINKER_CYCLE_TIMER

ST_BLINKER_RIGHT_ON -> ST_BLINKER_IDLE
  on EV_BTN_BLINKER_RIGHT_RELEASED

ST_BLINKER_RIGHT_OFF -> ST_BLINKER_IDLE
  on EV_BTN_BLINKER_RIGHT_RELEASED

ST_BLINKER_RIGHT_ON -> ST_WARNING_ON
  on EV_BTN_WARNING_RELEASED
  start WARNING_CYCLE_TIMER

ST_BLINKER_RIGHT_OFF -> ST_WARNING_ON
  on EV_BTN_WARNING_RELEASED
  start WARNING_CYCLE_TIMER

ST_BLINKER_RIGHT_ON -> ST_BLINKER_IDLE
  on EV_SEATSENSOR_RELEASED

ST_BLINKER_RIGHT_OFF -> ST_BLINKER_IDLE
  on EV_SEATSENSOR_RELEASED


ST_WARNING_ON -> ST_WARNING_OFF
  on EV_WARNING_CYCLE_TIMEOUT
  start WARNING_CYCLE_TIMER

ST_WARNING_OFF -> ST_WARNING_ON
  on EV_WARNING_CYCLE_TIMEOUT
  start WARNING_CYCLE_TIMER

ST_WARNING_ON -> ST_BLINKER_IDLE
  on EV_BTN_WARNING_RELEASED

ST_WARNING_OFF -> ST_BLINKER_IDLE
  on EV_BTN_WARNING_RELEASED

```
### Outputs

```
ST_BLINKER_IDLE (default) -> all off
ST_WARNING_ON -> CAN left + right and led left + right + warning led feedback + display VSD feedback are ON, audio feedback too
ST_WARNING_OFF -> CAN left + right and led left + right + warning led feedback + display VSD feedback are OFF, audio feedback OFF
ST_BLINKER_RIGHT_OFF -> CAN left + right and led left + right + warning led feedback + display VSD feedback are OFF, audio feedback OFF
ST_BLINKER_LEFT_OFF  -> CAN left + right and led left + right + warning led feedback + display VSD feedback are OFF, audio feedback OFF
ST_BLINKER_RIGHT_ON  -> CAN right and led right + display VSD feedback right are ON, audio feedback ON
ST_BLINKER_LEFT_ON   -> CAN left and led left + display VSD feedback left are ON, audio feedback ON
```

---

## DRIVE DIRECTION FSM (FORWARD / REVERSE)
using a non latching switch we can select forward / backward.
I want to be able to select freely forward / backward as long as the speed is bellow a given threshold speed (5kmh)
If i'm above and I toggle, I want my toggle to be valid for 5 second (if the speed drop bellow the threshold in that time frame we count it otherwise, we ignore it.)

### States

```
ST_DRIVE_NEUTRAL          (default / safe)

ST_DRIVE_FORWARD
ST_DRIVE_FORWARD_LOCK
ST_DRIVE_FWD_PENDING

ST_DRIVE_REVERSE
ST_DRIVE_REVERSE_LOCK
ST_DRIVE_REV_PENDING

```

### Transitions

```
ST_DRIVE_NEUTRAL -> ST_DRIVE_FORWARD
  on EV_BTN_FORWARD_SHORT
  on EV_BTN_FORWARD_LONG

ST_DRIVE_NEUTRAL -> ST_DRIVE_REVERSE
  on EV_BTN_REVERSE_SHORT
  on EV_BTN_REVERSE_LONG


ST_DRIVE_FORWARD -> ST_DRIVE_FORWARD_LOCK
  on EV_VEHICLE_SPEED_ABOVE_5KPH

ST_DRIVE_FORWARD -> ST_DRIVE_REVERSE
  on EV_BTN_REVERSE_SHORT
  on EV_BTN_REVERSE_LONG

ST_DRIVE_FORWARD -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


ST_DRIVE_FORWARD_LOCK -> ST_DRIVE_FORWARD
  on EV_VEHICLE_SPEED_BELOW_5KPH

ST_DRIVE_FORWARD_LOCK -> ST_DRIVE_REV_PENDING
  on EV_BTN_REVERSE_SHORT
  on EV_BTN_REVERSE_LONG
  start DRIVE_CHANGE_TIMER (5 sec)

ST_DRIVE_FORWARD_LOCK -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


ST_DRIVE_FWD_PENDING -> ST_DRIVE_FORWARD
  on EV_VEHICLE_SPEED_BELOW_5KPH

ST_DRIVE_FWD_PENDING -> ST_DRIVE_REVERSE_LOCK
  on EV_DRIVE_CHANGE_TIMER_TIMEOUT

ST_DRIVE_FWD_PENDING -> ST_DRIVE_REVERSE_LOCK
  on EV_BTN_REVERSE_SHORT
  on EV_BTN_REVERSE_LONG

ST_DRIVE_FWD_PENDING -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


ST_DRIVE_REVERSE -> ST_DRIVE_REVERSE_LOCK
  on EV_VEHICLE_SPEED_ABOVE_5KPH

ST_DRIVE_REVERSE -> ST_DRIVE_FORWARD
  on EV_BTN_FORWARD_SHORT
  on EV_BTN_FORWARD_LONG

ST_DRIVE_REVERSE -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


ST_DRIVE_REVERSE_LOCK -> ST_DRIVE_REVERSE
  on EV_VEHICLE_SPEED_BELOW_5KPH

ST_DRIVE_REVERSE_LOCK -> ST_DRIVE_FWD_PENDING
  on EV_BTN_FORWARD_SHORT
  on EV_BTN_FORWARD_LONG
  start DRIVE_CHANGE_TIMER (5 sec)

ST_DRIVE_REVERSE_LOCK -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


ST_DRIVE_REV_PENDING -> ST_DRIVE_REVERSE
  on EV_VEHICLE_SPEED_BELOW_5KPH

ST_DRIVE_REV_PENDING -> ST_DRIVE_FORWARD_LOCK
  on EV_DRIVE_CHANGE_TIMER_TIMEOUT

ST_DRIVE_REV_PENDING -> ST_DRIVE_FORWARD_LOCK
  on EV_BTN_FORWARD_SHORT
  on EV_BTN_FORWARD_LONG

ST_DRIVE_REV_PENDING -> ST_DRIVE_NEUTRAL
  on EV_SEATSENSOR_RELEASED


```

### Outputs

```

ST_DRIVE_FORWARD   ->  send CAN to forward motion
ST_DRIVE_REV_PENDING   -> send CAN to forward motion
ST_DRIVE_FORWARD_LOCK  -> send CAN to forward motion

ST_DRIVE_NEUTRAL -> send CAN to neutral

ST_DRIVE_REVERSE   -> send CAN to reverse motion
ST_DRIVE_FWD_PENDING   ->  send CAN to reverse motion
ST_DRIVE_REVERSE_LOCK  -> send CAN to reverse motion
```

---

## BRAKE FSM
when the brake switch are pressed we can influence the regen braking.
by default when no brake is pressed, there is no regen
when the left brake is pressed, we have 25% of the regen.
when the right brake is pressed, we have 50% of the regen.
when both are pressed, we have 100% of the regen braking.

we need to implement a ramp up on the regen braking on the silixcon to make it smooth (and not go from 0 to 100% instantly)

we also need to take this CAN state on the lighboard to light up the brake signal.

### States

```
ST_BRAKE_LEVEL_0
ST_BRAKE_LEVEL_25
ST_BRAKE_LEVEL_50
ST_BRAKE_LEVEL_100
```

### Transitions

```
ST_BRAKE_LEVEL_0 -> ST_BRAKE_LEVEL_0
  on EV_BRAKE_NONE

ST_BRAKE_LEVEL_0 -> ST_BRAKE_LEVEL_25
  on EV_BRAKE_LEFT_ONLY

ST_BRAKE_LEVEL_0 -> ST_BRAKE_LEVEL_50
  on EV_BRAKE_RIGHT_ONLY

ST_BRAKE_LEVEL_0 -> ST_BRAKE_LEVEL_100
  on EV_BRAKE_BOTH


ST_BRAKE_LEVEL_25 -> ST_BRAKE_LEVEL_0
  on EV_BRAKE_NONE

ST_BRAKE_LEVEL_25 -> ST_BRAKE_LEVEL_25
  on EV_BRAKE_LEFT_ONLY

ST_BRAKE_LEVEL_25 -> ST_BRAKE_LEVEL_50
  on EV_BRAKE_RIGHT_ONLY

ST_BRAKE_LEVEL_25 -> ST_BRAKE_LEVEL_100
  on EV_BRAKE_BOTH


ST_BRAKE_LEVEL_50 -> ST_BRAKE_LEVEL_0
  on EV_BRAKE_NONE

ST_BRAKE_LEVEL_50 -> ST_BRAKE_LEVEL_25
  on EV_BRAKE_LEFT_ONLY

ST_BRAKE_LEVEL_50 -> ST_BRAKE_LEVEL_50
  on EV_BRAKE_RIGHT_ONLY

ST_BRAKE_LEVEL_50 -> ST_BRAKE_LEVEL_100
  on EV_BRAKE_BOTH


ST_BRAKE_LEVEL_100 -> ST_BRAKE_LEVEL_0
  on EV_BRAKE_NONE

ST_BRAKE_LEVEL_100 -> ST_BRAKE_LEVEL_25
  on EV_BRAKE_LEFT_ONLY

ST_BRAKE_LEVEL_100 -> ST_BRAKE_LEVEL_50
  on EV_BRAKE_RIGHT_ONLY

ST_BRAKE_LEVEL_100 -> ST_BRAKE_LEVEL_100
  on EV_BRAKE_BOTH
```

### Outputs

```
ST_BRAKE_LEVEL_0    -> regen_setpoint = 0%
ST_BRAKE_LEVEL_25   -> regen_setpoint = 25%
ST_BRAKE_LEVEL_50   -> regen_setpoint = 50%
ST_BRAKE_LEVEL_100  -> regen_setpoint = 100%
```

---

## USB POWER FSM

### States

```
ST_USB_OFF     (default)
ST_USB_POWERED
ST_USB_GRACE
```

### Transitions

```
ST_USB_OFF -> ST_USB_POWERED
  on EV_SEAT_PRESSED

ST_USB_POWERED -> ST_USB_GRACE
  on EV_SEAT_RELEASED_10S
  start USB_GRACE_TIMER (30 min)

ST_USB_GRACE -> ST_USB_OFF
  on EV_USB_GRACE_TIMEOUT

ST_USB_GRACE -> ST_USB_POWERED
  on EV_SEAT_PRESSED
```
### Outputs

---

## AUDIO OUTPUT

### States

```
ST_AUDIOAUX_OFF     (default)
ST_AUDIOAUX_POWERED
ST_AUDIOAUX_GRACE
```

### Transitions

```
ST_USB_OFF -> ST_USB_POWERED
  on EV_SEAT_PRESSED

ST_USB_POWERED -> ST_USB_GRACE
  on EV_SEAT_RELEASED
  start USB_GRACE_TIMER (5 sec)

ST_USB_GRACE -> ST_USB_OFF
  on EV_USB_GRACE_TIMEOUT

ST_USB_GRACE -> ST_USB_POWERED
  on EV_SEAT_PRESSED
```
### Outputs

---


## MAPPING BUTTON FSM

### States

```
ST_MAPPING_CLEAR   (default)
ST_MAPPING_PRESSED
```

### Transitions

```
ST_MAPPING_CLEAR -> ST_MAPPING_PRESSED
  on EV_BTN_MAPPING_SHORT
  on EV_BTN_MAPPING_LONG
  start MAPPING_BUTTON_TIMER (2 sec)

ST_MAPPING_PRESSED -> ST_MAPPING_CLEAR
  on EV_MAPPING_BUTTON_TIMEOUT
```
### Outputs

---

## SEAT HEATER FSM

### States

```
ST_SEATHEATER_STANDBY   (default)
ST_SEATHEATER_EDITION
ST_SEATHEATER_ON
```

### Transitions

```
ST_SEATHEATER_STANDBY -> ST_SEATHEATER_EDITION
  on EV_BTN_SEATHEATER_LONG
  start SEATHEATER_EDIT_TIMER (10 sec)

ST_SEATHEATER_EDITION -> ST_SEATHEATER_ON
  on EV_BTN_SEATHEATER_LONG
  start SEATHEATER_ON_TIMER (30 min)

ST_SEATHEATER_EDITION -> ST_SEATHEATER_STANDBY
  on EV_SEATHEATER_EDIT_TIMEOUT

ST_SEATHEATER_STANDBY -> ST_SEATHEATER_ON
  on EV_BTN_SEATHEATER_SHORT
  start SEATHEATER_ON_TIMER (30 min)

ST_SEATHEATER_ON -> ST_SEATHEATER_STANDBY
  on EV_BTN_SEATHEATER_SHORT
  on EV_SEATHEATER_ON_TIMEOUT
```

### Outputs

---

## HAND HEATER FSM

### States

```
ST_HANDHEATER_STANDBY   (default)
ST_HANDHEATER_EDITION
ST_HANDHEATER_ON
```

### Transitions

```
ST_HANDHEATER_STANDBY -> ST_HANDHEATER_EDITION
  on EV_BTN_HANDHEATER_LONG
  start HANDHEATER_EDIT_TIMER (10 sec)

ST_HANDHEATER_ON -> ST_HANDHEATER_EDITION
  on EV_BTN_HANDHEATER_LONG
  start HANDHEATER_EDIT_TIMER (10 sec)

ST_HANDHEATER_EDITION -> ST_HANDHEATER_ON
  on EV_BTN_HANDHEATER_LONG
  start HANDHEATER_ON_TIMER (30 min)

ST_HANDHEATER_EDITION -> ST_HANDHEATER_STANDBY
  on EV_HANDHEATER_EDIT_TIMEOUT

ST_HANDHEATER_STANDBY -> ST_HANDHEATER_ON
  on EV_BTN_HANDHEATER_SHORT
  start HANDHEATER_ON_TIMER (30 min)

ST_HANDHEATER_ON -> ST_HANDHEATER_STANDBY
  on EV_BTN_HANDHEATER_SHORT
  on EV_HANDHEATER_ON_TIMEOUT

ST_HANDHEATER_EDITION -> ST_HANDHEATER_EDITION
  on EV_ROT_HANDHEATER_INC
  on EV_ROT_HANDHEATER_DEC
```
### Outputs


---

## SPEED LIMITER FSM

### States

```
ST_SPEED_LIMIT_EDITION   (default)
ST_ASSIST_COEFF_EDITION
```

### Transitions

```
ST_SPEED_LIMIT_EDITION -> ST_ASSIST_COEFF_EDITION
  on EV_BTN_SPEEDLIMIT_SHORT

ST_ASSIST_COEFF_EDITION -> ST_SPEED_LIMIT_EDITION
  on EV_BTN_SPEEDLIMIT_SHORT
```
### Outputs


---


### RGB backlight
mapping affect the whole RGB state.
when setting the temperature too.

STATE MAPPING_1
STATE MAPPING_2
STATE MAPPING_3
STATE MAPPING_4
STATE MAPPING_5

STATE TEMPERATURE_LOW
STATE TEMPERATURE_MEDIUM
STATE TEMPERATURE_WARM
STATE TEMPERATURE_HOT
STATE TEMPERATURE_INFERNO

## CAN Communication

This firmware supports CAN communication for broadcasting and receiving control inputs. Below are the details of the CAN messages implemented:

### Brake Status
- **Digital Brake Status**: Broadcasts the on/off status for left, right, and any brake (3 bits).
- **Analog Brake Level**: Broadcasts the computed braking level as a percentage (0%, 25%, 50%, 100%).

### Throttle
- **Throttle Input**: Broadcasts the throttle value received from the input device.

### Reverse Bit
- **Reverse Configuration**: Broadcasts the reverse bit to configure the siliXcon controller to use this as an input.

These messages are designed to integrate seamlessly with the siliXcon LYNX firmware, ensuring compatibility and efficient communication.

### CAN ID and Bit Mapping

#### CAN IDs
- **Brake Status**: 0x100
- **Throttle**: 0x101
- **Reverse Bit**: 0x102
- **Control Message**: 0x5FF

#### Bit Mapping
- **Brake Status (0x100)**:
  - Bit 0: Left brake (on/off)
  - Bit 1: Right brake (on/off)
  - Bit 2: Any brake (on/off)
  - Bits 3-7: Reserved
  - Byte 1: Analog brake level (0-100%)

- **Throttle (0x101)**:
  - Byte 0: Throttle value (0-255)
  - Bytes 1-7: Reserved

- **Reverse Bit (0x102)**:
  - Bit 0: Reverse status (on/off)
  - Bits 1-7: Reserved
  - Bytes 1-7: Reserved

While we broadcast the status on the can with the previous ID, we also repack theses value in the following package for the silixcon controller to use.
We thus pack : The throttle (analog), the brake level (analog, 0,25,50,100 %), and as digital inputs :  the reverse, the brake right + brake left, the seat sensor, the mapping button press


- **Control Message (0x5FF)**:
  - **Message Requirements**:
    - Timeout: 200ms
    - Continuous presence required
    - Timeout triggers MODE 20 (CAN timeout)
  
  - **Invalid Data Handling**:
    - For INT_16 values:
      - 32767 indicates invalid reading
      - On invalid input, either use this value or stop message transmission

  - **Message Structure**:
    - Byte 0-1: INT_16, CAN Level 1 (1,255)
    - Byte 2-3: INT_16, CAN Level 2 (2,255)
    - Byte 4-5: INT_16, CAN Level 3 (3,255)
    - Byte 6: UINT_8, Digital Inputs:
      - Bit 0: Digital in 0
      - Bit 1: Digital in 1
      - Bit 2: Digital in 2
      - Bit 3: Digital in 3
      - Digital inputs: 10,255; 11,255; 12,255; 13,255
      - Map switching: 20,255; 21,255; 22,255; 23,255
    - Byte 7: UINT_8, Commands:
      - Bit 0: Disarm - Activates seatswitch mode

  - **Map Switching Behavior**:
    - 2x values: Auto-generates pulse on each message
    - 1x values: Requires pulse completion

This mapping ensures compatibility with the siliXcon LYNX firmware and adheres to the fixed DLC of 8 bytes for all messages.

