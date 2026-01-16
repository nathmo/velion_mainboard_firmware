# velion_mainboard_firmware
This is the firmware that control the main board and control pannel of the vehicle.

# General logic
Use ESP32 FreeRTOS on Arduino  IDE

there is a mainloop which first read the input, generate event. then iterrate over all state machine and feed them the first event untill all event are consumed. then there the output stage which run the PID step and the output monitoring.

INPUTS Loop (1kHz mini)
Call AceButton and AceRotary to handle buttons and give event

LOGIC FSMs
execute each FSM with the event queue

OUTPUT 
PID V1 

# Pin mapping 
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

I also have a GY-91 with MPU-9250 and BMP280 as a 10 DOF IMU. address for the  MPU9250 shoud led 1101000 = 0x68, BMP : 1110110 = 0x76


# Input event
button/switch pressed
button/switch released short press
button/switch releasd long press
rotary increment / decrement
analog value exceed a threshold
can packet
lateral acceleration exceed a threshold
longitudinal acceleration excced a threshold

# State machines
general rules about timer, if set while running, is start again at the set time but will not generate more than ONE timeout event.
we should have an event queue and a STATE table. the state table is for global state that must be present or absent for something to happen with an event. the event are for every state change.

### Fog light
this is driven by a non-latching switch
there is the visual feedback to turn on/off
there is the can code to send for the dispaly feedback. (same used by the rear ligtning board)

STATE FOG_OFF (default) -> no can code output, not visual feedback

STATE FOG_ON -> send can code, send visual feedback

Transition from FOG_OFF to FOG_ON : button fog released short press
Transition from FOG_OFF to FOG_ON : button fog releasd long press

Transition from FOG_ON to FOG_OFF : button fog released short press
Transition from FOG_ON to FOG_OFF : button fog releasd long press

extra logic with the seat sensor ? make a bip ? turn off after a timeout ?
The "grand phare" are not the same and controlled by shortpress (appel de phare) / long press ( toggle on / off) on the smartswitch. 

### Defroster
this is driven by a NON-latching switch
there is the visual feedback to turn on/off
there is the heater to turn on
there is a timmer to start

STATE DEFROSTER_OFF (default) -> all is off

STATE DEFROSTER_ON -> visual feedback is on, mosftet is ON. a DEFROSTER_TIMEOUT is started with timeout of 10 min

STATE DEFROSTER_FAULT_BLINK -> visual feedback is Blinking, mosftet is OFF. a DEFROSTER_FAULT_TIMEOUT is started with timeout of 4 seconds

Transition from DEFROSTER_OFF to DEFROSTER_ON : button defroster released short press
Transition from DEFROSTER_OFF to DEFROSTER_ON : button defroster releasd long press

Transition from DEFROSTER_ON to DEFROSTER_OFF : button defroster released short press
Transition from DEFROSTER_ON to DEFROSTER_OFF : button defroster releasd long press
Transition from DEFROSTER_ON to DEFROSTER_OFF : DEFROSTER_TIMEOUT event

Transition from DEFROSTER_ON to DEFROSTER_FAULT_BLINK : BATTERY LEVEL bellow 30 % event -> Should make the visual feedback blink for a few seconds
Transition from DEFROSTER_ON to DEFROSTER_FAULT_BLINK : DEFROSTER_ERROR (shortcut, no current drawn, ...) -> Should make the visual feedback blink for a few seconds

Transition from DEFROSTER_FAULT_BLINK to DEFROSTER_OFF : DEFROSTER_FAULT_TIMEOUT we are done blinking and just turn off everything

### Interrior ligthning
this is driven by a NON-latching switch
there is the visual feedback to turn on/off
there is the interrior light to turn on
there is a timmer to start

STATE CABINLIGHT_OFF (default) -> all is off
STATE CABINLIGHT_ON -> visual feedback is on, mosftet is ON. a CABINLIGHT_TIMEOUT is started with timeout of 60 min
STATE CABINLIGHT_ON_TRUNK -> visual feedback is on, mosftet is ON. a CABINLIGHT_TIMEOUT is started with timeout of 5 min

Transition from CABINLIGHT_OFF to CABINLIGHT_ON : button cabinlight released short press
Transition from CABINLIGHT_OFF to CABINLIGHT_ON : button cabinlight releasd long press
Transition from CABINLIGHT_OFF to CABINLIGHT_ON_TRUNK : button TRUNKlight released short press
Transition from CABINLIGHT_OFF to CABINLIGHT_ON_TRUNK : button TRUNKlight releasd long press


Transition from CABINLIGHT_ON to CABINLIGHT_OFF : button cabinlight released short press
Transition from CABINLIGHT_ON to CABINLIGHT_OFF : button cabinlight releasd long press
Transition from CABINLIGHT_ON to CABINLIGHT_OFF : CABINLIGHT_TIMEOUT event

Transition from CABINLIGHT_ON_TRUNK to CABINLIGHT_OFF : button cabinlight released short press
Transition from CABINLIGHT_ON_TRUNK to CABINLIGHT_OFF : button cabinlight releasd long press
Transition from CABINLIGHT_ON_TRUNK to CABINLIGHT_OFF : CABINLIGHT_TIMEOUT event

the interrior light up if one open the trunk. it can be turned off directly or wait for the shorter timeout.
the interrior light up if one chosse to do so but it will not stay on for more than 60 min in case its forgotten ON.

### Trunk latch
this is driven by a NON-latching switch
there is the visual feedback to turn on/off
there is the trunk latch to power on
there is a timmer to start

STATE TRUNK_LOCKED -> all is off, (used if speed if greater than 5 km/h)
STATE TRUNK_LATCHED (default) -> all is off
STATE TRUNK_OPENNING -> led is on, coil is energized, TRUNK_TIMEOUT is started for 2 seconds.
STATE TRUNK_LEDFEEDBACK_ON -> led is on, coil is energized, TRUNK_TIMEOUT is started for 2 seconds.
STATE TRUNK_LEDFEEDBACK_ERROR -> feedback led is blinking, TRUNK_TIMEOUT is started for 4 seconds.

Transition from TRUNK_LATCHED to TRUNK_OPENNING : button trunk released short press
Transition from TRUNK_LATCHED to TRUNK_OPENNING : button trunk releasd long press

Transition from TRUNK_LATCHED to TRUNK_LOCKED : vehicle speed is greater than 5km/h

Transition from TRUNK_LOCKED to TRUNK_LATCHED : vehicle speed is lower or equal than 5km/h

Transition from TRUNK_LOCKED to TRUNK_LEDFEEDBACK_ERROR : button trunk released short press
Transition from TRUNK_LOCKED to TRUNK_LEDFEEDBACK_ERROR : button trunk released long press

Transition from TRUNK_OPENNING to TRUNK_LEDFEEDBACK_ON : TRUNK_TIMEOUT

Transition from TRUNK_LEDFEEDBACK_ON to TRUNK_LATCHED : TRUNK_TIMEOUT

Transition from TRUNK_LEDFEEDBACK_ERROR to TRUNK_LOCKED : TRUNK_TIMEOUT event

### WARNINGS light
this is driven by a non-latching switch
there is the visual feedback to turn off/blinking
there is the can code to send for the dispaly feedback. (same used by the rear ligtning board)

STATE WARNING_OFF (default) -> off

STATE WARNING_CLEAR -> no can code output, not visual feedback,blinker left + right off, start WARNING_TIMEOUT 

STATE WARNING_BLINK -> can code output for HUD + Front/rear board, visual feedback on the button,blinker left + right on, start WARNING_TIMEOUT


Transition from WARNING_OFF to WARNING_BLINK : button fog released short press
Transition from WARNING_OFF to WARNING_BLINK : button fog releasd long press

Transition from WARNING_BLINK to WARNING_OFF : button fog released short press
Transition from WARNING_BLINK to WARNING_OFF : button fog releasd long press
Transition from WARNING_BLINK to WARNING_CLEAR : WARNING_TIMEOUT event

Transition from WARNING_CLEAR to WARNING_OFF : button fog released short press
Transition from WARNING_CLEAR to WARNING_OFF : button fog releasd long press
Transition from WARNING_CLEAR to WARNING_BLINK : WARNING_TIMEOUT event

### FORWARD / REVERSE
this is driven by a non-latching switch and there is "latching" seat sensor
there is the can code to send for the motor controller while in reverse.

this allow set forward mode using a press on the forward button, but if we go too fast in reverse, we cannot switch unless we slow down within 10 second
same principle for reverse. this allow you to rock back and forth to get out of a ditch but prevent you from destroying the vehicle if there is a missclick while driving.

STATE NEUTRAL (default) -> off

STATE FORWARD -> can code output for motor driver

STATE FORWARD_LOCK -> can code output for motor driver

STATE REVERSE -> can code output for motor driver

STATE REVERSE_LOCK -> can code output for motor driver

Transition from NEUTRAL to FORWARD : button forward released short press
Transition from NEUTRAL to FORWARD : button forward releasd long press

Transition from NEUTRAL to REVERSE : button reverse released short press
Transition from NEUTRAL to REVERSE : button reverse releasd long press

Transition from REVERSE to FORWARD : button forward released short press
Transition from REVERSE to FORWARD : button forward releasd long press
Transition from REVERSE to FORWARD : FORWARD_TIMEOUT event

Transition from FORWARD to REVERSE : button reverse released short press
Transition from FORWARD to REVERSE : button reverse releasd long press
Transition from FORWARD to REVERSE : REVERSE_TIMEOUT event

Transition from FORWARD to FORWARD_LOCK : forward speed greater than 5kmh

Transition from REVERSE to REVERSE_LOCK : reverse speed greater than 5kmh

Transition from FORWARD_LOCK to FORWARD : forward speed lower or equal than 5kmh

Transition from REVERSE_LOCK to REVERSE : reverse speed lower or equal than 5kmh

Transition from REVERSE_LOCK to REVERSE_LOCK : button forward released short press , but we start FORWARD_TIMEOUT at 10 sec
Transition from REVERSE_LOCK to REVERSE_LOCK : button forward releasd long press, but we start FORWARD_TIMEOUT at 10 sec

Transition from FORWARD_LOCK to FORWARD_LOCK : button reverse released short press , but we start REVERSE_TIMEOUT at 10 sec
Transition from FORWARD_LOCK to FORWARD_LOCK : button reverse releasd long press, but we start REVERSE_TIMEOUT at 10 sec


Transition from FORWARD to NEUTRAL : seat sensor released for more than 10 sec event
Transition from FORWARD_LOCK to NEUTRAL :  seat sensor released for more than 10 sec relased event
Transition from REVERSE to NEUTRAL :  seat sensor released for more than 10 sec event
Transition from FORWARD_LOCK to NEUTRAL :  seat sensor released for more than 10 sec event

IMPLEMENT LOGIC WHEN READING BUTTON IF BOTH ARE PRESENT -> ERROR WITH THE BUTTON

### BRAKE SWITCH INPUT

STATE BRAKE_LEVEL_0
STATE BRAKE_LEVEL_25
STATE BRAKE_LEVEL_50
STATE BRAKE_LEVEL_100

Transition from ANY to BRAKE_LEVEL_0 : no brake pressed
Transition from ANY to BRAKE_LEVEL_25 : left brake pressed and NOT right brake pressed
Transition from ANY to BRAKE_LEVEL_50 : right brake pressed and NOT left brake pressed
Transition from ANY to BRAKE_LEVEL_100 : right brake pressed and left brake pressed

### SEAT HEATER
use RGB to show the tempearture preset while edditing, timeout after 10 sec.
long press to change the temperature preset.
shortpress to enable with timeout (30 min)

STATE SEATHEATER_STANDBY (default) -> stay blocked

STATE SEATHEATER_EDITION -> allow edition of PID target. set a SEATHEATER_TIMEOUT for 10 second

STATE SEATHEATER_ON -> enable heating timeout for 30 minute.

Transition from SEATHEATER_STANDBY to SEATHEATER_EDITION : button SEATHEATER releasd long press

Transition from SEATHEATER_EDITION to SEATHEATER_STANDBY : button SEATHEATER releasd long press
Transition from SEATHEATER_EDITION to SEATHEATER_STANDBY : SEATHEATER_TIMEOUT event

Transition from SEATHEATER_STANDBY to SEATHEATER_ON : button SEATHEATER releasd short press

Transition from SEATHEATER_ON to SEATHEATER_STANDBY : button SEATHEATER releasd short press
Transition from SEATHEATER_ON to SEATHEATER_STANDBY : SEATHEATER_TIMEOUT event

### HAND HEATER

use RGB to show the tempearture preset while edditing, timeout after 10 sec.
long press to change the temperature preset.
shortpress to enable with timeout (30 min)

STATE HANDHEATER_STANDBY (default) -> stay blocked

STATE HANDHEATER_EDITION -> allow edition of PID target. set a HANDHEATER_TIMEOUT for 10 second

STATE HANDHEATER_ON -> enable heating timeout for 30 minute.

Transition from HANDHEATER_STANDBY to HANDHEATER_EDITION : button HANDHEATER releasd long press
Transition from HANDHEATER_ON to HANDHEATER_EDITION : button HANDHEATER releasd long press

Transition from HANDHEATER_EDITION to HANDHEATER_ON : button HANDHEATER releasd long press (always turn on after validating settings)

Transition from HANDHEATER_EDITION to HANDHEATER_STANDBY : HANDHEATER_TIMEOUT event (unless it was a missclick)

Transition from HANDHEATER_STANDBY to HANDHEATER_ON : button HANDHEATER releasd short press

Transition from HANDHEATER_ON to HANDHEATER_STANDBY : button HANDHEATER releasd short press
Transition from HANDHEATER_ON to HANDHEATER_STANDBY : HANDHEATER_TIMEOUT event

Transition from HANDHEATER_EDITION to HANDHEATER_EDITION : incrementation / decremenation event that change the preset value

### Speed limiter

STATE SPEED_LIMIT_EDITION (default) 
STATE ASSIST_COEFF_EDITION 

Transition from SPEED_LIMIT_EDITION to ASSIST_COEFF_EDITION : button speedlimiter released short press
Transition from ASSIST_COEFF_EDITION to SPEED_LIMIT_EDITION : button speedlimiter releasd short press

rotation event increment / decrement the selected variable (speed limiter or assist coeeff)


### MAPPING button

STATE MAPPING_CLEAR (default) -> nothing
STATE MAPPING_PRESSED -> tun on indicator light, start a timeout for 2 second

Transition from MAPPING_CLEAR to MAPPING_PRESSED : button forward released short press
Transition from MAPPING_CLEAR to MAPPING_PRESSED : button forward releasd long press

Transition from MAPPING_PRESSED to MAPPING_CLEAR : MAPPING_BUTTON_TIMEOUT event

### AUDIO ouput

1:1 mapping with seat sensor released for more than 10 sec event

### USB output

mapping with seat sensor released for more than 10 sec event
implement a 30 minute timer

STATE USB_POWERED
STATE USB_GRACE, start a timer of 30 min
STATE USB_OFF (default)

Transition from USB_OFF to USB_POWERED : seat sensor pressed event
Transition from USB_POWERED to c : seat sensor released for more than 10 sec event
Transition from v to USB_OFF : USB_TIMEOUT event

### DRL leds
1:1 mapping between can code and mosfet output
### LEFT blinker
1:1 mapping between can code and mosfet output
### RIGHT blinker
1:1 mapping between can code and mosfet output

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

# Output and PID
use PID v1
digital output can be set to ON / OFF / SLOW BLINK

