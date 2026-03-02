/*
 * Velion Mainboard Firmware
 * ESP32 + MCP23017 x2 + INA228 x4 + MPU6050/BMP280 + WS2812 + CAN (TWAI)
 *
 * Architecture:
 *   1. INPUT  — read buttons via MCP, CAN RX, sensors
 *   2. EVENT  — generate discrete events from input state changes
 *   3. FSM    — step each state machine with current events
 *   4. OUTPUT — apply FSM states to HW (MOSFETs, LEDs, CAN TX)
 *
 * Libraries (install via Arduino Library Manager):
 *   - Adafruit MCP23X17
 *   - Adafruit NeoPixel
 *   - Adafruit MPU6050
 *   - Adafruit BMP280
 *   - Adafruit INA228
 *   - ESP32 board package v2.0.17 (includes driver/twai.h)
 */

#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA228.h>
#include "driver/twai.h"

// ═══════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS (required for Arduino IDE prototype generation)
// ═══════════════════════════════════════════════════════════════════
struct ButtonState;
struct SoftTimer;
struct Events;
void btnReset(ButtonState &b);
void btnUpdate(ButtonState &b, bool rawNow);
void timerStart(SoftTimer &t, uint32_t ms);
void timerStop(SoftTimer &t);
bool timerExpired(SoftTimer &t);

// ═══════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS (HAL)
// ═══════════════════════════════════════════════════════════════════
#define THROTTLE_INPUT                36
#define InterruptEXTGPIO              39
#define POT_SPEED_A                   34
#define POT_SPEED_B                   35
#define POT_SEATHEATER_A              32
#define POT_SEATHEATER_B              33
#define POT_HANDHEATER_A              25
#define POT_HANDHEATER_B              26
#define DEFROSTER_MOSFET_GATE         13
#define INTERRIOR_LIGHT_MOSFET_GATE   15
#define BLIKER_LEFT_MOSFET_GATE        2
#define BLIKER_RIGHT_MOSFET_GATE       0
#define HANDHEATER_MOSFET_GATE         4
#define DRL_RIGHT_MOSFET_GATE         16
#define DRL_LEFT_MOSFET_GATE          17
#define SEATHEATER_MOSFET_GATE         5
#define RGBLED_DIN                    21
#define SDA_PIN                       23
#define SCL_PIN                       22

// CAN (TWAI) pins — adjust to your PCB
#define CAN_TX_PIN  GPIO_NUM_27
#define CAN_RX_PIN  GPIO_NUM_14

// ═══════════════════════════════════════════════════════════════════
//  MCP23017 PIN ALIASES (logical pin numbers inside Adafruit lib)
//  GPA0-GPA7 = 0-7,   GPB0-GPB7 = 8-15
// ═══════════════════════════════════════════════════════════════════

// MCP1 (0x21) ---------------------------------------------------
#define MCP1_WARNING_LED         8   // GPB0
#define MCP1_CABLIGHT_LED        9   // GPB1
#define MCP1_DEFROST_LED        10   // GPB2
#define MCP1_TRUNK_LED          11   // GPB3
#define MCP1_BROUILLARD_LED     12   // GPB4
#define MCP1_MAP_LED            13   // GPB5
#define MCP1_DEFROST_INPUT      14   // GPB6
#define MCP1_TRUNK_INPUT        15   // GPB7

#define MCP1_AUXAUDIO_GATE       0   // GPA0
#define MCP1_AUXUSB_GATE         1   // GPA1
#define MCP1_LATCH_TRUNK_GATE    2   // GPA2
#define MCP1_POWER_GATE          3   // GPA3

// MCP2 (0x20) ---------------------------------------------------
#define MCP2_DEFROSTER_ALERT     8   // GPB0
#define MCP2_HANDHEATER_ALERT    9   // GPB1
#define MCP2_SEATHEATER_ALERT   10   // GPB2
#define MCP2_POWERINPUT_ALERT   11   // GPB3
#define MCP2_POT_SEATHEATER_X   12   // GPB4
#define MCP2_POT_HANDHEATER_X   13   // GPB5
#define MCP2_POT_SPEED_X        14   // GPB6
#define MCP2_SEAT_INPUT         15   // GPB7

#define MCP2_BRAKE_RIGHT_INPUT   0   // GPA0
#define MCP2_BRAKE_LEFT_INPUT    1   // GPA1
#define MCP2_REVERSE_INPUT       2   // GPA2
#define MCP2_FORWARD_INPUT       3   // GPA3
#define MCP2_BROUILLARD_INPUT    4   // GPA4
#define MCP2_CABLIGHT_INPUT      5   // GPA5
#define MCP2_WARNING_INPUT       6   // GPA6
#define MCP2_MAP_INPUT           7   // GPA7

// ═══════════════════════════════════════════════════════════════════
//  NeoPixel
// ═══════════════════════════════════════════════════════════════════
#define NUM_PIXELS 5
Adafruit_NeoPixel pixels(NUM_PIXELS, RGBLED_DIN, NEO_GRB + NEO_KHZ800);

// ═══════════════════════════════════════════════════════════════════
//  PERIPHERAL OBJECTS
// ═══════════════════════════════════════════════════════════════════
Adafruit_MCP23X17 mcp1;   // 0x21
Adafruit_MCP23X17 mcp2;   // 0x20

Adafruit_INA228 inaMain;  // 0x40 total
Adafruit_INA228 inaHand;  // 0x41 hand heater
Adafruit_INA228 inaSeat;  // 0x44 seat heater
Adafruit_INA228 inaDef;   // 0x45 defroster

Adafruit_MPU6050 mpu;
Adafruit_BMP280  bmp;

bool mcp1_ok = false, mcp2_ok = false;
bool inaMain_ok = false, inaHand_ok = false, inaSeat_ok = false, inaDef_ok = false;
bool mpu_ok = false, bmp_ok = false;

// ═══════════════════════════════════════════════════════════════════
//  INA228 CONFIG
// ═══════════════════════════════════════════════════════════════════
#define SHUNT_RESISTOR_OHMS  0.001f
#define MAX_EXPECTED_CURRENT 50.0f

// ═══════════════════════════════════════════════════════════════════
//  CAN IDs (SiliXcon LYNX protocol)
// ═══════════════════════════════════════════════════════════════════
#define CAN_ID_CONTROL       0x5FF   // TX: control input message
#define CAN_ID_VDS_BTN_EVT   0x5FE   // RX: VDS display button event
#define CAN_ID_LYNX_STATUS   0x600   // RX: LYNX status
#define CAN_ID_MOTOR_STATUS  0x610   // RX: motor / vehicle speed
#define CAN_ID_BATTERY_STATUS 0x618  // RX: battery SOC/voltage/current

// Mainboard broadcast range 0x420-0x480
#define CAN_ID_MB_LIGHTS     0x420   // TX: lighting state
#define CAN_ID_MB_BLINKER    0x421   // TX: blinker/warning state
#define CAN_ID_MB_DRIVE      0x422   // TX: drive direction (fwd/rev/neutral)
#define CAN_ID_MB_BRAKE      0x423   // TX: brake / regen level

// Speed threshold
#define SPEED_THRESHOLD_KPH  5

// ═══════════════════════════════════════════════════════════════════
//  BUTTON DEBOUNCE — simple edge detect with debounce
// ═══════════════════════════════════════════════════════════════════
#define BTN_DEBOUNCE_MS    30
#define BTN_LONG_PRESS_MS  800

struct ButtonState {
  bool     raw;           // current debounced level (true = pressed)
  bool     prevRaw;       // previous cycle
  uint32_t pressTime;     // millis when pressed
  bool     shortRelease;  // one-shot
  bool     longRelease;   // one-shot
  bool     pressed;       // one-shot on press
  bool     released;      // one-shot on release (don't care long/short)
  uint32_t lastChange;
};

ButtonState btnMap       = {};
ButtonState btnTrunk     = {};
ButtonState btnDefrost   = {};
ButtonState btnCablight  = {};
ButtonState btnBrouillard = {};
ButtonState btnWarning   = {};
ButtonState btnForward   = {};
ButtonState btnReverse   = {};
ButtonState btnSeat      = {};
ButtonState btnBrakeLeft = {};
ButtonState btnBrakeRight = {};

void btnReset(ButtonState &b) {
  b.shortRelease = false;
  b.longRelease  = false;
  b.pressed      = false;
  b.released     = false;
}

void btnUpdate(ButtonState &b, bool rawNow) {
  uint32_t now = millis();
  if (rawNow != b.raw) {
    if (now - b.lastChange > BTN_DEBOUNCE_MS) {
      b.prevRaw    = b.raw;
      b.raw        = rawNow;
      b.lastChange = now;

      if (b.raw && !b.prevRaw) {          // press edge
        b.pressed   = true;
        b.pressTime = now;
      }
      if (!b.raw && b.prevRaw) {          // release edge
        b.released = true;
        uint32_t held = now - b.pressTime;
        if (held >= BTN_LONG_PRESS_MS) b.longRelease  = true;
        else                           b.shortRelease = true;
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  EVENT FLAGS  (cleared every loop cycle)
// ═══════════════════════════════════════════════════════════════════
struct Events {
  // Mapping button
  bool BTN_MAPPING_SHORT;
  bool BTN_MAPPING_LONG;
  bool MAPPING_BUTTON_TIMEOUT;

  // Cabin light
  bool BTN_CABLIGHT_AND_STATIONNARY;
  bool BTN_CABLIGHT_AND_DRIVING;
  bool BTN_CABLIGHT_RELEASED;
  bool CABINLIGHT_TIMER_LONG_TIMEOUT;
  bool CABINLIGHT_TIMER_SHORT_TIMEOUT;
  bool BTN_TRUNK_AND_STATIONNARY;

  // Speed / driving
  bool DRIVING;
  bool STATIONNARY;
  bool VEHICLE_SPEED_ABOVE_5KPH;
  bool VEHICLE_SPEED_BELOW_5KPH;

  // Trunk
  bool BTN_TRUNK_RELEASED_AND_STATIONNARY;
  bool BTN_TRUNK_RELEASED_AND_DRIVING;
  bool TRUNK_ACTION_TIMER_TIMEOUT;
  bool TRUNK_BLINK_TIMER_TIMEOUT;
  bool TRUNK_ERROR_TIMER_TIMEOUT;

  // Fog
  bool BTN_BROUILLARD_SHORT;
  bool BTN_BROUILLARD_LONG;
  bool FOG_ON_TIMER_TIMEOUT;

  // Seat sensor
  bool SEATSENSOR_PRESSED;
  bool SEATSENSOR_RELEASED;

  // Defroster
  bool BTN_DEFROSTER_SHORT_AND_BATTERY_OK;
  bool BTN_DEFROSTER_LONG_AND_BATTERY_OK;
  bool BTN_DEFROSTER_SHORT;
  bool BTN_DEFROSTER_LONG;
  bool DEFROSTER_ON_TIMER_TIMEOUT;
  bool DEFROSTER_FAULT_TIMER_TIMEOUT;
  bool DEFROSTER_BLINK_TIMER_TIMEOUT;
  bool BATTERY_BELOW_30_PERCENT;
  bool DEFROSTER_FAULT;

  // Blinker / warning
  bool BTN_BLINKER_LEFT_RELEASED;
  bool BTN_BLINKER_RIGHT_RELEASED;
  bool BTN_WARNING_RELEASED;
  bool BLINKER_CYCLE_TIMEOUT;
  bool WARNING_CYCLE_TIMEOUT;

  // Drive direction
  bool BTN_FORWARD_SHORT;
  bool BTN_FORWARD_LONG;
  bool BTN_REVERSE_SHORT;
  bool BTN_REVERSE_LONG;
  bool DRIVE_CHANGE_TIMER_TIMEOUT;

  // Brake
  bool BRAKE_NONE;
  bool BRAKE_LEFT_ONLY;
  bool BRAKE_RIGHT_ONLY;
  bool BRAKE_BOTH;

  // Power (from VDS CAN button)
  bool POWER_BUTTON_SHORT_CAN;
  bool POWER_BUTTON_LONG_CAN;
};

Events ev;

void clearEvents() { memset(&ev, 0, sizeof(ev)); }

// ═══════════════════════════════════════════════════════════════════
//  SOFTWARE TIMERS
// ═══════════════════════════════════════════════════════════════════
struct SoftTimer {
  bool     running;
  uint32_t startMs;
  uint32_t durationMs;
};

void timerStart(SoftTimer &t, uint32_t ms) {
  t.running    = true;
  t.startMs    = millis();
  t.durationMs = ms;
}

void timerStop(SoftTimer &t) { t.running = false; }

bool timerExpired(SoftTimer &t) {
  if (!t.running) return false;
  if (millis() - t.startMs >= t.durationMs) {
    t.running = false;
    return true;
  }
  return false;
}

// Timer instances
SoftTimer tmrMapping          = {false, 0, 2000};
SoftTimer tmrCablightLong     = {false, 0, 3600000UL};   // 60 min
SoftTimer tmrCablightShort    = {false, 0, 300000UL};     // 5 min
SoftTimer tmrTrunkAction      = {false, 0, 2000};
SoftTimer tmrTrunkBlink       = {false, 0, 400};
SoftTimer tmrTrunkError       = {false, 0, 3000};
SoftTimer tmrFogOn            = {false, 0, 120000UL};     // 2 min
SoftTimer tmrDefrosterOn      = {false, 0, 300000UL};     // 5 min
SoftTimer tmrDefrosterFault   = {false, 0, 3000};
SoftTimer tmrDefrosterBlink   = {false, 0, 400};
SoftTimer tmrBlinkerCycle     = {false, 0, 400};
SoftTimer tmrWarningCycle     = {false, 0, 125};
SoftTimer tmrDriveChange      = {false, 0, 5000};
SoftTimer tmrPwrIdle          = {false, 0, 3600000UL};    // 60 min
SoftTimer tmrDrlIdle          = {false, 0, 3600000UL};
SoftTimer tmrLowbeamIdle      = {false, 0, 300000UL};     // 5 min

// ═══════════════════════════════════════════════════════════════════
//  FSM STATES
// ═══════════════════════════════════════════════════════════════════

// — Mapping button —
enum MappingState { ST_MAPPING_CLEAR, ST_MAPPING_PRESSED };
MappingState fsmMapping = ST_MAPPING_CLEAR;

// — Cabin light —
enum CabinLightState { ST_CABINLIGHT_OFF, ST_CABINLIGHT_ON,
                        ST_CABINLIGHT_ON_TRUNK, ST_CABINLIGHT_ON_DRV };
CabinLightState fsmCabinLight = ST_CABINLIGHT_OFF;

// — Trunk —
enum TrunkState { ST_TRUNK_LATCHED, ST_TRUNK_OPENING,
                   ST_TRUNK_BLINK_ON, ST_TRUNK_BLINK_OFF };
TrunkState fsmTrunk = ST_TRUNK_LATCHED;

// — Fog —
enum FogState { ST_FSM_FOG_OFF, ST_FSM_FOG_ON, ST_FSM_FOG_ON_GRACE };
FogState fsmFog = ST_FSM_FOG_OFF;

// — Defroster —
enum DefrosterState { ST_FSM_DEFROSTER_OFF, ST_FSM_DEFROSTER_ON,
                       ST_FSM_DEFROSTER_FAULT_BLINK_OFF,
                       ST_FSM_DEFROSTER_FAULT_BLINK_ON };
DefrosterState fsmDefroster = ST_FSM_DEFROSTER_OFF;

// — Blinker / Warning —
enum BlinkerState { ST_BLINKER_IDLE,
                     ST_WARNING_ON, ST_WARNING_OFF,
                     ST_BLINKER_LEFT_ON,  ST_BLINKER_LEFT_OFF,
                     ST_BLINKER_RIGHT_ON, ST_BLINKER_RIGHT_OFF };
BlinkerState fsmBlinker = ST_BLINKER_IDLE;

// — Drive direction —
enum DriveState { ST_DRIVE_NEUTRAL,
                   ST_DRIVE_FORWARD, ST_DRIVE_FORWARD_LOCK, ST_DRIVE_FWD_PENDING,
                   ST_DRIVE_REVERSE, ST_DRIVE_REVERSE_LOCK, ST_DRIVE_REV_PENDING };
DriveState fsmDrive = ST_DRIVE_NEUTRAL;

// — Brake —
enum BrakeState { ST_BRAKE_LEVEL_0, ST_BRAKE_LEVEL_25,
                   ST_BRAKE_LEVEL_50, ST_BRAKE_LEVEL_100 };
BrakeState fsmBrake = ST_BRAKE_LEVEL_0;

// — Power —
enum PowerState { ST_POWER_OFF, ST_POWER_ON, ST_POWER_ON_GRACE };
PowerState fsmPower = ST_POWER_OFF;

// — DRL —
enum DrlState { ST_DRL_OFF, ST_DRL_ON, ST_DRL_ON_GRACE };
DrlState fsmDrl = ST_DRL_OFF;

// — Low beam —
enum LowbeamState { ST_LOWBEAM_OFF, ST_LOWBEAM_ON, ST_LOWBEAM_ON_GRACE };
LowbeamState fsmLowbeam = ST_LOWBEAM_OFF;

// ═══════════════════════════════════════════════════════════════════
//  LOCAL STATE (from sensors / CAN)
// ═══════════════════════════════════════════════════════════════════
int16_t  vehicleSpeedKph   = 0;    // from CAN 0x610 bytes 4-5
uint8_t  batterySocPct     = 100;  // from CAN 0x618 byte 2
int16_t  batteryVoltage01V = 0;    // 0.01 V units
int16_t  batteryCurrent02A = 0;    // 0.02 A units
uint8_t  lynxMode          = 0;
uint8_t  currentMap        = 0;
bool     prevSpeedAbove5   = false;

// ═══════════════════════════════════════════════════════════════════
//  CAN TX: 0x5FF control message — built every cycle
// ═══════════════════════════════════════════════════════════════════
int16_t  canLevel1 = 0x7FFF;  // 32767 = invalid / not used
int16_t  canLevel2 = 0x7FFF;
int16_t  canLevel3 = 0x7FFF;
uint8_t  canDigIn  = 0;       // byte 6: bits 0-3 digital in, bits 4-7 map switch
uint8_t  canCmd    = 0;       // byte 7: bit 0 = disarm/seatswitch

// ═══════════════════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════════════════
uint32_t lastSensorPrint  = 0;
#define  SENSOR_PRINT_MS  5000

uint32_t lastCanTx        = 0;
#define  CAN_TX_INTERVAL  50     // 50 ms → 20 Hz (well within 200 ms timeout)

uint32_t lastCanBroadcast = 0;
#define  CAN_BROADCAST_MS 100

// ═══════════════════════════════════════════════════════════════════
//  SETUP HELPERS
// ═══════════════════════════════════════════════════════════════════

void setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("[CAN] Driver installed");
  } else {
    Serial.println("[CAN] Driver install FAILED");
  }
  if (twai_start() == ESP_OK) {
    Serial.println("[CAN] Started");
  } else {
    Serial.println("[CAN] Start FAILED");
  }
}

void setupMCP() {
  // MCP1 (0x21) — LEDs + MOSFET gates + 2 buttons
  if (mcp1.begin_I2C(0x21)) {
    mcp1_ok = true;
    Serial.println("[MCP1] 0x21 OK");

    // GPB outputs (LEDs)
    mcp1.pinMode(MCP1_WARNING_LED,    OUTPUT);
    mcp1.pinMode(MCP1_CABLIGHT_LED,   OUTPUT);
    mcp1.pinMode(MCP1_DEFROST_LED,    OUTPUT);
    mcp1.pinMode(MCP1_TRUNK_LED,      OUTPUT);
    mcp1.pinMode(MCP1_BROUILLARD_LED, OUTPUT);
    mcp1.pinMode(MCP1_MAP_LED,        OUTPUT);

    // GPB inputs (buttons)
    mcp1.pinMode(MCP1_DEFROST_INPUT, INPUT_PULLUP);
    mcp1.pinMode(MCP1_TRUNK_INPUT,   INPUT_PULLUP);

    // GPA outputs (MOSFETs)
    mcp1.pinMode(MCP1_AUXAUDIO_GATE,    OUTPUT);
    mcp1.pinMode(MCP1_AUXUSB_GATE,      OUTPUT);
    mcp1.pinMode(MCP1_LATCH_TRUNK_GATE, OUTPUT);
    mcp1.pinMode(MCP1_POWER_GATE,       OUTPUT);

    // *** POWER MOSFET always ON ***
    mcp1.digitalWrite(MCP1_POWER_GATE, HIGH);
  } else {
    Serial.println("[MCP1] 0x21 NOT FOUND");
  }

  // MCP2 (0x20) — alerts, rotary switches, buttons
  if (mcp2.begin_I2C(0x20)) {
    mcp2_ok = true;
    Serial.println("[MCP2] 0x20 OK");

    // GPB inputs
    mcp2.pinMode(MCP2_DEFROSTER_ALERT,  INPUT);
    mcp2.pinMode(MCP2_HANDHEATER_ALERT, INPUT);
    mcp2.pinMode(MCP2_SEATHEATER_ALERT, INPUT);
    mcp2.pinMode(MCP2_POWERINPUT_ALERT, INPUT);
    mcp2.pinMode(MCP2_POT_SEATHEATER_X, INPUT);
    mcp2.pinMode(MCP2_POT_HANDHEATER_X, INPUT);
    mcp2.pinMode(MCP2_POT_SPEED_X,      INPUT);
    mcp2.pinMode(MCP2_SEAT_INPUT,        INPUT_PULLUP);

    // GPA inputs (buttons with pull-ups)
    mcp2.pinMode(MCP2_BRAKE_RIGHT_INPUT, INPUT_PULLUP);
    mcp2.pinMode(MCP2_BRAKE_LEFT_INPUT,  INPUT_PULLUP);
    mcp2.pinMode(MCP2_REVERSE_INPUT,     INPUT_PULLUP);
    mcp2.pinMode(MCP2_FORWARD_INPUT,     INPUT_PULLUP);
    mcp2.pinMode(MCP2_BROUILLARD_INPUT,  INPUT_PULLUP);
    mcp2.pinMode(MCP2_CABLIGHT_INPUT,    INPUT_PULLUP);
    mcp2.pinMode(MCP2_WARNING_INPUT,     INPUT_PULLUP);
    mcp2.pinMode(MCP2_MAP_INPUT,         INPUT_PULLUP);

    // Interrupts for alert pins (GPB0-GPB3)
    mcp2.setupInterrupts(true, false, LOW);
    for (int i = MCP2_DEFROSTER_ALERT; i <= MCP2_POWERINPUT_ALERT; i++) {
      mcp2.setupInterruptPin(i, CHANGE);
    }
  } else {
    Serial.println("[MCP2] 0x20 NOT FOUND");
  }
}

void setupINA() {
  if (inaMain.begin(0x40)) {
    inaMain_ok = true;
    inaMain.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
    inaMain.setAveragingCount(INA228_COUNT_16);
    Serial.println("[INA] 0x40 OK (total)");
  } else {
    Serial.println("[INA] 0x40 FAIL");
  }

  if (inaHand.begin(0x41)) {
    inaHand_ok = true;
    inaHand.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
    inaHand.setAveragingCount(INA228_COUNT_16);
    Serial.println("[INA] 0x41 OK (hand heater)");
  } else {
    Serial.println("[INA] 0x41 FAIL");
  }

  if (inaSeat.begin(0x44)) {
    inaSeat_ok = true;
    inaSeat.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
    inaSeat.setAveragingCount(INA228_COUNT_16);
    Serial.println("[INA] 0x44 OK (seat heater)");
  } else {
    Serial.println("[INA] 0x44 FAIL");
  }

  if (inaDef.begin(0x45)) {
    inaDef_ok = true;
    inaDef.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
    inaDef.setAveragingCount(INA228_COUNT_16);
    Serial.println("[INA] 0x45 OK (defroster)");
  } else {
    Serial.println("[INA] 0x45 FAIL");
  }
}

void setupIMU() {
  if (mpu.begin(0x68)) {
    mpu_ok = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[IMU] MPU6050 @ 0x68 OK");
  } else {
    Serial.println("[IMU] MPU6050 @ 0x68 FAIL");
  }

  if (bmp.begin(0x76)) {
    bmp_ok = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("[IMU] BMP280 @ 0x76 OK");
  } else {
    Serial.println("[IMU] BMP280 @ 0x76 FAIL");
  }
}

void setupNeoPixel() {
  pixels.begin();
  pixels.setBrightness(60);
  // Set all 5 LEDs to white (backlight)
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  }
  pixels.show();
  Serial.println("[LED] WS2812 x5 white backlight set");
}

void setupGPIO() {
  // MOSFET outputs — all OFF initially
  pinMode(DEFROSTER_MOSFET_GATE,       OUTPUT); digitalWrite(DEFROSTER_MOSFET_GATE,       LOW);
  pinMode(INTERRIOR_LIGHT_MOSFET_GATE, OUTPUT); digitalWrite(INTERRIOR_LIGHT_MOSFET_GATE, LOW);
  pinMode(BLIKER_LEFT_MOSFET_GATE,     OUTPUT); digitalWrite(BLIKER_LEFT_MOSFET_GATE,     LOW);
  pinMode(BLIKER_RIGHT_MOSFET_GATE,    OUTPUT); digitalWrite(BLIKER_RIGHT_MOSFET_GATE,    LOW);
  pinMode(HANDHEATER_MOSFET_GATE,      OUTPUT); digitalWrite(HANDHEATER_MOSFET_GATE,      LOW);
  pinMode(DRL_RIGHT_MOSFET_GATE,       OUTPUT); digitalWrite(DRL_RIGHT_MOSFET_GATE,       LOW);
  pinMode(DRL_LEFT_MOSFET_GATE,        OUTPUT); digitalWrite(DRL_LEFT_MOSFET_GATE,        LOW);
  pinMode(SEATHEATER_MOSFET_GATE,      OUTPUT); digitalWrite(SEATHEATER_MOSFET_GATE,      LOW);

  // Analog / interrupt inputs
  pinMode(THROTTLE_INPUT,   INPUT);
  pinMode(InterruptEXTGPIO, INPUT);
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println(" Velion Mainboard Firmware — Booting");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  setupGPIO();
  setupCAN();
  setupMCP();
  setupINA();
  setupIMU();
  setupNeoPixel();

  Serial.println("[BOOT] Ready.\n");
}

// ═══════════════════════════════════════════════════════════════════
//  INPUT STAGE
// ═══════════════════════════════════════════════════════════════════

// --- Read MCP buttons (active low: pressed = LOW → true) ---
void readButtons() {
  if (mcp1_ok) {
    btnUpdate(btnDefrost, !mcp1.digitalRead(MCP1_DEFROST_INPUT));
    btnUpdate(btnTrunk,   !mcp1.digitalRead(MCP1_TRUNK_INPUT));
  }
  if (mcp2_ok) {
    btnUpdate(btnBrakeRight, !mcp2.digitalRead(MCP2_BRAKE_RIGHT_INPUT));
    btnUpdate(btnBrakeLeft,  !mcp2.digitalRead(MCP2_BRAKE_LEFT_INPUT));
    btnUpdate(btnReverse,    !mcp2.digitalRead(MCP2_REVERSE_INPUT));
    btnUpdate(btnForward,    !mcp2.digitalRead(MCP2_FORWARD_INPUT));
    btnUpdate(btnBrouillard, !mcp2.digitalRead(MCP2_BROUILLARD_INPUT));
    btnUpdate(btnCablight,   !mcp2.digitalRead(MCP2_CABLIGHT_INPUT));
    btnUpdate(btnWarning,    !mcp2.digitalRead(MCP2_WARNING_INPUT));
    btnUpdate(btnMap,        !mcp2.digitalRead(MCP2_MAP_INPUT));
    btnUpdate(btnSeat,       !mcp2.digitalRead(MCP2_SEAT_INPUT));
  }
}

// --- CAN RX — read all pending, print, parse known IDs ---
void readCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    // Debug: print every CAN packet on serial
    Serial.printf("[CAN RX] ID=0x%03X DLC=%d Data=", msg.identifier, msg.data_length_code);
    for (int i = 0; i < msg.data_length_code; i++) {
      Serial.printf("%02X ", msg.data[i]);
    }
    Serial.println();

    // Parse known IDs (all little-endian per SiliXcon docs)
    switch (msg.identifier) {
      case CAN_ID_MOTOR_STATUS: {
        // 0x610: bytes 4-5 = vehicle speed [km/h] INT16 LE
        if (msg.data_length_code >= 6) {
          vehicleSpeedKph = (int16_t)(msg.data[4] | (msg.data[5] << 8));
        }
        break;
      }
      case CAN_ID_BATTERY_STATUS: {
        // 0x618: byte 2 = SOC%, bytes 4-5 = Vbat [0.01V], bytes 6-7 = Ibat [0.02A]
        if (msg.data_length_code >= 8) {
          batterySocPct     = msg.data[2];
          batteryVoltage01V = (int16_t)(msg.data[4] | (msg.data[5] << 8));
          batteryCurrent02A = (int16_t)(msg.data[6] | (msg.data[7] << 8));
        }
        break;
      }
      case CAN_ID_LYNX_STATUS: {
        // 0x600: byte 1 = LYNX mode, byte 2 = current power map
        if (msg.data_length_code >= 3) {
          lynxMode   = msg.data[1];
          currentMap = msg.data[2];
        }
        break;
      }
      case CAN_ID_VDS_BTN_EVT: {
        // 0x5FE: bytes 0-3 = UINT32 VDS /common/buttons value
        if (msg.data_length_code >= 4) {
          uint32_t btns = msg.data[0] | (msg.data[1] << 8) |
                          ((uint32_t)msg.data[2] << 16) | ((uint32_t)msg.data[3] << 24);
          Serial.printf("[VDS] buttons=0x%08X\n", btns);
          // Treat button 1 as power toggle
          if (btns & 0x01) ev.POWER_BUTTON_SHORT_CAN = true;
        }
        break;
      }
      default:
        break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  EVENT GENERATION
// ═══════════════════════════════════════════════════════════════════
void generateEvents() {
  bool stationary = (abs(vehicleSpeedKph) < SPEED_THRESHOLD_KPH);
  bool driving    = !stationary;
  bool speedAbove5 = !stationary;

  // Speed crossing events (edge-triggered)
  if (speedAbove5 && !prevSpeedAbove5) {
    ev.VEHICLE_SPEED_ABOVE_5KPH = true;
    ev.DRIVING = true;
  }
  if (!speedAbove5 && prevSpeedAbove5) {
    ev.VEHICLE_SPEED_BELOW_5KPH = true;
    ev.STATIONNARY = true;
  }
  prevSpeedAbove5 = speedAbove5;

  // --- Mapping button ---
  if (btnMap.shortRelease) ev.BTN_MAPPING_SHORT = true;
  if (btnMap.longRelease)  ev.BTN_MAPPING_LONG  = true;

  // --- Cabin light button ---
  if ((btnCablight.shortRelease || btnCablight.longRelease) && stationary)
    ev.BTN_CABLIGHT_AND_STATIONNARY = true;
  if ((btnCablight.shortRelease || btnCablight.longRelease) && driving)
    ev.BTN_CABLIGHT_AND_DRIVING = true;
  if (btnCablight.released)
    ev.BTN_CABLIGHT_RELEASED = true;

  // --- Trunk button → cabin light trunk trigger ---
  if ((btnTrunk.shortRelease || btnTrunk.longRelease) && stationary)
    ev.BTN_TRUNK_AND_STATIONNARY = true;

  // --- Trunk FSM events ---
  if (btnTrunk.released && stationary) ev.BTN_TRUNK_RELEASED_AND_STATIONNARY = true;
  if (btnTrunk.released && driving)    ev.BTN_TRUNK_RELEASED_AND_DRIVING     = true;

  // --- Fog ---
  if (btnBrouillard.shortRelease) ev.BTN_BROUILLARD_SHORT = true;
  if (btnBrouillard.longRelease)  ev.BTN_BROUILLARD_LONG  = true;

  // --- Seat sensor ---
  if (btnSeat.pressed)  ev.SEATSENSOR_PRESSED  = true;
  if (btnSeat.released) ev.SEATSENSOR_RELEASED = true;

  // --- Defroster ---
  bool battOk = (batterySocPct > 30);
  if (btnDefrost.shortRelease) {
    ev.BTN_DEFROSTER_SHORT = true;
    if (battOk) ev.BTN_DEFROSTER_SHORT_AND_BATTERY_OK = true;
  }
  if (btnDefrost.longRelease) {
    ev.BTN_DEFROSTER_LONG = true;
    if (battOk) ev.BTN_DEFROSTER_LONG_AND_BATTERY_OK = true;
  }
  if (batterySocPct <= 30 && fsmDefroster == ST_FSM_DEFROSTER_ON) {
    ev.BATTERY_BELOW_30_PERCENT = true;
  }

  // --- Warning ---
  if (btnWarning.released) ev.BTN_WARNING_RELEASED = true;

  // --- Forward / Reverse (drive direction) ---
  if (btnForward.shortRelease) ev.BTN_FORWARD_SHORT = true;
  if (btnForward.longRelease)  ev.BTN_FORWARD_LONG  = true;
  if (btnReverse.shortRelease) ev.BTN_REVERSE_SHORT = true;
  if (btnReverse.longRelease)  ev.BTN_REVERSE_LONG  = true;

  // --- Brake composite ---
  bool bl = btnBrakeLeft.raw;
  bool br = btnBrakeRight.raw;
  if (!bl && !br) ev.BRAKE_NONE       = true;
  if ( bl && !br) ev.BRAKE_LEFT_ONLY  = true;
  if (!bl &&  br) ev.BRAKE_RIGHT_ONLY = true;
  if ( bl &&  br) ev.BRAKE_BOTH       = true;

  // --- Timer timeout → events ---
  if (timerExpired(tmrMapping))        ev.MAPPING_BUTTON_TIMEOUT          = true;
  if (timerExpired(tmrCablightLong))   ev.CABINLIGHT_TIMER_LONG_TIMEOUT   = true;
  if (timerExpired(tmrCablightShort))  ev.CABINLIGHT_TIMER_SHORT_TIMEOUT  = true;
  if (timerExpired(tmrTrunkAction))    ev.TRUNK_ACTION_TIMER_TIMEOUT      = true;
  if (timerExpired(tmrTrunkBlink))     ev.TRUNK_BLINK_TIMER_TIMEOUT       = true;
  if (timerExpired(tmrTrunkError))     ev.TRUNK_ERROR_TIMER_TIMEOUT       = true;
  if (timerExpired(tmrFogOn))          ev.FOG_ON_TIMER_TIMEOUT            = true;
  if (timerExpired(tmrDefrosterOn))    ev.DEFROSTER_ON_TIMER_TIMEOUT      = true;
  if (timerExpired(tmrDefrosterFault)) ev.DEFROSTER_FAULT_TIMER_TIMEOUT   = true;
  if (timerExpired(tmrDefrosterBlink)) ev.DEFROSTER_BLINK_TIMER_TIMEOUT   = true;
  if (timerExpired(tmrBlinkerCycle))   ev.BLINKER_CYCLE_TIMEOUT           = true;
  if (timerExpired(tmrWarningCycle))   ev.WARNING_CYCLE_TIMEOUT           = true;
  if (timerExpired(tmrDriveChange))    ev.DRIVE_CHANGE_TIMER_TIMEOUT      = true;
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: MAPPING BUTTON
//  ST_MAPPING_CLEAR → ST_MAPPING_PRESSED on short/long press
//  ST_MAPPING_PRESSED → ST_MAPPING_CLEAR on 2 s timeout
//  While PRESSED, byte 6 bit 4 of 0x5FF is set → SiliXcon map switch
// ═══════════════════════════════════════════════════════════════════
void fsmStepMapping() {
  switch (fsmMapping) {
    case ST_MAPPING_CLEAR:
      if (ev.BTN_MAPPING_SHORT || ev.BTN_MAPPING_LONG) {
        fsmMapping = ST_MAPPING_PRESSED;
        timerStart(tmrMapping, 2000);
      }
      break;
    case ST_MAPPING_PRESSED:
      if (ev.MAPPING_BUTTON_TIMEOUT) {
        fsmMapping = ST_MAPPING_CLEAR;
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: CABIN LIGHT
// ═══════════════════════════════════════════════════════════════════
void fsmStepCabinLight() {
  switch (fsmCabinLight) {
    case ST_CABINLIGHT_OFF:
      if (ev.BTN_CABLIGHT_AND_STATIONNARY) {
        fsmCabinLight = ST_CABINLIGHT_ON;
        timerStart(tmrCablightLong, 3600000UL);   // 60 min
      } else if (ev.BTN_CABLIGHT_AND_DRIVING) {
        fsmCabinLight = ST_CABINLIGHT_ON_DRV;
      } else if (ev.BTN_TRUNK_AND_STATIONNARY) {
        fsmCabinLight = ST_CABINLIGHT_ON_TRUNK;
        timerStart(tmrCablightShort, 300000UL);   // 5 min
      }
      break;

    case ST_CABINLIGHT_ON:
      if (ev.BTN_CABLIGHT_RELEASED || ev.CABINLIGHT_TIMER_LONG_TIMEOUT || ev.DRIVING) {
        fsmCabinLight = ST_CABINLIGHT_OFF;
        timerStop(tmrCablightLong);
      }
      break;

    case ST_CABINLIGHT_ON_DRV:
      if (ev.STATIONNARY) {
        fsmCabinLight = ST_CABINLIGHT_ON;
        timerStart(tmrCablightLong, 3600000UL);
      } else if (ev.BTN_CABLIGHT_RELEASED) {
        fsmCabinLight = ST_CABINLIGHT_OFF;
      }
      break;

    case ST_CABINLIGHT_ON_TRUNK:
      if (ev.BTN_CABLIGHT_RELEASED || ev.CABINLIGHT_TIMER_SHORT_TIMEOUT) {
        fsmCabinLight = ST_CABINLIGHT_OFF;
        timerStop(tmrCablightShort);
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: TRUNK
// ═══════════════════════════════════════════════════════════════════
void fsmStepTrunk() {
  switch (fsmTrunk) {
    case ST_TRUNK_LATCHED:
      if (ev.BTN_TRUNK_RELEASED_AND_STATIONNARY) {
        fsmTrunk = ST_TRUNK_OPENING;
        timerStart(tmrTrunkAction, 2000);
      } else if (ev.BTN_TRUNK_RELEASED_AND_DRIVING) {
        fsmTrunk = ST_TRUNK_BLINK_ON;
        timerStart(tmrTrunkError, 3000);
        timerStart(tmrTrunkBlink, 400);
      }
      break;

    case ST_TRUNK_OPENING:
      if (ev.TRUNK_ACTION_TIMER_TIMEOUT) fsmTrunk = ST_TRUNK_LATCHED;
      break;

    case ST_TRUNK_BLINK_ON:
      if (ev.TRUNK_ERROR_TIMER_TIMEOUT) { fsmTrunk = ST_TRUNK_LATCHED; break; }
      if (ev.TRUNK_BLINK_TIMER_TIMEOUT) { fsmTrunk = ST_TRUNK_BLINK_OFF; timerStart(tmrTrunkBlink, 400); }
      break;

    case ST_TRUNK_BLINK_OFF:
      if (ev.TRUNK_ERROR_TIMER_TIMEOUT) { fsmTrunk = ST_TRUNK_LATCHED; break; }
      if (ev.TRUNK_BLINK_TIMER_TIMEOUT) { fsmTrunk = ST_TRUNK_BLINK_ON;  timerStart(tmrTrunkBlink, 400); }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: FOG LIGHT
// ═══════════════════════════════════════════════════════════════════
void fsmStepFog() {
  switch (fsmFog) {
    case ST_FSM_FOG_OFF:
      if (ev.BTN_BROUILLARD_SHORT || ev.BTN_BROUILLARD_LONG) {
        fsmFog = ST_FSM_FOG_ON;
      }
      break;

    case ST_FSM_FOG_ON:
      if (ev.BTN_BROUILLARD_SHORT || ev.BTN_BROUILLARD_LONG) {
        fsmFog = ST_FSM_FOG_OFF;
      } else if (ev.SEATSENSOR_RELEASED) {
        fsmFog = ST_FSM_FOG_ON_GRACE;
        timerStart(tmrFogOn, 120000UL);   // 2 min
      }
      break;

    case ST_FSM_FOG_ON_GRACE:
      if (ev.SEATSENSOR_PRESSED) {
        fsmFog = ST_FSM_FOG_ON;
        timerStop(tmrFogOn);
      } else if (ev.FOG_ON_TIMER_TIMEOUT) {
        fsmFog = ST_FSM_FOG_OFF;
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: DEFROSTER
// ═══════════════════════════════════════════════════════════════════
void fsmStepDefroster() {
  switch (fsmDefroster) {
    case ST_FSM_DEFROSTER_OFF:
      if (ev.BTN_DEFROSTER_SHORT_AND_BATTERY_OK || ev.BTN_DEFROSTER_LONG_AND_BATTERY_OK) {
        fsmDefroster = ST_FSM_DEFROSTER_ON;
        timerStart(tmrDefrosterOn, 300000UL);   // 5 min
      }
      break;

    case ST_FSM_DEFROSTER_ON:
      if (ev.BTN_DEFROSTER_SHORT || ev.BTN_DEFROSTER_LONG || ev.DEFROSTER_ON_TIMER_TIMEOUT) {
        fsmDefroster = ST_FSM_DEFROSTER_OFF;
        timerStop(tmrDefrosterOn);
      } else if (ev.BATTERY_BELOW_30_PERCENT || ev.DEFROSTER_FAULT) {
        fsmDefroster = ST_FSM_DEFROSTER_FAULT_BLINK_ON;
        timerStart(tmrDefrosterFault, 3000);
        timerStart(tmrDefrosterBlink, 400);
        timerStop(tmrDefrosterOn);
      }
      break;

    case ST_FSM_DEFROSTER_FAULT_BLINK_ON:
      if (ev.DEFROSTER_FAULT_TIMER_TIMEOUT) { fsmDefroster = ST_FSM_DEFROSTER_OFF; break; }
      if (ev.DEFROSTER_BLINK_TIMER_TIMEOUT) {
        fsmDefroster = ST_FSM_DEFROSTER_FAULT_BLINK_OFF;
        timerStart(tmrDefrosterBlink, 400);
      }
      break;

    case ST_FSM_DEFROSTER_FAULT_BLINK_OFF:
      if (ev.DEFROSTER_FAULT_TIMER_TIMEOUT) { fsmDefroster = ST_FSM_DEFROSTER_OFF; break; }
      if (ev.DEFROSTER_BLINK_TIMER_TIMEOUT) {
        fsmDefroster = ST_FSM_DEFROSTER_FAULT_BLINK_ON;
        timerStart(tmrDefrosterBlink, 400);
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: BLINKER / WARNING
// ═══════════════════════════════════════════════════════════════════
void fsmStepBlinker() {
  switch (fsmBlinker) {
    case ST_BLINKER_IDLE:
      if (ev.BTN_WARNING_RELEASED) {
        fsmBlinker = ST_WARNING_ON;
        timerStart(tmrWarningCycle, 125);
      }
      if (ev.BTN_BLINKER_LEFT_RELEASED) {
        fsmBlinker = ST_BLINKER_LEFT_ON;
        timerStart(tmrBlinkerCycle, 400);
      }
      if (ev.BTN_BLINKER_RIGHT_RELEASED) {
        fsmBlinker = ST_BLINKER_RIGHT_ON;
        timerStart(tmrBlinkerCycle, 400);
      }
      break;

    // --- WARNING ---
    case ST_WARNING_ON:
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrWarningCycle); break; }
      if (ev.WARNING_CYCLE_TIMEOUT) { fsmBlinker = ST_WARNING_OFF; timerStart(tmrWarningCycle, 125); }
      break;
    case ST_WARNING_OFF:
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrWarningCycle); break; }
      if (ev.WARNING_CYCLE_TIMEOUT) { fsmBlinker = ST_WARNING_ON;  timerStart(tmrWarningCycle, 125); }
      break;

    // --- LEFT ---
    case ST_BLINKER_LEFT_ON:
      if (ev.BTN_BLINKER_LEFT_RELEASED || ev.SEATSENSOR_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrBlinkerCycle); break; }
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_WARNING_ON; timerStart(tmrWarningCycle, 125); break; }
      if (ev.BLINKER_CYCLE_TIMEOUT) { fsmBlinker = ST_BLINKER_LEFT_OFF; timerStart(tmrBlinkerCycle, 400); }
      break;
    case ST_BLINKER_LEFT_OFF:
      if (ev.BTN_BLINKER_LEFT_RELEASED || ev.SEATSENSOR_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrBlinkerCycle); break; }
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_WARNING_ON; timerStart(tmrWarningCycle, 125); break; }
      if (ev.BLINKER_CYCLE_TIMEOUT) { fsmBlinker = ST_BLINKER_LEFT_ON;  timerStart(tmrBlinkerCycle, 400); }
      break;

    // --- RIGHT ---
    case ST_BLINKER_RIGHT_ON:
      if (ev.BTN_BLINKER_RIGHT_RELEASED || ev.SEATSENSOR_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrBlinkerCycle); break; }
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_WARNING_ON; timerStart(tmrWarningCycle, 125); break; }
      if (ev.BLINKER_CYCLE_TIMEOUT) { fsmBlinker = ST_BLINKER_RIGHT_OFF; timerStart(tmrBlinkerCycle, 400); }
      break;
    case ST_BLINKER_RIGHT_OFF:
      if (ev.BTN_BLINKER_RIGHT_RELEASED || ev.SEATSENSOR_RELEASED) { fsmBlinker = ST_BLINKER_IDLE; timerStop(tmrBlinkerCycle); break; }
      if (ev.BTN_WARNING_RELEASED) { fsmBlinker = ST_WARNING_ON; timerStart(tmrWarningCycle, 125); break; }
      if (ev.BLINKER_CYCLE_TIMEOUT) { fsmBlinker = ST_BLINKER_RIGHT_ON;  timerStart(tmrBlinkerCycle, 400); }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: DRIVE DIRECTION
// ═══════════════════════════════════════════════════════════════════
void fsmStepDrive() {
  switch (fsmDrive) {
    case ST_DRIVE_NEUTRAL:
      if (ev.BTN_FORWARD_SHORT || ev.BTN_FORWARD_LONG) { fsmDrive = ST_DRIVE_FORWARD; break; }
      if (ev.BTN_REVERSE_SHORT || ev.BTN_REVERSE_LONG) { fsmDrive = ST_DRIVE_REVERSE; break; }
      break;

    // --- FORWARD ---
    case ST_DRIVE_FORWARD:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_ABOVE_5KPH)                  { fsmDrive = ST_DRIVE_FORWARD_LOCK; break; }
      if (ev.BTN_REVERSE_SHORT || ev.BTN_REVERSE_LONG)  { fsmDrive = ST_DRIVE_REVERSE; break; }
      break;

    case ST_DRIVE_FORWARD_LOCK:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_BELOW_5KPH)                  { fsmDrive = ST_DRIVE_FORWARD; break; }
      if (ev.BTN_REVERSE_SHORT || ev.BTN_REVERSE_LONG)  { fsmDrive = ST_DRIVE_REV_PENDING; timerStart(tmrDriveChange, 5000); break; }
      break;

    case ST_DRIVE_FWD_PENDING:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_BELOW_5KPH)                  { fsmDrive = ST_DRIVE_FORWARD; break; }
      if (ev.DRIVE_CHANGE_TIMER_TIMEOUT)                { fsmDrive = ST_DRIVE_REVERSE_LOCK; break; }
      if (ev.BTN_REVERSE_SHORT || ev.BTN_REVERSE_LONG)  { fsmDrive = ST_DRIVE_REVERSE_LOCK; break; }
      break;

    // --- REVERSE ---
    case ST_DRIVE_REVERSE:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_ABOVE_5KPH)                  { fsmDrive = ST_DRIVE_REVERSE_LOCK; break; }
      if (ev.BTN_FORWARD_SHORT || ev.BTN_FORWARD_LONG)  { fsmDrive = ST_DRIVE_FORWARD; break; }
      break;

    case ST_DRIVE_REVERSE_LOCK:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_BELOW_5KPH)                  { fsmDrive = ST_DRIVE_REVERSE; break; }
      if (ev.BTN_FORWARD_SHORT || ev.BTN_FORWARD_LONG)  { fsmDrive = ST_DRIVE_FWD_PENDING; timerStart(tmrDriveChange, 5000); break; }
      break;

    case ST_DRIVE_REV_PENDING:
      if (ev.SEATSENSOR_RELEASED)                       { fsmDrive = ST_DRIVE_NEUTRAL; break; }
      if (ev.VEHICLE_SPEED_BELOW_5KPH)                  { fsmDrive = ST_DRIVE_REVERSE; break; }
      if (ev.DRIVE_CHANGE_TIMER_TIMEOUT)                { fsmDrive = ST_DRIVE_FORWARD_LOCK; break; }
      if (ev.BTN_FORWARD_SHORT || ev.BTN_FORWARD_LONG)  { fsmDrive = ST_DRIVE_FORWARD_LOCK; break; }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: BRAKE
// ═══════════════════════════════════════════════════════════════════
void fsmStepBrake() {
  if (ev.BRAKE_NONE)       fsmBrake = ST_BRAKE_LEVEL_0;
  if (ev.BRAKE_LEFT_ONLY)  fsmBrake = ST_BRAKE_LEVEL_25;
  if (ev.BRAKE_RIGHT_ONLY) fsmBrake = ST_BRAKE_LEVEL_50;
  if (ev.BRAKE_BOTH)       fsmBrake = ST_BRAKE_LEVEL_100;
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: POWER
// ═══════════════════════════════════════════════════════════════════
void fsmStepPower() {
  bool pwrEvt = ev.POWER_BUTTON_SHORT_CAN || ev.POWER_BUTTON_LONG_CAN || ev.BTN_FORWARD_SHORT || ev.BTN_FORWARD_LONG || ev.BTN_REVERSE_SHORT || ev.BTN_REVERSE_LONG;

  switch (fsmPower) {
    case ST_POWER_OFF:
      if (pwrEvt) fsmPower = ST_POWER_ON;
      break;
    case ST_POWER_ON:
      if (pwrEvt) { fsmPower = ST_POWER_OFF; break; }
      if (ev.SEATSENSOR_RELEASED) { fsmPower = ST_POWER_ON_GRACE; timerStart(tmrPwrIdle, 3600000UL); }
      break;
    case ST_POWER_ON_GRACE:
      if (timerExpired(tmrPwrIdle))  { fsmPower = ST_POWER_OFF; break; }
      if (ev.SEATSENSOR_PRESSED)     { fsmPower = ST_POWER_ON; timerStop(tmrPwrIdle); }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: DRL
// ═══════════════════════════════════════════════════════════════════
void fsmStepDrl() {
  bool pwrEvt = ev.POWER_BUTTON_SHORT_CAN || ev.POWER_BUTTON_LONG_CAN;

  switch (fsmDrl) {
    case ST_DRL_OFF:
      if (pwrEvt) fsmDrl = ST_DRL_ON;
      break;
    case ST_DRL_ON:
      if (pwrEvt) { fsmDrl = ST_DRL_OFF; break; }
      if (ev.SEATSENSOR_RELEASED) { fsmDrl = ST_DRL_ON_GRACE; timerStart(tmrDrlIdle, 3600000UL); }
      break;
    case ST_DRL_ON_GRACE:
      if (timerExpired(tmrDrlIdle)) { fsmDrl = ST_DRL_OFF; break; }
      if (ev.SEATSENSOR_PRESSED)    { fsmDrl = ST_DRL_ON; timerStop(tmrDrlIdle); }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  FSM: LOW BEAM
// ═══════════════════════════════════════════════════════════════════
void fsmStepLowbeam() {
  bool pwrEvt = ev.POWER_BUTTON_SHORT_CAN || ev.POWER_BUTTON_LONG_CAN;

  switch (fsmLowbeam) {
    case ST_LOWBEAM_OFF:
      if (ev.DRIVING) fsmLowbeam = ST_LOWBEAM_ON;
      break;
    case ST_LOWBEAM_ON:
      if (pwrEvt)         { fsmLowbeam = ST_LOWBEAM_OFF; break; }
      if (ev.STATIONNARY) { fsmLowbeam = ST_LOWBEAM_ON_GRACE; timerStart(tmrLowbeamIdle, 300000UL); }
      break;
    case ST_LOWBEAM_ON_GRACE:
      if (pwrEvt)                       { fsmLowbeam = ST_LOWBEAM_OFF; timerStop(tmrLowbeamIdle); break; }
      if (ev.DRIVING)                   { fsmLowbeam = ST_LOWBEAM_ON;  timerStop(tmrLowbeamIdle); break; }
      if (timerExpired(tmrLowbeamIdle)) { fsmLowbeam = ST_LOWBEAM_OFF; }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════
//  OUTPUT STAGE — apply FSM states to hardware
// ═══════════════════════════════════════════════════════════════════
void applyOutputs() {
  // ---- POWER MOSFET (MCP1 GPA3): always ON ----
  if (mcp1_ok) mcp1.digitalWrite(MCP1_POWER_GATE, HIGH);

  // ---- CABIN LIGHT (ESP GPIO 15 + MCP1 GPB1 CABLIGHT_LED) ----
  bool cabOn = (fsmCabinLight != ST_CABINLIGHT_OFF);
  digitalWrite(INTERRIOR_LIGHT_MOSFET_GATE, cabOn ? HIGH : LOW);
  if (mcp1_ok) mcp1.digitalWrite(MCP1_CABLIGHT_LED, cabOn ? HIGH : LOW);

  // ---- TRUNK ----
  if (mcp1_ok) {
    bool trunkCoil = (fsmTrunk == ST_TRUNK_OPENING);
    bool trunkLed  = (fsmTrunk == ST_TRUNK_OPENING || fsmTrunk == ST_TRUNK_BLINK_ON);
    mcp1.digitalWrite(MCP1_LATCH_TRUNK_GATE, trunkCoil ? HIGH : LOW);
    mcp1.digitalWrite(MCP1_TRUNK_LED,        trunkLed  ? HIGH : LOW);
  }

  // ---- FOG ----
  if (mcp1_ok) {
    bool fogOn = (fsmFog == ST_FSM_FOG_ON || fsmFog == ST_FSM_FOG_ON_GRACE);
    mcp1.digitalWrite(MCP1_BROUILLARD_LED, fogOn ? HIGH : LOW);
  }

  // ---- DEFROSTER ----
  {
    bool defMosfet = (fsmDefroster == ST_FSM_DEFROSTER_ON);
    bool defLed    = (fsmDefroster == ST_FSM_DEFROSTER_ON ||
                      fsmDefroster == ST_FSM_DEFROSTER_FAULT_BLINK_ON);
    digitalWrite(DEFROSTER_MOSFET_GATE, defMosfet ? HIGH : LOW);
    if (mcp1_ok) mcp1.digitalWrite(MCP1_DEFROST_LED, defLed ? HIGH : LOW);
  }

  // ---- MAP LED (on during PRESSED state) ----
  if (mcp1_ok) mcp1.digitalWrite(MCP1_MAP_LED, (fsmMapping == ST_MAPPING_PRESSED) ? HIGH : LOW);

  // ---- WARNING LED ----
  if (mcp1_ok) {
    bool warnLedOn = (fsmBlinker == ST_WARNING_ON);
    mcp1.digitalWrite(MCP1_WARNING_LED, warnLedOn ? HIGH : LOW);
  }

  // ---- BLINKER MOSFETS ----
  {
    bool blinkLeftOn  = (fsmBlinker == ST_BLINKER_LEFT_ON  || fsmBlinker == ST_WARNING_ON);
    bool blinkRightOn = (fsmBlinker == ST_BLINKER_RIGHT_ON || fsmBlinker == ST_WARNING_ON);
    digitalWrite(BLIKER_LEFT_MOSFET_GATE,  blinkLeftOn  ? HIGH : LOW);
    digitalWrite(BLIKER_RIGHT_MOSFET_GATE, blinkRightOn ? HIGH : LOW);
  }

  // ---- DRL ----
  {
    bool drlOn = (fsmDrl == ST_DRL_ON || fsmDrl == ST_DRL_ON_GRACE);
    digitalWrite(DRL_LEFT_MOSFET_GATE,  drlOn ? HIGH : LOW);
    digitalWrite(DRL_RIGHT_MOSFET_GATE, drlOn ? HIGH : LOW);
  }
}

// ═══════════════════════════════════════════════════════════════════
//  CAN TX: 0x5FF — Control Input Message (every 50 ms)
//
//  Byte 0-1: INT16 CAN Level 1 (32767=invalid)   → throttle or unused
//  Byte 2-3: INT16 CAN Level 2 (32767=invalid)
//  Byte 4-5: INT16 CAN Level 3 (32767=invalid)
//  Byte 6:   UINT8 digital inputs & map switching
//            bits 0-3: digital in 0-3 (IDs 10-13,255)
//            bits 4-7: map switch     (IDs 20-23,255)
//  Byte 7:   UINT8 commands  bit 0 = disarm/seatswitch
//
//  Map switching: we use "2x" auto-pulse — set bit 4 while
//  fsmMapping == ST_MAPPING_PRESSED, controller triggers map change.
// ═══════════════════════════════════════════════════════════════════
void sendCanControl() {
  uint32_t now = millis();
  if (now - lastCanTx < CAN_TX_INTERVAL) return;
  lastCanTx = now;

  // Build byte 6
  canDigIn = 0;
  if (btnSeat.raw) canDigIn |= 0x01;                        // digital in 0 = seat
  if (fsmMapping == ST_MAPPING_PRESSED) canDigIn |= 0x10;   // map switch pulse (bit 4)

  // Build byte 7
  canCmd = 0;
  if (!btnSeat.raw) canCmd |= 0x01;  // disarm when seat released

  // Pack (little-endian)
  twai_message_t msg;
  msg.identifier       = CAN_ID_CONTROL;
  msg.data_length_code = 8;
  msg.flags            = 0;
  msg.data[0] = (uint8_t)(canLevel1 & 0xFF);
  msg.data[1] = (uint8_t)((canLevel1 >> 8) & 0xFF);
  msg.data[2] = (uint8_t)(canLevel2 & 0xFF);
  msg.data[3] = (uint8_t)((canLevel2 >> 8) & 0xFF);
  msg.data[4] = (uint8_t)(canLevel3 & 0xFF);
  msg.data[5] = (uint8_t)((canLevel3 >> 8) & 0xFF);
  msg.data[6] = canDigIn;
  msg.data[7] = canCmd;

  twai_transmit(&msg, pdMS_TO_TICKS(5));
}

// ═══════════════════════════════════════════════════════════════════
//  CAN TX: Mainboard broadcast (0x420-0x423, every 100 ms)
// ═══════════════════════════════════════════════════════════════════
void sendCanBroadcast() {
  uint32_t now = millis();
  if (now - lastCanBroadcast < CAN_BROADCAST_MS) return;
  lastCanBroadcast = now;

  twai_message_t msg;
  msg.flags = 0;

  // --- 0x420: Lights ---
  msg.identifier       = CAN_ID_MB_LIGHTS;
  msg.data_length_code = 4;
  msg.data[0] = (fsmCabinLight != ST_CABINLIGHT_OFF) ? 1 : 0;
  msg.data[1] = (fsmFog != ST_FSM_FOG_OFF)           ? 1 : 0;
  msg.data[2] = (fsmDrl != ST_DRL_OFF)               ? 1 : 0;
  msg.data[3] = (fsmLowbeam != ST_LOWBEAM_OFF)       ? 1 : 0;
  twai_transmit(&msg, pdMS_TO_TICKS(5));

  // --- 0x421: Blinker ---
  msg.identifier       = CAN_ID_MB_BLINKER;
  msg.data_length_code = 3;
  msg.data[0] = (uint8_t)fsmBlinker;
  msg.data[1] = (fsmBlinker == ST_BLINKER_LEFT_ON  || fsmBlinker == ST_WARNING_ON) ? 1 : 0;
  msg.data[2] = (fsmBlinker == ST_BLINKER_RIGHT_ON || fsmBlinker == ST_WARNING_ON) ? 1 : 0;
  twai_transmit(&msg, pdMS_TO_TICKS(5));

  // --- 0x422: Drive direction ---
  msg.identifier       = CAN_ID_MB_DRIVE;
  msg.data_length_code = 1;
  switch (fsmDrive) {
    case ST_DRIVE_FORWARD: case ST_DRIVE_FORWARD_LOCK: case ST_DRIVE_REV_PENDING:
      msg.data[0] = 1; break;   // forward
    case ST_DRIVE_REVERSE: case ST_DRIVE_REVERSE_LOCK: case ST_DRIVE_FWD_PENDING:
      msg.data[0] = 2; break;   // reverse
    default:
      msg.data[0] = 0; break;   // neutral
  }
  twai_transmit(&msg, pdMS_TO_TICKS(5));

  // --- 0x423: Brake / regen level ---
  msg.identifier       = CAN_ID_MB_BRAKE;
  msg.data_length_code = 1;
  switch (fsmBrake) {
    case ST_BRAKE_LEVEL_0:   msg.data[0] = 0;   break;
    case ST_BRAKE_LEVEL_25:  msg.data[0] = 25;  break;
    case ST_BRAKE_LEVEL_50:  msg.data[0] = 50;  break;
    case ST_BRAKE_LEVEL_100: msg.data[0] = 100; break;
  }
  twai_transmit(&msg, pdMS_TO_TICKS(5));
}

// ═══════════════════════════════════════════════════════════════════
//  SENSOR PRINT — every 5 s on serial (INA, IMU, CAN state, FSMs)
// ═══════════════════════════════════════════════════════════════════
void printSensors() {
  uint32_t now = millis();
  if (now - lastSensorPrint < SENSOR_PRINT_MS) return;
  lastSensorPrint = now;

  Serial.println("──── Sensor Dump ────");

  // INA228 current sensors
  if (inaMain_ok) {
    float v = inaMain.readBusVoltage();
    float a = inaMain.readCurrent();
    Serial.printf("[INA 0x40 TOTAL]  V=%.2f V  I=%.3f A\n", v / 1000.0f, a / 1000.0f);
  }
  if (inaHand_ok) {
    float v = inaHand.readBusVoltage();
    float a = inaHand.readCurrent();
    Serial.printf("[INA 0x41 HAND ]  V=%.2f V  I=%.3f A\n", v / 1000.0f, a / 1000.0f);
  }
  if (inaSeat_ok) {
    float v = inaSeat.readBusVoltage();
    float a = inaSeat.readCurrent();
    Serial.printf("[INA 0x44 SEAT ]  V=%.2f V  I=%.3f A\n", v / 1000.0f, a / 1000.0f);
  }
  if (inaDef_ok) {
    float v = inaDef.readBusVoltage();
    float a = inaDef.readCurrent();
    Serial.printf("[INA 0x45 DEFR ]  V=%.2f V  I=%.3f A\n", v / 1000.0f, a / 1000.0f);
  }

  // MPU6050 + BMP280
  if (mpu_ok) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    Serial.printf("[MPU 0x68] Acc  X=%.2f Y=%.2f Z=%.2f m/s2\n",
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    Serial.printf("[MPU 0x68] Gyro X=%.2f Y=%.2f Z=%.2f rad/s\n",
                  gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
  }
  if (bmp_ok) {
    Serial.printf("[BMP 0x76] T=%.1f C  P=%.1f hPa  Alt=%.1f m\n",
                  bmp.readTemperature(),
                  bmp.readPressure() / 100.0f,
                  bmp.readAltitude(1013.25f));
  }

  // CAN-derived state
  Serial.printf("[CAN] speed=%d kph  SOC=%d%%  Vbat=%.1fV  Ibat=%.1fA  map=%d  mode=%d\n",
                vehicleSpeedKph, batterySocPct,
                batteryVoltage01V * 0.01f,
                batteryCurrent02A * 0.02f,
                currentMap, lynxMode);

  // FSM states
  Serial.printf("[FSM] pwr=%d drl=%d low=%d fog=%d def=%d cab=%d trk=%d blk=%d drv=%d brk=%d map=%d\n",
                fsmPower, fsmDrl, fsmLowbeam, fsmFog, fsmDefroster,
                fsmCabinLight, fsmTrunk, fsmBlinker, fsmDrive, fsmBrake, fsmMapping);

  Serial.println("─────────────────────");
}

// ═══════════════════════════════════════════════════════════════════
//  CLEAR BUTTON ONE-SHOTS (after event consumption)
// ═══════════════════════════════════════════════════════════════════
void clearButtonOneShots() {
  btnReset(btnMap);
  btnReset(btnTrunk);
  btnReset(btnDefrost);
  btnReset(btnCablight);
  btnReset(btnBrouillard);
  btnReset(btnWarning);
  btnReset(btnForward);
  btnReset(btnReverse);
  btnReset(btnSeat);
  btnReset(btnBrakeLeft);
  btnReset(btnBrakeRight);
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP  (target ≥ 1 kHz)
// ═══════════════════════════════════════════════════════════════════
void loop() {
  // 1. Clear events from previous cycle
  clearEvents();

  // 2. INPUT — read hardware
  readButtons();
  readCAN();

  // 3. EVENT — derive discrete events from state + edges
  generateEvents();

  // 4. FSM — step every state machine
  fsmStepMapping();
  fsmStepCabinLight();
  fsmStepTrunk();
  fsmStepFog();
  fsmStepDefroster();
  fsmStepBlinker();
  fsmStepDrive();
  fsmStepBrake();
  fsmStepPower();
  fsmStepDrl();
  fsmStepLowbeam();

  // 5. OUTPUT — write hardware
  applyOutputs();
  sendCanControl();
  sendCanBroadcast();

  // 6. DIAGNOSTICS — periodic sensor print
  printSensors();

  // 7. Clear one-shot flags
  clearButtonOneShots();

  // Minimal yield — I2C reads are the bottleneck (~1 ms per MCP transaction)
  delay(1);
}
