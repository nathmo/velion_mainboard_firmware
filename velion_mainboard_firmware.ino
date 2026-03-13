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
 *   - AceButton (by Brian T. Park)
 *   - ESP32 board package v2.0.17 (includes driver/twai.h)
 *   - WiFi + WebServer (built-in ESP32)
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_INA228.h>
#include <AceButton.h>
#include "driver/twai.h"

using namespace ace_button;

// ═══════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS (required for Arduino IDE prototype generation)
// ═══════════════════════════════════════════════════════════════════
struct BtnEvents;
struct SoftTimer;
struct Events;
struct SonarChannel;
void bevReset(BtnEvents &b);
void timerStart(SoftTimer &t, uint32_t ms);
void timerStop(SoftTimer &t);
bool timerExpired(SoftTimer &t);

// FSM enum forward declarations (must be before auto-prototypes)
enum MappingState    { ST_MAPPING_CLEAR, ST_MAPPING_PRESSED };
enum CabinLightState { ST_CABINLIGHT_OFF, ST_CABINLIGHT_ON,
                        ST_CABINLIGHT_ON_TRUNK, ST_CABINLIGHT_ON_DRV };
enum TrunkState      { ST_TRUNK_LATCHED, ST_TRUNK_OPENING,
                        ST_TRUNK_BLINK_ON, ST_TRUNK_BLINK_OFF };
enum FogState        { ST_FSM_FOG_OFF, ST_FSM_FOG_ON, ST_FSM_FOG_ON_GRACE };
enum DefrosterState  { ST_FSM_DEFROSTER_OFF, ST_FSM_DEFROSTER_ON,
                        ST_FSM_DEFROSTER_FAULT_BLINK_OFF,
                        ST_FSM_DEFROSTER_FAULT_BLINK_ON };
enum BlinkerState    { ST_BLINKER_IDLE,
                        ST_WARNING_ON, ST_WARNING_OFF,
                        ST_BLINKER_LEFT_ON,  ST_BLINKER_LEFT_OFF,
                        ST_BLINKER_RIGHT_ON, ST_BLINKER_RIGHT_OFF };
enum DriveState      { ST_DRIVE_NEUTRAL,
                        ST_DRIVE_FORWARD, ST_DRIVE_FORWARD_LOCK, ST_DRIVE_FWD_PENDING,
                        ST_DRIVE_REVERSE, ST_DRIVE_REVERSE_LOCK, ST_DRIVE_REV_PENDING };
enum BrakeState      { ST_BRAKE_LEVEL_0, ST_BRAKE_LEVEL_25,
                        ST_BRAKE_LEVEL_50, ST_BRAKE_LEVEL_100 };
enum PowerState      { ST_POWER_OFF, ST_POWER_ON, ST_POWER_ON_GRACE };
enum DrlState        { ST_DRL_OFF, ST_DRL_ON, ST_DRL_ON_GRACE };
enum LowbeamState    { ST_LOWBEAM_OFF, ST_LOWBEAM_ON, ST_LOWBEAM_ON_GRACE };

// ═══════════════════════════════════════════════════════════════════
//  WIFI HOTSPOT & WEB SERVER CONFIGURATION
// ═══════════════════════════════════════════════════════════════════
#define WIFI_SSID     "Velion-Dashboard"
#define WIFI_PASSWORD "velion1234"

WebServer webServer(80);

// CAN packet log ring buffer for the web terminal
#define CAN_LOG_SIZE 200
String canLog[CAN_LOG_SIZE];
int    canLogHead = 0;
int    canLogCount = 0;

void canLogPush(const String &line) {
  canLog[canLogHead] = line;
  canLogHead = (canLogHead + 1) % CAN_LOG_SIZE;
  if (canLogCount < CAN_LOG_SIZE) canLogCount++;
}

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

// CAN (TWAI) pins
#define CAN_TX_PIN  GPIO_NUM_18
#define CAN_RX_PIN  GPIO_NUM_19

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
#define CAN_ID_CONTROL        0x5FF   // TX: control input message
#define CAN_ID_VDS_BTN_EVT    0x5FE   // RX: VDS display button event
#define CAN_ID_LYNX_STATUS    0x600   // RX: LYNX status
#define CAN_ID_MOTOR_STATUS   0x610   // RX: motor / vehicle speed
#define CAN_ID_BATTERY_STATUS 0x618   // RX: battery SOC/voltage/current
#define CAN_ID_THROTTLE_RX    0x407   // RX: throttle 8-bit ADC from control board
#define CAN_ID_LIGHT_CONTROL  0x400   // RX: Filovelox smart button light-control word

// CAN light-control bits (match lightboard protocol)
#define BIT_RIGHT_BLINK       3
#define BIT_LEFT_BLINK        4

// Mainboard broadcast range 0x420-0x480
#define CAN_ID_MB_LIGHTS     0x420   // TX: lighting state
#define CAN_ID_MB_BLINKER    0x421   // TX: blinker/warning state
#define CAN_ID_MB_DRIVE      0x422   // TX: drive direction (fwd/rev/neutral)
#define CAN_ID_MB_BRAKE      0x423   // TX: brake / regen level

// Speed threshold
#define SPEED_THRESHOLD_KPH  5

// ═══════════════════════════════════════════════════════════════════
//  ACEBUTTON — MCP23017 virtual buttons via manual injection
//
//  AceButton normally reads digitalRead(); for MCP-attached buttons
//  we subclass ButtonConfig to read from the MCP expanders, then
//  let AceButton do debounce + short/long press detection.
// ═══════════════════════════════════════════════════════════════════

// One-shot event flags produced by AceButton callbacks
struct BtnEvents {
  bool shortPress;
  bool longPress;
  bool pressed;
  bool released;
};

BtnEvents bevMap        = {};
BtnEvents bevTrunk      = {};
BtnEvents bevDefrost    = {};
BtnEvents bevCablight   = {};
BtnEvents bevBrouillard = {};
BtnEvents bevWarning    = {};
BtnEvents bevForward    = {};
BtnEvents bevReverse    = {};
BtnEvents bevSeat       = {};
BtnEvents bevBrakeLeft  = {};
BtnEvents bevBrakeRight = {};

void bevReset(BtnEvents &b) {
  b.shortPress = false;
  b.longPress  = false;
  b.pressed    = false;
  b.released   = false;
}

// ── MCP-aware ButtonConfig ──────────────────────────────────────
// Since buttons are on MCP expanders (not ESP32 GPIOs), we use
// checkState() for manual level injection. The pin parameter is
// unused; we use the AceButton `id` field to identify each button.

enum ButtonId : uint8_t {
  BTN_ID_MAP = 0,
  BTN_ID_TRUNK,
  BTN_ID_DEFROST,
  BTN_ID_CABLIGHT,
  BTN_ID_BROUILLARD,
  BTN_ID_WARNING,
  BTN_ID_FORWARD,
  BTN_ID_REVERSE,
  BTN_ID_SEAT,
  BTN_ID_BRAKE_LEFT,
  BTN_ID_BRAKE_RIGHT,
  BTN_ID_COUNT
};

class McpButtonConfig : public ButtonConfig {
  public:
    int readButton(uint8_t /*pin*/) override {
      // We don't use readButton(); we call AceButton::checkState() directly.
      return HIGH;  // default idle = not pressed
    }
};

McpButtonConfig mcpBtnConfig;

// Declare AceButton instances — pin=0 (unused), defaultReleased=HIGH, id=ButtonId
AceButton abMap       (&mcpBtnConfig, 0, HIGH, BTN_ID_MAP);
AceButton abTrunk     (&mcpBtnConfig, 0, HIGH, BTN_ID_TRUNK);
AceButton abDefrost   (&mcpBtnConfig, 0, HIGH, BTN_ID_DEFROST);
AceButton abCablight  (&mcpBtnConfig, 0, HIGH, BTN_ID_CABLIGHT);
AceButton abBrouillard(&mcpBtnConfig, 0, HIGH, BTN_ID_BROUILLARD);
AceButton abWarning   (&mcpBtnConfig, 0, HIGH, BTN_ID_WARNING);
AceButton abForward   (&mcpBtnConfig, 0, HIGH, BTN_ID_FORWARD);
AceButton abReverse   (&mcpBtnConfig, 0, HIGH, BTN_ID_REVERSE);
AceButton abSeat      (&mcpBtnConfig, 0, HIGH, BTN_ID_SEAT);
AceButton abBrakeLeft (&mcpBtnConfig, 0, HIGH, BTN_ID_BRAKE_LEFT);
AceButton abBrakeRight(&mcpBtnConfig, 0, HIGH, BTN_ID_BRAKE_RIGHT);

// Lookup table: id → BtnEvents*
BtnEvents* bevTable[BTN_ID_COUNT];

void initBevTable() {
  bevTable[BTN_ID_MAP]         = &bevMap;
  bevTable[BTN_ID_TRUNK]       = &bevTrunk;
  bevTable[BTN_ID_DEFROST]     = &bevDefrost;
  bevTable[BTN_ID_CABLIGHT]    = &bevCablight;
  bevTable[BTN_ID_BROUILLARD]  = &bevBrouillard;
  bevTable[BTN_ID_WARNING]     = &bevWarning;
  bevTable[BTN_ID_FORWARD]     = &bevForward;
  bevTable[BTN_ID_REVERSE]     = &bevReverse;
  bevTable[BTN_ID_SEAT]        = &bevSeat;
  bevTable[BTN_ID_BRAKE_LEFT]  = &bevBrakeLeft;
  bevTable[BTN_ID_BRAKE_RIGHT] = &bevBrakeRight;
}

// AceButton event handler (all buttons share this)
void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/) {
  uint8_t id = button->getId();
  if (id >= BTN_ID_COUNT) return;
  BtnEvents* bev = bevTable[id];
  if (!bev) return;

  switch (eventType) {
    case AceButton::kEventPressed:
      bev->pressed = true;
      break;
    case AceButton::kEventReleased:
      bev->released   = true;
      bev->shortPress = true;
      break;
    case AceButton::kEventLongPressed:
      bev->longPress = true;
      break;
    case AceButton::kEventLongReleased:
      bev->released  = true;
      bev->longPress = true;
      break;
  }
}

void setupAceButtons() {
  initBevTable();

  mcpBtnConfig.setEventHandler(handleButtonEvent);
  mcpBtnConfig.setFeature(ButtonConfig::kFeatureLongPress);
  mcpBtnConfig.setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
  mcpBtnConfig.setDebounceDelay(30);
  mcpBtnConfig.setLongPressDelay(800);

  Serial.println("[BTN] AceButton x11 configured (MCP checkState injection)");
}

// Raw level booleans (kept for brake composite in generateEvents)
bool rawBrakeLeft  = false;
bool rawBrakeRight = false;
bool rawSeat       = false;

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
//  FSM STATE VARIABLES (enums declared in forward declarations above)
// ═══════════════════════════════════════════════════════════════════

MappingState    fsmMapping    = ST_MAPPING_CLEAR;
CabinLightState fsmCabinLight = ST_CABINLIGHT_OFF;
TrunkState      fsmTrunk      = ST_TRUNK_LATCHED;
FogState        fsmFog        = ST_FSM_FOG_OFF;
DefrosterState  fsmDefroster  = ST_FSM_DEFROSTER_OFF;
BlinkerState    fsmBlinker    = ST_BLINKER_IDLE;
DriveState      fsmDrive      = ST_DRIVE_NEUTRAL;
BrakeState      fsmBrake      = ST_BRAKE_LEVEL_0;
PowerState      fsmPower      = ST_POWER_OFF;
DrlState        fsmDrl        = ST_DRL_OFF;
LowbeamState    fsmLowbeam    = ST_LOWBEAM_OFF;

bool mapCanPulsePending = false;

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
uint8_t  canThrottleRx     = 0;       // from CAN 0x407 byte 0 (8-bit ADC)
uint32_t lastThrottleRxMs  = 0;       // timestamp of last 0x407 reception
#define  THROTTLE_RX_TIMEOUT_MS 500   // consider throttle stale after 500 ms

uint16_t canLightCtrlBits      = 0;
uint16_t canLightCtrlBitsPrev  = 0;
bool     canBlinkLeftToggleReq = false;
bool     canBlinkRightToggleReq = false;

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

// Web output controls
bool webDefrosterTogglePending = false;

// ═══════════════════════════════════════════════════════════════════
//  SETUP HELPERS
// ═══════════════════════════════════════════════════════════════════

void setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
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
    // *** AUX AUDIO MOSFET always ON (temporary behavior) ***
    mcp1.digitalWrite(MCP1_AUXAUDIO_GATE, HIGH);
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
  // MOSFET outputs — default startup levels
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
//  WIFI AP + WEB SERVER
// ═══════════════════════════════════════════════════════════════════

// Helper: FSM enum → readable string
const char* mappingStateName(MappingState s) {
  switch (s) { case ST_MAPPING_CLEAR: return "CLEAR"; case ST_MAPPING_PRESSED: return "PRESSED"; } return "?";
}
const char* cabinLightStateName(CabinLightState s) {
  switch (s) { case ST_CABINLIGHT_OFF: return "OFF"; case ST_CABINLIGHT_ON: return "ON";
    case ST_CABINLIGHT_ON_TRUNK: return "ON_TRUNK"; case ST_CABINLIGHT_ON_DRV: return "ON_DRV"; } return "?";
}
const char* trunkStateName(TrunkState s) {
  switch (s) { case ST_TRUNK_LATCHED: return "LATCHED"; case ST_TRUNK_OPENING: return "OPENING";
    case ST_TRUNK_BLINK_ON: return "BLINK_ON"; case ST_TRUNK_BLINK_OFF: return "BLINK_OFF"; } return "?";
}
const char* fogStateName(FogState s) {
  switch (s) { case ST_FSM_FOG_OFF: return "OFF"; case ST_FSM_FOG_ON: return "ON";
    case ST_FSM_FOG_ON_GRACE: return "ON_GRACE"; } return "?";
}
const char* defrosterStateName(DefrosterState s) {
  switch (s) { case ST_FSM_DEFROSTER_OFF: return "OFF"; case ST_FSM_DEFROSTER_ON: return "ON";
    case ST_FSM_DEFROSTER_FAULT_BLINK_OFF: return "FAULT_BLINK_OFF";
    case ST_FSM_DEFROSTER_FAULT_BLINK_ON: return "FAULT_BLINK_ON"; } return "?";
}
const char* blinkerStateName(BlinkerState s) {
  switch (s) { case ST_BLINKER_IDLE: return "IDLE";
    case ST_WARNING_ON: return "WARNING_ON"; case ST_WARNING_OFF: return "WARNING_OFF";
    case ST_BLINKER_LEFT_ON: return "LEFT_ON"; case ST_BLINKER_LEFT_OFF: return "LEFT_OFF";
    case ST_BLINKER_RIGHT_ON: return "RIGHT_ON"; case ST_BLINKER_RIGHT_OFF: return "RIGHT_OFF"; } return "?";
}
const char* driveStateName(DriveState s) {
  switch (s) { case ST_DRIVE_NEUTRAL: return "NEUTRAL";
    case ST_DRIVE_FORWARD: return "FORWARD"; case ST_DRIVE_FORWARD_LOCK: return "FORWARD_LOCK";
    case ST_DRIVE_FWD_PENDING: return "FWD_PENDING";
    case ST_DRIVE_REVERSE: return "REVERSE"; case ST_DRIVE_REVERSE_LOCK: return "REVERSE_LOCK";
    case ST_DRIVE_REV_PENDING: return "REV_PENDING"; } return "?";
}
const char* brakeStateName(BrakeState s) {
  switch (s) { case ST_BRAKE_LEVEL_0: return "0%"; case ST_BRAKE_LEVEL_25: return "25%";
    case ST_BRAKE_LEVEL_50: return "50%"; case ST_BRAKE_LEVEL_100: return "100%"; } return "?";
}
const char* powerStateName(PowerState s) {
  switch (s) { case ST_POWER_OFF: return "OFF"; case ST_POWER_ON: return "ON";
    case ST_POWER_ON_GRACE: return "ON_GRACE"; } return "?";
}
const char* drlStateName(DrlState s) {
  switch (s) { case ST_DRL_OFF: return "OFF"; case ST_DRL_ON: return "ON";
    case ST_DRL_ON_GRACE: return "ON_GRACE"; } return "?";
}
const char* lowbeamStateName(LowbeamState s) {
  switch (s) { case ST_LOWBEAM_OFF: return "OFF"; case ST_LOWBEAM_ON: return "ON";
    case ST_LOWBEAM_ON_GRACE: return "ON_GRACE"; } return "?";
}

// ── Serve main dashboard page ────────────────────────────────────
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Velion Dashboard</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#c9d1d9;padding:12px}
  h1{text-align:center;color:#58a6ff;margin-bottom:12px;font-size:1.4em}
  h2{color:#58a6ff;font-size:1.05em;margin:8px 0 4px;border-bottom:1px solid #21262d;padding-bottom:4px}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:10px}
  .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px}
  table{width:100%;border-collapse:collapse}
  td{padding:3px 6px;border-bottom:1px solid #21262d;font-size:0.9em}
  td:first-child{color:#8b949e;width:45%}
  .on{color:#3fb950;font-weight:bold}
  .off{color:#8b949e}
    .actions{margin-top:8px;display:flex;gap:8px;flex-wrap:wrap}
    .btn{border:1px solid #30363d;background:#21262d;color:#c9d1d9;padding:6px 10px;
     border-radius:6px;cursor:pointer;font-size:0.85em}
    .btn-on{border-color:#238636;color:#3fb950}
    .btn-off{border-color:#da3633;color:#f85149}
    .hint{font-size:0.78em;color:#8b949e;margin-top:8px}
  #term{background:#010409;border:1px solid #30363d;border-radius:8px;
        padding:8px;font-family:'Cascadia Code',monospace;font-size:0.82em;
        height:340px;overflow-y:auto;white-space:pre;color:#7ee787;margin-top:10px}
  .status-bar{text-align:center;font-size:0.8em;color:#484f58;margin-top:8px}
</style></head><body>
<h1>⚡ Velion Mainboard Dashboard</h1>
<div class="grid" id="cards"></div>
<h2>🖥️ CAN Bus Terminal</h2>
<div id="term"></div>
<div class="status-bar">Auto-refresh every 1 s &mdash; WiFi AP: Velion-Dashboard</div>
<script>
function cls(v){return (v==='OFF'||v==='IDLE'||v==='NEUTRAL'||v==='CLEAR'||v==='0%')?'off':'on';}
function row(l,v){return '<tr><td>'+l+'</td><td class="'+cls(v)+'">'+v+'</td></tr>';}
function buildCards(d){
  let h='';
  // FSM card
  h+='<div class="card"><h2>🔄 State Machines</h2><table>';
  h+=row('Power',d.power); h+=row('Drive',d.drive); h+=row('Brake',d.brake);
  h+=row('DRL',d.drl); h+=row('Low Beam',d.lowbeam); h+=row('Blinker',d.blinker);
  h+=row('Cabin Light',d.cabinLight); h+=row('Fog',d.fog);
  h+=row('Defroster',d.defroster); h+=row('Trunk',d.trunk); h+=row('Mapping',d.mapping);
  h+='</table></div>';
  // Vehicle card
  h+='<div class="card"><h2>🚗 Vehicle (CAN)</h2><table>';
  h+=row('Speed',d.speed+' km/h'); h+=row('Battery SOC',d.soc+'%');
  h+=row('Battery Voltage',d.vbat+' V'); h+=row('Battery Current',d.ibat+' A');
  h+=row('LYNX Mode',d.lynxMode); h+=row('Power Map',d.currentMap);
  h+='</table></div>';
  // Inputs card
  h+='<div class="card"><h2>🎛️ Inputs</h2><table>';
  h+=row('Seat Sensor',d.seat?'OCCUPIED':'EMPTY');
  h+=row('Brake Left',d.brakeL?'PRESSED':'Released');
  h+=row('Brake Right',d.brakeR?'PRESSED':'Released');
  h+=row('Throttle ADC',d.throttle);
  h+='</table></div>';
  // Output control card
  h+='<div class="card"><h2>⚙️ Outputs</h2><table>';
  h+=row('AUX Audio MOSFET',d.auxAudio?'ON':'OFF');
  h+=row('Defroster MOSFET',d.defMosfet?'ON':'OFF');
  h+='</table>';
  h+='<div class="actions">';
  h+='<button class="btn btn-on" onclick="toggleDefroster()">Toggle Defroster (FSM)</button>';
  h+='</div><div class="hint">AUX Audio MOSFET is forced ON in firmware.</div></div>';
  // Peripherals card
  h+='<div class="card"><h2>🔌 Peripherals</h2><table>';
  h+=row('MCP1 (0x21)',d.mcp1?'OK':'FAIL');
  h+=row('MCP2 (0x20)',d.mcp2?'OK':'FAIL');
  h+=row('INA Main',d.inaMain?'OK':'FAIL');
  h+=row('INA Hand',d.inaHand?'OK':'FAIL');
  h+=row('INA Seat',d.inaSeat?'OK':'FAIL');
  h+=row('INA Defrost',d.inaDef?'OK':'FAIL');
  h+=row('MPU6050',d.mpu?'OK':'FAIL');
  h+=row('BMP280',d.bmp?'OK':'FAIL');
  h+=row('Uptime',d.uptime+' s');
  h+='</table></div>';
  document.getElementById('cards').innerHTML=h;
}
function updateTerm(lines){
  let t=document.getElementById('term');
  t.textContent=lines.join('\n');
  t.scrollTop=t.scrollHeight;
}
async function toggleDefroster(){
  try{
    await fetch('/api/output?name=defroster&action=toggle');
    poll();
  }catch(e){}
}
async function poll(){
  try{
    let r=await fetch('/api/state');let d=await r.json();buildCards(d);
    let r2=await fetch('/api/can');let d2=await r2.json();updateTerm(d2.lines);
  }catch(e){}
}
setInterval(poll,1000);
poll();
</script></body></html>
)rawliteral";
  webServer.send(200, "text/html", html);
}

// ── JSON API: state ──────────────────────────────────────────────
void handleApiState() {
  bool defMosfet = (fsmDefroster == ST_FSM_DEFROSTER_ON);

  char json[1024];
  snprintf(json, sizeof(json),
    "{\"power\":\"%s\",\"drive\":\"%s\",\"brake\":\"%s\","
    "\"drl\":\"%s\",\"lowbeam\":\"%s\",\"blinker\":\"%s\","
    "\"cabinLight\":\"%s\",\"fog\":\"%s\",\"defroster\":\"%s\","
    "\"trunk\":\"%s\",\"mapping\":\"%s\","
    "\"speed\":%d,\"soc\":%d,\"vbat\":\"%.1f\",\"ibat\":\"%.1f\","
    "\"lynxMode\":%d,\"currentMap\":%d,"
    "\"seat\":%s,\"brakeL\":%s,\"brakeR\":%s,"
    "\"throttle\":%d,"
    "\"auxAudio\":%s,\"defMosfet\":%s,"
    "\"mcp1\":%s,\"mcp2\":%s,"
    "\"inaMain\":%s,\"inaHand\":%s,\"inaSeat\":%s,\"inaDef\":%s,"
    "\"mpu\":%s,\"bmp\":%s,"
    "\"uptime\":%lu}",
    powerStateName(fsmPower), driveStateName(fsmDrive), brakeStateName(fsmBrake),
    drlStateName(fsmDrl), lowbeamStateName(fsmLowbeam), blinkerStateName(fsmBlinker),
    cabinLightStateName(fsmCabinLight), fogStateName(fsmFog), defrosterStateName(fsmDefroster),
    trunkStateName(fsmTrunk), mappingStateName(fsmMapping),
    vehicleSpeedKph, batterySocPct,
    batteryVoltage01V * 0.01f, batteryCurrent02A * 0.02f,
    lynxMode, currentMap,
    rawSeat ? "true" : "false",
    rawBrakeLeft ? "true" : "false",
    rawBrakeRight ? "true" : "false",
    analogRead(THROTTLE_INPUT),
    "true",
    defMosfet ? "true" : "false",
    mcp1_ok ? "true" : "false", mcp2_ok ? "true" : "false",
    inaMain_ok ? "true" : "false", inaHand_ok ? "true" : "false",
    inaSeat_ok ? "true" : "false", inaDef_ok ? "true" : "false",
    mpu_ok ? "true" : "false", bmp_ok ? "true" : "false",
    (unsigned long)(millis() / 1000)
  );
  webServer.send(200, "application/json", json);
}

// ── JSON API: output control ─────────────────────────────────────
void handleApiOutput() {
  if (!webServer.hasArg("name") || !webServer.hasArg("action")) {
    webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing name/action\"}");
    return;
  }

  String name = webServer.arg("name");
  String action = webServer.arg("action");

  if (name == "defroster" && action == "toggle") {
    webDefrosterTogglePending = true;
    webServer.send(200, "application/json", "{\"ok\":true,\"name\":\"defroster\",\"action\":\"toggle\"}");
    return;
  }

  webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown output\"}");
}

// ── JSON API: CAN log ────────────────────────────────────────────
void handleApiCan() {
  String out = "{\"lines\":[";
  int start = (canLogCount < CAN_LOG_SIZE) ? 0 : canLogHead;
  int count = canLogCount;
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % CAN_LOG_SIZE;
    if (i > 0) out += ",";
    // Escape quotes in the log line
    String escaped = canLog[idx];
    escaped.replace("\"", "\\\"");
    out += "\"" + escaped + "\"";
  }
  out += "]}";
  webServer.send(200, "application/json", out);
}

void setupWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  delay(100);
  Serial.printf("[WiFi] AP started: SSID=%s  IP=%s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());

  webServer.on("/",          handleRoot);
  webServer.on("/api/state", handleApiState);
  webServer.on("/api/can",   handleApiCan);
  webServer.on("/api/output", handleApiOutput);
  webServer.begin();
  Serial.println("[WiFi] Web server started on port 80");
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println(" Velion Mainboard Firmware — Booting");
  Serial.println(" Please connect via the wifi hotspot Velion-Dashboard for debugging");
  Serial.println(" passwrod : velion1234");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  setupGPIO();
  setupCAN();
  setupMCP();
  setupINA();
  setupIMU();
  setupNeoPixel();
  setupAceButtons();
  setupWiFiAP();

  Serial.println("[BOOT] Ready.\n");
}

// ═══════════════════════════════════════════════════════════════════
//  INPUT STAGE
// ═══════════════════════════════════════════════════════════════════

// --- Read MCP buttons via AceButton::checkState() ---
// We read raw MCP state, invert (active-low), and feed AceButton.
// AceButton default: HIGH = released, LOW = pressed.
void readButtons() {
  if (mcp1_ok) {
    abDefrost.checkState(mcp1.digitalRead(MCP1_DEFROST_INPUT) ? HIGH : LOW);
    abTrunk.checkState(  mcp1.digitalRead(MCP1_TRUNK_INPUT)   ? HIGH : LOW);
  }
  if (mcp2_ok) {
    uint8_t blState = mcp2.digitalRead(MCP2_BRAKE_LEFT_INPUT)  ? HIGH : LOW;
    uint8_t brState = mcp2.digitalRead(MCP2_BRAKE_RIGHT_INPUT) ? HIGH : LOW;
    uint8_t stState = mcp2.digitalRead(MCP2_SEAT_INPUT)        ? HIGH : LOW;
    rawBrakeLeft  = (blState == LOW);   // active-low: LOW = pressed
    rawBrakeRight = (brState == LOW);
    rawSeat       = (stState == LOW);

    abBrakeRight.checkState(brState);
    abBrakeLeft.checkState( blState);
    abReverse.checkState(   mcp2.digitalRead(MCP2_REVERSE_INPUT)     ? HIGH : LOW);
    abForward.checkState(   mcp2.digitalRead(MCP2_FORWARD_INPUT)     ? HIGH : LOW);
    abBrouillard.checkState(mcp2.digitalRead(MCP2_BROUILLARD_INPUT)  ? HIGH : LOW);
    abCablight.checkState(  mcp2.digitalRead(MCP2_CABLIGHT_INPUT)    ? HIGH : LOW);
    abWarning.checkState(   mcp2.digitalRead(MCP2_WARNING_INPUT)     ? HIGH : LOW);
    abMap.checkState(       mcp2.digitalRead(MCP2_MAP_INPUT)         ? HIGH : LOW);
    abSeat.checkState(      stState);
  }
}

// --- CAN RX — read all pending, print, parse known IDs ---
void readCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    // Build log string for serial + web terminal
    char buf[80];
    int pos = snprintf(buf, sizeof(buf), "[CAN RX] ID=0x%03X DLC=%d Data=", msg.identifier, msg.data_length_code);
    for (int i = 0; i < msg.data_length_code && pos < (int)sizeof(buf) - 4; i++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", msg.data[i]);
    }
    Serial.println(buf);
    canLogPush(String(buf));

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
      case CAN_ID_THROTTLE_RX: {
        // 0x407: byte 0 = throttle 8-bit ADC value from control board
        if (msg.data_length_code >= 1) {
          canThrottleRx    = msg.data[0];
          lastThrottleRxMs = millis();
        }
        break;
      }
      case CAN_ID_LIGHT_CONTROL: {
        // 0x400: bytes 0-1 = UINT16 light-control bits (LE)
        if (msg.data_length_code >= 2) {
          canLightCtrlBitsPrev = canLightCtrlBits;
          canLightCtrlBits = (uint16_t)(msg.data[0] | (msg.data[1] << 8));

          bool leftNow   = (canLightCtrlBits & (1U << BIT_LEFT_BLINK)) != 0;
          bool leftPrev  = (canLightCtrlBitsPrev & (1U << BIT_LEFT_BLINK)) != 0;
          bool rightNow  = (canLightCtrlBits & (1U << BIT_RIGHT_BLINK)) != 0;
          bool rightPrev = (canLightCtrlBitsPrev & (1U << BIT_RIGHT_BLINK)) != 0;

          // Rising edge = one toggle request for each side.
          if (leftNow && !leftPrev)   canBlinkLeftToggleReq = true;
          if (rightNow && !rightPrev) canBlinkRightToggleReq = true;
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
  if (bevMap.shortPress) ev.BTN_MAPPING_SHORT = true;
  if (bevMap.longPress)  ev.BTN_MAPPING_LONG  = true;

  // --- Cabin light button ---
  if ((bevCablight.shortPress || bevCablight.longPress) && stationary)
    ev.BTN_CABLIGHT_AND_STATIONNARY = true;
  if ((bevCablight.shortPress || bevCablight.longPress) && driving)
    ev.BTN_CABLIGHT_AND_DRIVING = true;
  if (bevCablight.released)
    ev.BTN_CABLIGHT_RELEASED = true;

  // --- Trunk button → cabin light trunk trigger ---
  if ((bevTrunk.shortPress || bevTrunk.longPress) && stationary)
    ev.BTN_TRUNK_AND_STATIONNARY = true;

  // --- Trunk FSM events ---
  if (bevTrunk.released && stationary) ev.BTN_TRUNK_RELEASED_AND_STATIONNARY = true;
  if (bevTrunk.released && driving)    ev.BTN_TRUNK_RELEASED_AND_DRIVING     = true;

  // --- Fog ---
  if (bevBrouillard.shortPress) ev.BTN_BROUILLARD_SHORT = true;
  if (bevBrouillard.longPress)  ev.BTN_BROUILLARD_LONG  = true;

  // --- Seat sensor ---
  if (bevSeat.pressed)  ev.SEATSENSOR_PRESSED  = true;
  if (bevSeat.released) ev.SEATSENSOR_RELEASED = true;

  // --- Defroster ---
  bool battOk = (batterySocPct > 30);
  if (webDefrosterTogglePending) {
    ev.BTN_DEFROSTER_SHORT = true;
    if (battOk) ev.BTN_DEFROSTER_SHORT_AND_BATTERY_OK = true;
    webDefrosterTogglePending = false;
  }
  if (bevDefrost.shortPress) {
    ev.BTN_DEFROSTER_SHORT = true;
    if (battOk) ev.BTN_DEFROSTER_SHORT_AND_BATTERY_OK = true;
  }
  if (bevDefrost.longPress) {
    ev.BTN_DEFROSTER_LONG = true;
    if (battOk) ev.BTN_DEFROSTER_LONG_AND_BATTERY_OK = true;
  }
  if (batterySocPct <= 30 && fsmDefroster == ST_FSM_DEFROSTER_ON) {
    ev.BATTERY_BELOW_30_PERCENT = true;
  }

  // --- Warning ---
  if (bevWarning.released) ev.BTN_WARNING_RELEASED = true;

  // --- Blinker commands from Filovelox smart button over CAN ---
  if (canBlinkLeftToggleReq) {
    ev.BTN_BLINKER_LEFT_RELEASED = true;
    canBlinkLeftToggleReq = false;
  }
  if (canBlinkRightToggleReq) {
    ev.BTN_BLINKER_RIGHT_RELEASED = true;
    canBlinkRightToggleReq = false;
  }

  // --- Forward / Reverse (drive direction) ---
  if (bevForward.shortPress) ev.BTN_FORWARD_SHORT = true;
  if (bevForward.longPress)  ev.BTN_FORWARD_LONG  = true;
  if (bevReverse.shortPress) ev.BTN_REVERSE_SHORT = true;
  if (bevReverse.longPress)  ev.BTN_REVERSE_LONG  = true;

  // --- Brake composite (uses raw level, not edge) ---
  bool bl = rawBrakeLeft;
  bool br = rawBrakeRight;
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
//  Entering PRESSED queues a single CAN pulse; the FSM state still
//  remains active for LED/diagnostic visibility during the 2 s window.
// ═══════════════════════════════════════════════════════════════════
void fsmStepMapping() {
  switch (fsmMapping) {
    case ST_MAPPING_CLEAR:
      if (ev.BTN_MAPPING_SHORT || ev.BTN_MAPPING_LONG) {
        fsmMapping = ST_MAPPING_PRESSED;
        mapCanPulsePending = true;
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
  if (mcp1_ok) {
    mcp1.digitalWrite(MCP1_POWER_GATE, HIGH);
    mcp1.digitalWrite(MCP1_AUXAUDIO_GATE, HIGH);  // forced ON for now
  }

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
    bool defLed    = defMosfet || (fsmDefroster == ST_FSM_DEFROSTER_FAULT_BLINK_ON);
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
//  Map switching: emit a single high pulse in one 0x5FF packet when the
//  mapping button is pressed, then return the bit low on subsequent packets.
// ═══════════════════════════════════════════════════════════════════
void sendCanControl() {
  uint32_t now = millis();
  if (now - lastCanTx < CAN_TX_INTERVAL) return;
  lastCanTx = now;

  // ── CAN Level 1 (bytes 0-1): Throttle ──
  // Forward the 8-bit ADC throttle from 0x407 scaled to INT16 (0-32766).
  // If throttle RX timed out, report invalid (32767).
  if (now - lastThrottleRxMs < THROTTLE_RX_TIMEOUT_MS) {
    canLevel1 = (int16_t)((uint32_t)canThrottleRx * 32766 / 255);
  } else {
    canLevel1 = 0x7FFF;  // invalid — no recent throttle data
  }

  // ── CAN Level 2 (bytes 2-3): Brake analog level ──
  // Map brake FSM state to analoglevel (0/25/50/100 → 0..32766).
  switch (fsmBrake) {
    case ST_BRAKE_LEVEL_0:   canLevel2 = 0;     break;
    case ST_BRAKE_LEVEL_25:  canLevel2 = 8191;  break;  // ~25%
    case ST_BRAKE_LEVEL_50:  canLevel2 = 16383; break;  // ~50%
    case ST_BRAKE_LEVEL_100: canLevel2 = 32766; break;  // 100%
  }

  // ── CAN Level 3 (bytes 4-5): unused, keep invalid ──
  canLevel3 = 0x7FFF;

  // ── Byte 6: Digital inputs + map switching ──
  canDigIn = 0;
  if (rawSeat)       canDigIn |= 0x01;   // bit 0 = digital in 0 (seat sensor)
  if (rawBrakeLeft | rawBrakeRight)  canDigIn |= 0x02;   // bit 1 = digital in 1 (brake left)
  if (mapCanPulsePending) canDigIn |= 0x04;   // bit 2 = one-shot map pulse
  // bit 3 = digital in 3 (reverse): high when drive FSM is in any reverse state
  if (fsmDrive == ST_DRIVE_REVERSE || fsmDrive == ST_DRIVE_REVERSE_LOCK || fsmDrive == ST_DRIVE_REV_PENDING)
    canDigIn |= 0x08;

  // ── Byte 7: Commands ──
  canCmd = 0;
  if (!rawSeat) canCmd |= 0x01;  // bit 0 = disarm when seat released

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

  if (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK && mapCanPulsePending) {
    mapCanPulsePending = false;
  }
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
  bevReset(bevMap);
  bevReset(bevTrunk);
  bevReset(bevDefrost);
  bevReset(bevCablight);
  bevReset(bevBrouillard);
  bevReset(bevWarning);
  bevReset(bevForward);
  bevReset(bevReverse);
  bevReset(bevSeat);
  bevReset(bevBrakeLeft);
  bevReset(bevBrakeRight);
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

  // 7. WEB — handle HTTP clients
  webServer.handleClient();

  // 8. Clear one-shot flags
  clearButtonOneShots();

  // Minimal yield — I2C reads are the bottleneck (~1 ms per MCP transaction)
  delay(1);
}
