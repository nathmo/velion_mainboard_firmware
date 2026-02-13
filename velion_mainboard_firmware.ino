#include <Wire.h>
#include <Adafruit_INA228.h>

// ================= PWM Pins =====================
#define DEFROSTER_MOSFET_GATE   13
#define HANDHEATER_MOSFET_GATE  4
#define SEATHEATER_MOSFET_GATE  5

// ================= INA228 Objects ===============
Adafruit_INA228 inaMain;
Adafruit_INA228 inaHand;
Adafruit_INA228 inaSeat;
Adafruit_INA228 inaDef;

// ================= Shunt Config =================
#define SHUNT_RESISTOR_OHMS 0.001     // 1 mΩ
#define MAX_EXPECTED_CURRENT 50.0     // Adjust to your real max current (Amps)

// ================= Software PWM =================
#define PWM_PERIOD_MS 100   // 10 Hz → 100 ms period

uint8_t dutySeat = 0;
uint8_t dutyHand = 0;
uint8_t dutyDef  = 10;

unsigned long pwmStartTime = 0;

// =================================================

void updateSoftwarePWM() {
  unsigned long now = millis();

  if (now - pwmStartTime >= PWM_PERIOD_MS) {
    pwmStartTime = now;
  }

  unsigned long elapsed = now - pwmStartTime;

  unsigned long onSeat = (PWM_PERIOD_MS * dutySeat) / 100;
  unsigned long onHand = (PWM_PERIOD_MS * dutyHand) / 100;
  unsigned long onDef  = (PWM_PERIOD_MS * dutyDef)  / 100;

  digitalWrite(SEATHEATER_MOSFET_GATE, elapsed < onSeat);
  digitalWrite(HANDHEATER_MOSFET_GATE, elapsed < onHand);
  digitalWrite(DEFROSTER_MOSFET_GATE,  elapsed < onDef);
}

// ------------- Serial Command Parser -------------
void handleSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex < 0) return;

  String device = cmd.substring(0, spaceIndex);
  int value = cmd.substring(spaceIndex + 1).toInt();

  if (value < 0) value = 0;
  if (value > 100) value = 100;

  if (device == "hand") {
    dutyHand = value;
  } 
  else if (device == "seat") {
    dutySeat = value;
  } 
  else if (device == "defroster") {
    dutyDef = value;
  }
}

// ====================== Setup ======================
void setup() {

  Serial.begin(115200);
  Serial.println("booting");
  Wire.begin(23, 22);

  pinMode(DEFROSTER_MOSFET_GATE, OUTPUT);
  pinMode(HANDHEATER_MOSFET_GATE, OUTPUT);
  pinMode(SEATHEATER_MOSFET_GATE, OUTPUT);

  pwmStartTime = millis();

  // ---- Initialize INA228 Devices ----
  if (!inaMain.begin(0x40)) while (1);
  Serial.println("A");
  delay(100);
  //if (!inaHand.begin(0x41)) while (1);
  Serial.println("B");
  delay(100);
  if (!inaSeat.begin(0x44)) while (1);
  Serial.println("C");
  delay(100);
  if (!inaDef.begin(0x45)) while (1);
  Serial.println("D");
  delay(100);
  
  // ---- Configure Shunt + Current Range ----
  inaMain.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
  inaHand.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
  inaSeat.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
  inaDef.setShunt(SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT);
  Serial.print("E");
  // Optional averaging for smoother readings
  inaMain.setAveragingCount(INA228_COUNT_16);
  inaHand.setAveragingCount(INA228_COUNT_16);
  inaSeat.setAveragingCount(INA228_COUNT_16);
  inaDef.setAveragingCount(INA228_COUNT_16);
  Serial.print("F");
}

// ====================== Loop ======================
void loop() {

  handleSerial();
  updateSoftwarePWM();

  // Read current directly in Amps
  float main_A = inaMain.getCurrent_mA() / 1000.0;
  float hand_A = inaHand.getCurrent_mA() / 1000.0;
  float def_A  = inaDef.getCurrent_mA()  / 1000.0;
  float seat_A = inaSeat.getCurrent_mA() / 1000.0;

  // Serial Plotter (4 columns, in Amps)
  Serial.print(main_A); Serial.print("\t");
  //Serial.print(hand_A); Serial.print("\t");
  Serial.print(def_A);  Serial.print("\t");
  Serial.println(seat_A);
}
