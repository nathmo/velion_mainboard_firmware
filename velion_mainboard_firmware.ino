/*
 * ESP32 DevKit V1 Circuit Test Controller
 * Comprehensive test firmware for electric circuit validation
 */

#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// Pin Definitions
#define THROTTLE_INPUT 36
#define InterruptEXTGPIO 39
#define POT_SPEED_A 34
#define POT_SPEED_B 35
#define POT_SEATHEATER_A 32
#define POT_SEATHEATER_B 33
#define POT_HANDHEATER_A 25
#define POT_HANDHEATER_B 26
#define DEFROSTER_MOSFET_GATE 13
#define INTERRIOR_LIGHT_MOSFET_GATE 15
#define BLIKER_LEFT_MOSFET_GATE 2
#define BLIKER_RIGHT_MOSFET_GATE 0
#define HANDHEATER_MOSFET_GATE 4
#define DRL_RIGHT_MOSFET_GATE 16
#define DRL_LEFT_MOSFET_GATE 17
#define SEATHEATER_MOSFET_GATE 5
#define RGBLED_DIN 21
#define SDA_PIN 23
#define SCL_PIN 22

// PWM Configuration
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// NeoPixel Configuration
#define NUM_PIXELS 5
Adafruit_NeoPixel pixels(NUM_PIXELS, RGBLED_DIN, NEO_GRB + NEO_KHZ800);

// MCP23017 Controllers
Adafruit_MCP23X17 mcp1;  // Address 0x21
Adafruit_MCP23X17 mcp2;  // Address 0x20

// IMU Sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// Rotary Encoder Structure
struct RotaryEncoder {
  int pinA;
  int pinB;
  int position;
  int lastEncoded;
  unsigned long lastDebounceTime;
  const char* name;
};

RotaryEncoder encoders[3] = {
  {POT_SPEED_A, POT_SPEED_B, 0, 0, 0, "SPEED"},
  {POT_SEATHEATER_A, POT_SEATHEATER_B, 0, 0, 0, "SEATHEATER"},
  {POT_HANDHEATER_A, POT_HANDHEATER_B, 0, 0, 0, "HANDHEATER"}
};

// Debounce Configuration
#define DEBOUNCE_DELAY 50
unsigned long lastInterruptTime = 0;

// Timer for IMU readings
unsigned long lastIMUPrint = 0;
#define IMU_PRINT_INTERVAL 5000

// MCP Pin States (for button logic)
uint16_t lastMCP1State = 0xFFFF;
uint16_t lastMCP2State = 0xFFFF;

void IRAM_ATTR handleInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > DEBOUNCE_DELAY) {
    Serial.println("INTERRUPT: InterruptEXTGPIO triggered");
    lastInterruptTime = currentTime;
  }
}

void setupPWM() {
  // ESP32 Arduino Core 3.x uses ledcAttach instead of ledcSetup + ledcAttachPin
  ledcAttach(DEFROSTER_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(INTERRIOR_LIGHT_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(BLIKER_LEFT_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(BLIKER_RIGHT_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(HANDHEATER_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(DRL_RIGHT_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(DRL_LEFT_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(SEATHEATER_MOSFET_GATE, PWM_FREQ, PWM_RESOLUTION);
}

void setupEncoders() {
  for (int i = 0; i < 3; i++) {
    pinMode(encoders[i].pinA, INPUT);
    pinMode(encoders[i].pinB, INPUT);
  }
}

void setupMCP23017() {
  // Initialize MCP1 (0x21)
  if (!mcp1.begin_I2C(0x21)) {
    Serial.println("ERROR: MCP23017 at 0x21 not found!");
  } else {
    Serial.println("MCP23017 at 0x21 initialized");
    
    // Configure MCP1 GPB as inputs/outputs
    mcp1.pinMode(8, OUTPUT);  // GPB0 - Warning_LED
    mcp1.pinMode(9, OUTPUT);  // GPB1 - CABLIGHT_LED
    mcp1.pinMode(10, OUTPUT); // GPB2 - DEFROST_LED
    mcp1.pinMode(11, OUTPUT); // GPB3 - TRUNK_LED
    mcp1.pinMode(12, OUTPUT); // GPB4 - BROUILLARD_LED
    mcp1.pinMode(13, OUTPUT); // GPB5 - MAP_LED
    mcp1.pinMode(14, INPUT_PULLUP); // GPB6 - DEFROST_INPUT
    mcp1.pinMode(15, INPUT_PULLUP); // GPB7 - TRUNK_INPUT
    
    // Configure MCP1 GPA as outputs
    mcp1.pinMode(0, OUTPUT);  // GPA0 - AUXAUDIO_MOSFET_GATE
    mcp1.pinMode(1, OUTPUT);  // GPA1 - AUXUSB_MOSFET_GATE
    mcp1.pinMode(2, OUTPUT);  // GPA2 - LATCH_TRUNK_MOSFTET_GATE
    mcp1.pinMode(3, OUTPUT);  // GPA3 - POWER_MOSFET_GATE
  }
  
  // Initialize MCP2 (0x20)
  if (!mcp2.begin_I2C(0x20)) {
    Serial.println("ERROR: MCP23017 at 0x20 not found!");
  } else {
    Serial.println("MCP23017 at 0x20 initialized");
    
    // Configure MCP2 GPB
    mcp2.pinMode(8, INPUT);   // GPB0 - Defroster_ALERT
    mcp2.pinMode(9, INPUT);   // GPB1 - HANDHEATER_ALERT
    mcp2.pinMode(10, INPUT);  // GPB2 - SEATHEATER_ALERT
    mcp2.pinMode(11, INPUT);  // GPB3 - POWERINPUT_ALERT
    mcp2.pinMode(12, INPUT);  // GPB4 - POT_SEATHEATER_X
    mcp2.pinMode(13, INPUT);  // GPB5 - POT_HANDHEATER_X
    mcp2.pinMode(14, INPUT);  // GPB6 - POT_SPEED_X
    mcp2.pinMode(15, INPUT_PULLUP); // GPB7 - SEAT_INPUT
    
    // Configure MCP2 GPA as inputs with pullups
    mcp2.pinMode(0, INPUT_PULLUP); // GPA0 - BRAKE_RIGHT_INPUT
    mcp2.pinMode(1, INPUT_PULLUP); // GPA1 - BRAKE_LEFT_INPUT
    mcp2.pinMode(2, INPUT_PULLUP); // GPA2 - REVERSE_INPUT
    mcp2.pinMode(3, INPUT_PULLUP); // GPA3 - FORWARD_INPUT
    mcp2.pinMode(4, INPUT_PULLUP); // GPA4 - BROUILLARD_INPUT
    mcp2.pinMode(5, INPUT_PULLUP); // GPA5 - CABLIGHT_INPUT
    mcp2.pinMode(6, INPUT_PULLUP); // GPA6 - WARNING_INPUT
    mcp2.pinMode(7, INPUT_PULLUP); // GPA7 - MAP_INPUT
    
    // Configure interrupts for GPB0-3
    mcp2.setupInterrupts(true, false, LOW);
    for (int i = 8; i <= 11; i++) {
      mcp2.setupInterruptPin(i, CHANGE);
    }
  }
}

void setupIMU() {
  if (!mpu.begin(0x68)) {
    Serial.println("ERROR: MPU9250 not found!");
  } else {
    Serial.println("MPU9250 initialized");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
  
  if (!bmp.begin(0x76)) {
    Serial.println("ERROR: BMP280 not found!");
  } else {
    Serial.println("BMP280 initialized");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nESP32 Circuit Test Controller");
  Serial.println("================================");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Setup components
  setupPWM();
  setupEncoders();
  
  // Setup interrupt pin
  pinMode(InterruptEXTGPIO, INPUT);
  attachInterrupt(digitalPinToInterrupt(InterruptEXTGPIO), handleInterrupt, FALLING);
  
  // Setup analog input
  pinMode(THROTTLE_INPUT, INPUT);
  
  // Initialize NeoPixels
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  // Initialize MCP23017s
  setupMCP23017();
  
  // Initialize IMU
  setupIMU();
  
  Serial.println("\nReady! Enter commands:");
  printHelp();
}

void readEncoders() {
  for (int i = 0; i < 3; i++) {
    int MSB = digitalRead(encoders[i].pinA);
    int LSB = digitalRead(encoders[i].pinB);
    int encoded = (MSB << 1) | LSB;
    int sum = (encoders[i].lastEncoded << 2) | encoded;
    
    if (millis() - encoders[i].lastDebounceTime > DEBOUNCE_DELAY) {
      if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoders[i].position++;
        Serial.print("ENCODER ");
        Serial.print(encoders[i].name);
        Serial.print(": ");
        Serial.println(encoders[i].position);
        encoders[i].lastDebounceTime = millis();
      }
      if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoders[i].position--;
        Serial.print("ENCODER ");
        Serial.print(encoders[i].name);
        Serial.print(": ");
        Serial.println(encoders[i].position);
        encoders[i].lastDebounceTime = millis();
      }
    }
    
    encoders[i].lastEncoded = encoded;
  }
}

void readMCPInputs() {
  // Read MCP1 inputs (buttons)
  uint16_t currentMCP1 = 0;
  currentMCP1 |= (!mcp1.digitalRead(14)) << 14; // DEFROST_INPUT
  currentMCP1 |= (!mcp1.digitalRead(15)) << 15; // TRUNK_INPUT
  
  // Read MCP2 inputs
  uint16_t currentMCP2 = 0;
  for (int i = 0; i < 8; i++) {
    currentMCP2 |= (!mcp2.digitalRead(i)) << i;
  }
  for (int i = 8; i < 16; i++) {
    currentMCP2 |= (mcp2.digitalRead(i)) << i;
  }
  
  // Check for changes and print
  if ((currentMCP1 & (1 << 14)) != (lastMCP1State & (1 << 14))) {
    Serial.print("DEFROST_INPUT: ");
    Serial.println((currentMCP1 & (1 << 14)) ? "PRESSED" : "RELEASED");
  }
  if ((currentMCP1 & (1 << 15)) != (lastMCP1State & (1 << 15))) {
    Serial.print("TRUNK_INPUT: ");
    Serial.println((currentMCP1 & (1 << 15)) ? "PRESSED" : "RELEASED");
  }
  
  // Check MCP2 inputs
  const char* inputNames[] = {
    "BRAKE_RIGHT", "BRAKE_LEFT", "REVERSE", "FORWARD",
    "BROUILLARD", "CABLIGHT", "WARNING", "MAP",
    "Defroster_ALERT", "HANDHEATER_ALERT", "SEATHEATER_ALERT", "POWERINPUT_ALERT",
    "POT_SEATHEATER_X", "POT_HANDHEATER_X", "POT_SPEED_X", "SEAT"
  };
  
  for (int i = 0; i < 16; i++) {
    if ((currentMCP2 & (1 << i)) != (lastMCP2State & (1 << i))) {
      Serial.print(inputNames[i]);
      Serial.print("_INPUT: ");
      Serial.println((currentMCP2 & (1 << i)) ? "PRESSED" : "RELEASED");
    }
  }
  
  lastMCP1State = currentMCP1;
  lastMCP2State = currentMCP2;
  
  // Implement button-LED logic
  mcp1.digitalWrite(13, !mcp2.digitalRead(7));   // MAP_LED <- MAP_INPUT
  mcp1.digitalWrite(11, !mcp1.digitalRead(15));  // TRUNK_LED <- TRUNK_INPUT
  mcp1.digitalWrite(10, !mcp1.digitalRead(14));  // DEFROST_LED <- DEFROST_INPUT
  mcp1.digitalWrite(9, !mcp2.digitalRead(5));    // CABLIGHT_LED <- CABLIGHT_INPUT
  mcp1.digitalWrite(8, !mcp2.digitalRead(6));    // WARNING_LED <- WARNING_INPUT
  mcp1.digitalWrite(12, !mcp2.digitalRead(4));   // BROUILLARD_LED <- BROUILLARD_INPUT
}

void printIMUData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  Serial.println("\n=== IMU Data ===");
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s² | ");
  Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s² | ");
  Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s²");
  
  Serial.print("Gyro X: "); Serial.print(g.gyro.x); Serial.print(" rad/s | ");
  Serial.print("Y: "); Serial.print(g.gyro.y); Serial.print(" rad/s | ");
  Serial.print("Z: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
  
  Serial.print("Temp: "); Serial.print(bmp.readTemperature()); Serial.println(" °C");
  Serial.print("Pressure: "); Serial.print(bmp.readPressure() / 100.0); Serial.println(" hPa");
  Serial.print("Altitude: "); Serial.print(bmp.readAltitude(1013.25)); Serial.println(" m");
  Serial.println("================\n");
}

void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  pwm <pin> <duty>     - Set PWM (0-255) on pin");
  Serial.println("  on <pin>             - Turn pin ON (255)");
  Serial.println("  off <pin>            - Turn pin OFF (0)");
  Serial.println("  mcp <addr> <pin> <0/1> - Set MCP pin");
  Serial.println("  led <idx> <0xRRGGBB> - Set NeoPixel color");
  Serial.println("  dim <brightness>     - Set NeoPixel brightness (0-255)");
  Serial.println("  analog               - Read analog input");
  Serial.println("  imu                  - Print IMU data now");
  Serial.println("  help                 - Show this help");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  
  int space1 = cmd.indexOf(' ');
  String command = space1 > 0 ? cmd.substring(0, space1) : cmd;
  command.toLowerCase();
  
  if (command == "help") {
    printHelp();
  }
  else if (command == "pwm") {
    int space2 = cmd.indexOf(' ', space1 + 1);
    int pin = cmd.substring(space1 + 1, space2).toInt();
    int duty = cmd.substring(space2 + 1).toInt();
    
    // In ESP32 Core 3.x, we write directly to the pin
    if (pin == DEFROSTER_MOSFET_GATE || pin == INTERRIOR_LIGHT_MOSFET_GATE ||
        pin == BLIKER_LEFT_MOSFET_GATE || pin == BLIKER_RIGHT_MOSFET_GATE ||
        pin == HANDHEATER_MOSFET_GATE || pin == DRL_RIGHT_MOSFET_GATE ||
        pin == DRL_LEFT_MOSFET_GATE || pin == SEATHEATER_MOSFET_GATE) {
      ledcWrite(pin, duty);
      Serial.print("Set GPIO"); Serial.print(pin); 
      Serial.print(" PWM to "); Serial.println(duty);
    } else {
      Serial.println("Invalid PWM pin");
    }
  }
  else if (command == "on" || command == "off") {
    int pin = cmd.substring(space1 + 1).toInt();
    int duty = (command == "on") ? 255 : 0;
    
    if (pin == DEFROSTER_MOSFET_GATE || pin == INTERRIOR_LIGHT_MOSFET_GATE ||
        pin == BLIKER_LEFT_MOSFET_GATE || pin == BLIKER_RIGHT_MOSFET_GATE ||
        pin == HANDHEATER_MOSFET_GATE || pin == DRL_RIGHT_MOSFET_GATE ||
        pin == DRL_LEFT_MOSFET_GATE || pin == SEATHEATER_MOSFET_GATE) {
      ledcWrite(pin, duty);
      Serial.print("GPIO"); Serial.print(pin); 
      Serial.print(" turned "); Serial.println(command);
    } else {
      Serial.println("Invalid pin");
    }
  }
  else if (command == "mcp") {
    int space2 = cmd.indexOf(' ', space1 + 1);
    int space3 = cmd.indexOf(' ', space2 + 1);
    int addr = strtol(cmd.substring(space1 + 1, space2).c_str(), NULL, 16);
    int pin = cmd.substring(space2 + 1, space3).toInt();
    int value = cmd.substring(space3 + 1).toInt();
    
    if (addr == 0x21) {
      mcp1.digitalWrite(pin, value);
      Serial.print("MCP 0x21 pin "); Serial.print(pin);
      Serial.print(" set to "); Serial.println(value);
    } else if (addr == 0x20) {
      mcp2.digitalWrite(pin, value);
      Serial.print("MCP 0x20 pin "); Serial.print(pin);
      Serial.print(" set to "); Serial.println(value);
    } else {
      Serial.println("Invalid MCP address");
    }
  }
  else if (command == "led") {
    int space2 = cmd.indexOf(' ', space1 + 1);
    int idx = cmd.substring(space1 + 1, space2).toInt();
    long color = strtol(cmd.substring(space2 + 1).c_str(), NULL, 16);
    
    if (idx >= 0 && idx < NUM_PIXELS) {
      uint8_t r = (color >> 16) & 0xFF;
      uint8_t g = (color >> 8) & 0xFF;
      uint8_t b = color & 0xFF;
      pixels.setPixelColor(idx, pixels.Color(r, g, b));
      pixels.show();
      Serial.print("LED "); Serial.print(idx);
      Serial.print(" set to 0x"); Serial.println(color, HEX);
    } else {
      Serial.println("Invalid LED index");
    }
  }
  else if (command == "dim") {
    int brightness = cmd.substring(space1 + 1).toInt();
    pixels.setBrightness(brightness);
    pixels.show();
    Serial.print("Brightness set to "); Serial.println(brightness);
  }
  else if (command == "analog") {
    int val = analogRead(THROTTLE_INPUT);
    Serial.print("THROTTLE_INPUT: "); Serial.println(val);
  }
  else if (command == "imu") {
    printIMUData();
  }
  else {
    Serial.println("Unknown command. Type 'help' for commands.");
  }
}

void loop() {
  // Read encoders
  readEncoders();
  
  // Read MCP inputs
  static unsigned long lastMCPRead = 0;
  if (millis() - lastMCPRead > 50) {
    readMCPInputs();
    lastMCPRead = millis();
  }
  
  // Print IMU data every 5 seconds
  if (millis() - lastIMUPrint > IMU_PRINT_INTERVAL) {
    printIMUData();
    lastIMUPrint = millis();
  }
  
  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  delay(10);
}