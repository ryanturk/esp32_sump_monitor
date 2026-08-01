// Offline sump monitor with serial-configurable settings.

#include <Arduino.h>
#include <ctype.h>
#include <EEPROM.h>
#include <stddef.h>
#include <stdlib.h>

// ================= PIN MAP =================
#define TRIG_PIN 25
#define ECHO_PIN 26
#define BUZZER_PIN 27
#define POWER_LED_PIN 21
#define ERROR_LED_PIN 16
#define ALERT_LED_PIN 17

// ================= DEFAULT SUMP SETTINGS ===
static const float DEFAULT_SENSOR_TO_BOTTOM_IN = 28.0f;
static const float DEFAULT_HIGH_WATER_IN = 16.0f;
static const float DEFAULT_CLEAR_WATER_IN = 15.0f;
static const unsigned long DEFAULT_ALERT_COOLDOWN_MS = 10UL * 60UL * 1000UL;
static const unsigned long DEFAULT_CHIRP_PERIOD_MS = 5UL * 1000UL;

// ================= EEPROM ==================
static const uint32_t SETTINGS_MAGIC = 0x53554D50UL;  // "SUMP"
static const uint16_t SETTINGS_VERSION = 2;
static const int EEPROM_BYTES = 128;

struct SumpSettings {
  uint32_t magic;
  uint16_t version;
  float sensorToBottomIn;
  float highWaterIn;
  float clearWaterIn;
  unsigned long alertCooldownMs;
  unsigned long chirpPeriodMs;
  uint32_t checksum;
};

SumpSettings settings;

// ================= STATE ===================
float distanceIn = -1.0f;
float waterHeightIn = 0.0f;
bool highWater = false;
int lastAlertSecAgo = -1;
unsigned long lastAlertMs = 0;
unsigned long lastChirpMs = 0;
unsigned long lastErrorBlinkMs = 0;
bool alarmLatched = false;
bool sensorTimedOut = false;
bool errorLedOn = false;

// ================= ULTRASONIC ==============
const unsigned long ECHO_TIMEOUT_US = 25000;
const int SAMPLES = 5;

// ================= TIMING ==================
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_PERIOD_MS = 1500;
const unsigned long ERROR_BLINK_PERIOD_MS = 250;

// ================= SERIAL COMMANDS =========
char commandBuffer[96];
size_t commandLength = 0;
unsigned long lastCommandCharMs = 0;
const unsigned long COMMAND_INPUT_TIMEOUT_MS = 3000;

// =====================================================
static uint32_t calculateChecksum(const SumpSettings &value) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
  const size_t checksumOffset = offsetof(SumpSettings, checksum);
  uint32_t checksum = 2166136261UL;

  for (size_t i = 0; i < checksumOffset; i++) {
    checksum ^= bytes[i];
    checksum *= 16777619UL;
  }

  return checksum;
}

static void applyDefaultSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;
  settings.sensorToBottomIn = DEFAULT_SENSOR_TO_BOTTOM_IN;
  settings.highWaterIn = DEFAULT_HIGH_WATER_IN;
  settings.clearWaterIn = DEFAULT_CLEAR_WATER_IN;
  settings.alertCooldownMs = DEFAULT_ALERT_COOLDOWN_MS;
  settings.chirpPeriodMs = DEFAULT_CHIRP_PERIOD_MS;
  settings.checksum = calculateChecksum(settings);
}

static bool settingsAreValid() {
  if (settings.magic != SETTINGS_MAGIC || settings.version != SETTINGS_VERSION) {
    return false;
  }

  if (settings.checksum != calculateChecksum(settings)) {
    return false;
  }

  if (settings.sensorToBottomIn <= 0.0f || settings.sensorToBottomIn > 200.0f) {
    return false;
  }

  if (settings.highWaterIn <= 0.0f || settings.highWaterIn > settings.sensorToBottomIn) {
    return false;
  }

  if (settings.clearWaterIn < 0.0f || settings.clearWaterIn >= settings.highWaterIn) {
    return false;
  }

  if (settings.chirpPeriodMs > 3600000UL) {
    return false;
  }

  return true;
}

static void saveSettings() {
  settings.magic = SETTINGS_MAGIC;
  settings.version = SETTINGS_VERSION;
  settings.checksum = calculateChecksum(settings);

  EEPROM.put(0, settings);
  EEPROM.commit();
}

static void loadSettings() {
  EEPROM.begin(EEPROM_BYTES);
  EEPROM.get(0, settings);

  if (!settingsAreValid()) {
    applyDefaultSettings();
    saveSettings();
    Serial.println("Loaded default sump settings");
    return;
  }

  Serial.println("Loaded saved sump settings");
}

static float usToInches(unsigned long us) {
  return us / 148.0f;
}

static float readDistanceInches() {
  float total = 0;
  int valid = 0;

  for (int i = 0; i < SAMPLES; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    if (duration > 0) {
      total += usToInches(duration);
      valid++;
    }
    delay(40);
    yield();
  }

  if (valid == 0) return -1.0f;
  return total / valid;
}

static void chirpBuzzer(int chirps) {
  for (int i = 0; i < chirps; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);

    if (i < chirps - 1) {
      delay(180);
    }
  }
}

static void updateAlarmChirp(unsigned long now) {
  if (!alarmLatched || settings.chirpPeriodMs == 0) {
    return;
  }

  if (lastChirpMs == 0 || now - lastChirpMs >= settings.chirpPeriodMs) {
    lastChirpMs = now;
    chirpBuzzer(1);
  }
}

static void updateIndicatorLeds(unsigned long now) {
  digitalWrite(ALERT_LED_PIN, alarmLatched ? HIGH : LOW);

  if (!sensorTimedOut) {
    errorLedOn = false;
    digitalWrite(ERROR_LED_PIN, LOW);
    return;
  }

  if (now - lastErrorBlinkMs >= ERROR_BLINK_PERIOD_MS) {
    lastErrorBlinkMs = now;
    errorLedOn = !errorLedOn;
    digitalWrite(ERROR_LED_PIN, errorLedOn ? HIGH : LOW);

    if (errorLedOn) {
      chirpBuzzer(1);
    }
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  STATUS");
  Serial.println("  HELP");
  Serial.println("  HOWTO");
  Serial.println("  DEFAULTS");
  Serial.println("  TESTCHIRP");
  Serial.println("  DEPTH=<inches>      sensor-to-bottom distance");
  Serial.println("  HIGH=<inches>       alarm trigger water height");
  Serial.println("  CLEAR=<inches>      alarm clear water height");
  Serial.println("  COOLDOWN=<seconds>  alert print cooldown");
  Serial.println("  CHIRP=<seconds>     alarm chirp interval, 0 disables");
  Serial.println();
}

static void printHowToUse() {
  Serial.println();
  Serial.println("How to use:");
  Serial.println("  Serial: 9600 baud, 8 data bits, no parity, 1 stop bit, no flow control.");
  Serial.println("  Send commands over the ESP32 USB-C serial port and press Enter.");
  Serial.println();
  Serial.println("Measurements:");
  Serial.println("  DEPTH is the distance from the sensor face to the sump bottom.");
  Serial.println("  water height = DEPTH - measured distance to water.");
  Serial.println("  HIGH turns the alarm on when water height reaches that value.");
  Serial.println("  CLEAR turns the alarm off after water falls to that value or lower.");
  Serial.println("  Keep CLEAR lower than HIGH to prevent alarm chatter.");
  Serial.println();
  Serial.println("Common setup:");
  Serial.println("  DEPTH=28");
  Serial.println("  HIGH=16");
  Serial.println("  CLEAR=15");
  Serial.println("  CHIRP=5");
  Serial.println("  STATUS");
  Serial.println();
  Serial.println("Pins:");
  Serial.println("  GPIO25 TRIG, GPIO26 ECHO, GPIO27 buzzer");
  Serial.println("  GPIO21 power LED, GPIO16 error LED, GPIO17 alert LED");
  Serial.println();
}

static void printStatus() {
  Serial.println();
  Serial.println("Sump status:");
  Serial.print("  distanceIn: ");
  Serial.println(distanceIn, 2);
  Serial.print("  waterHeightIn: ");
  Serial.println(waterHeightIn, 2);
  Serial.print("  highWater: ");
  Serial.println(highWater ? "true" : "false");
  Serial.print("  sensorError: ");
  Serial.println(sensorTimedOut ? "true" : "false");
  Serial.print("  lastAlertSecAgo: ");
  Serial.println(lastAlertSecAgo);
  Serial.println("Settings:");
  Serial.print("  DEPTH=");
  Serial.println(settings.sensorToBottomIn, 2);
  Serial.print("  HIGH=");
  Serial.println(settings.highWaterIn, 2);
  Serial.print("  CLEAR=");
  Serial.println(settings.clearWaterIn, 2);
  Serial.print("  COOLDOWN=");
  Serial.println(settings.alertCooldownMs / 1000UL);
  Serial.print("  CHIRP=");
  Serial.println(settings.chirpPeriodMs / 1000.0f, 2);
  Serial.println();
}

static bool parseFloatValue(const String &command, const char *name, float &value) {
  String prefix = String(name) + "=";
  if (!command.startsWith(prefix)) {
    return false;
  }

  String rawValue = command.substring(prefix.length());
  rawValue.trim();

  char *end = nullptr;
  value = strtod(rawValue.c_str(), &end);

  if (rawValue.length() == 0 || end == rawValue.c_str()) {
    return false;
  }

  while (*end != '\0') {
    if (!isspace(*end)) {
      return false;
    }
    end++;
  }

  return true;
}

static bool parseUnsignedLongValue(const String &command, const char *name, unsigned long &value) {
  String prefix = String(name) + "=";
  if (!command.startsWith(prefix)) {
    return false;
  }

  String rawValue = command.substring(prefix.length());
  rawValue.trim();

  char *end = nullptr;
  value = strtoul(rawValue.c_str(), &end, 10);

  if (rawValue.length() == 0 || end == rawValue.c_str()) {
    return false;
  }

  while (*end != '\0') {
    if (!isspace(*end)) {
      return false;
    }
    end++;
  }

  return true;
}

static void resetAlarmState() {
  alarmLatched = false;
  highWater = false;
  lastAlertMs = 0;
  lastChirpMs = 0;
  lastAlertSecAgo = -1;
}

static void handleCommand(const char *rawCommand) {
  String command = rawCommand;
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  float floatValue = 0.0f;
  unsigned long secondsValue = 0;

  if (command == "HELP") {
    printHelp();
    return;
  }

  if (command == "HOWTO") {
    printHowToUse();
    return;
  }

  if (command == "STATUS" || command == "S" || command == "STSTUS") {
    printStatus();
    return;
  }

  if (command == "TESTCHIRP") {
    Serial.println("Testing chirp");
    chirpBuzzer(3);
    Serial.println("Chirp test done");
    return;
  }

  if (command == "DEFAULTS") {
    applyDefaultSettings();
    saveSettings();
    resetAlarmState();
    Serial.println("Settings restored to defaults and saved");
    printStatus();
    return;
  }

  if (parseFloatValue(command, "DEPTH", floatValue)) {
    if (floatValue <= 0.0f || floatValue > 200.0f || settings.highWaterIn > floatValue) {
      Serial.println("Invalid DEPTH. Use a value greater than HIGH and no more than 200 inches.");
      return;
    }

    settings.sensorToBottomIn = floatValue;
    saveSettings();
    Serial.println("DEPTH saved");
    return;
  }

  if (parseFloatValue(command, "HIGH", floatValue)) {
    if (floatValue <= settings.clearWaterIn || floatValue > settings.sensorToBottomIn) {
      Serial.println("Invalid HIGH. Use a value greater than CLEAR and no more than DEPTH.");
      return;
    }

    settings.highWaterIn = floatValue;
    saveSettings();
    resetAlarmState();
    Serial.println("HIGH saved");
    return;
  }

  if (parseFloatValue(command, "CLEAR", floatValue)) {
    if (floatValue < 0.0f || floatValue >= settings.highWaterIn) {
      Serial.println("Invalid CLEAR. Use a value from 0 up to less than HIGH.");
      return;
    }

    settings.clearWaterIn = floatValue;
    saveSettings();
    resetAlarmState();
    Serial.println("CLEAR saved");
    return;
  }

  if (parseFloatValue(command, "CHIRP", floatValue)) {
    if (floatValue < 0.0f || floatValue > 3600.0f) {
      Serial.println("Invalid CHIRP. Use seconds from 0 to 3600.");
      return;
    }

    settings.chirpPeriodMs = (unsigned long)(floatValue * 1000.0f);
    saveSettings();
    lastChirpMs = 0;
    Serial.println("CHIRP saved");
    return;
  }

  if (parseUnsignedLongValue(command, "COOLDOWN", secondsValue)) {
    if (secondsValue > 86400UL) {
      Serial.println("Invalid COOLDOWN. Use seconds from 0 to 86400.");
      return;
    }

    settings.alertCooldownMs = secondsValue * 1000UL;
    saveSettings();
    Serial.println("COOLDOWN saved");
    return;
  }

  Serial.print("Unknown command: [");
  Serial.print(command);
  Serial.println("]. Send HELP for available commands.");
}

static void readCommandSerial() {
  unsigned long now = millis();
  if (commandLength > 0 && now - lastCommandCharMs > COMMAND_INPUT_TIMEOUT_MS) {
    commandLength = 0;
  }

  while (Serial.available()) {
    char character = Serial.read();
    lastCommandCharMs = now;

    if (character == '\r' || character == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (character == '\b' || character == 127) {
      if (commandLength > 0) {
        commandLength--;
      }
      continue;
    }

    if (iscntrl(character)) {
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = character;
    } else {
      commandLength = 0;
      Serial.println("Command too long");
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(100);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(POWER_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  digitalWrite(ALERT_LED_PIN, LOW);

  Serial.println();
  Serial.println("Offline sump monitor booting...");
  loadSettings();
  printHelp();
  printStatus();

  digitalWrite(POWER_LED_PIN, HIGH);
}

void loop() {
  readCommandSerial();

  unsigned long now = millis();
  updateIndicatorLeds(now);
  updateAlarmChirp(now);

  if (now - lastSampleMs < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSampleMs = now;

  float measuredDistanceIn = readDistanceInches();
  if (measuredDistanceIn < 0.0f) {
    if (!sensorTimedOut) {
      Serial.println("Sensor read timeout");
      sensorTimedOut = true;
    }
    distanceIn = -1.0f;
    updateIndicatorLeds(now);
    return;
  }

  if (sensorTimedOut) {
    Serial.println("Sensor reading restored");
    sensorTimedOut = false;
    updateIndicatorLeds(now);
  }

  distanceIn = measuredDistanceIn;

  float measuredWaterHeightIn = settings.sensorToBottomIn - measuredDistanceIn;
  if (measuredWaterHeightIn < 0.0f) measuredWaterHeightIn = 0.0f;
  waterHeightIn = measuredWaterHeightIn;

  if (!alarmLatched && waterHeightIn >= settings.highWaterIn) {
    alarmLatched = true;

    if (lastAlertMs == 0 || now - lastAlertMs > settings.alertCooldownMs) {
      lastAlertMs = now;
      Serial.println("ALERT: high water");
    }
  }

  if (alarmLatched && waterHeightIn <= settings.clearWaterIn) {
    alarmLatched = false;
    lastChirpMs = 0;
    Serial.println("Alarm cleared");
  }

  highWater = alarmLatched;
  lastAlertSecAgo = lastAlertMs == 0 ? -1 : (int)((now - lastAlertMs) / 1000UL);
  updateIndicatorLeds(now);

}
