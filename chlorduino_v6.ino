/*
  Chlorduino v6

  Firmware scaffold for an Arduino-based pool chlorinator controller using:
  - Seeed Studio XIAO SAMD21
  - Kamoer 12 V dosing pump driven by MOSFET
  - SH1106 I2C OLED
  - DS18B20 temperature sensor
  - NEO-6M GPS (time source / location)
  - Three buttons (up/down/select)
  - Battery voltage monitor via resistor divider

  This sketch intentionally starts with a conservative, non-blocking architecture.
  Device-specific libraries and calibration can be added incrementally.
*/

#include <FlashStorage.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SolarCalculator.h>
#include <TimeLib.h>
#include <Adafruit_SleepyDog.h>
#include <TinyGPS++.h>
#include <Bounce2.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdio.h>

enum SystemMode {
  MODE_BOOT,
  MODE_IDLE,
  MODE_DOSING,
  MODE_LOW_BATTERY,
  MODE_FAULT
};

enum ScreenId {
  SCREEN_IDLE,
  SCREEN_MENU,
  SCREEN_EDIT_DURATION,
  SCREEN_EDIT_UTC,
  SCREEN_EDIT_VOFFSET,
  SCREEN_EDIT_MANUAL,
  SCREEN_SYSTEM,
  SCREEN_FAULT
};

struct ButtonState {
  bool pressed = false;
  bool changed = false;
  bool longPressed = false;
  unsigned long lastChangeMs = 0;
  unsigned long pressedAtMs = 0;
};

enum RestartReason {
  RESTART_UNKNOWN,
  RESTART_POWER_ON,
  RESTART_BACKUP,
  RESTART_WATCHDOG,
  RESTART_SOFTWARE,
  RESTART_EXTERNAL,
  RESTART_BROWNOUT
};

struct SensorState {
  float waterTempF = NAN;
  float batteryVolts = NAN;
  float nextSunsetHours = -1.0f;
  float nextPumpRunHours = -1.0f;
  uint32_t nextDoseEpochUtc = 0;
  bool gpsHasFix = false;
  bool tempValid = false;
  bool batteryValid = false;
  uint8_t tempSensorCount = 0;
};

struct PumpState {
  bool enabled = false;
  bool commandedOn = false;
  bool keepDisplayOn = false;
  bool usbHeartbeatLed = false;
  unsigned long startedAtMs = 0;
  unsigned long targetRunMs = 0;
  unsigned long accumulatedTodayMs = 0;
};

struct UiState {
  ScreenId screen = SCREEN_IDLE;
  uint8_t menuIndex = 0;
  uint8_t menuScroll = 0;
  uint8_t systemScroll = 0;
  bool displayOn = true;
  unsigned long lastActivityMs = 0;
  uint16_t editDurationMinutes = 10;
  int8_t editUtcOffsetHours = -5;
  int16_t editBatteryOffsetTenths = 12;
  uint16_t editManualMinutes = 0;
};

struct StartupState {
  RestartReason restartReason = RESTART_UNKNOWN;
  uint8_t rawResetCause = 0;
  bool serialAttached = false;
  bool gpsLocked = false;
  unsigned long gpsLockWaitMs = 0;
  unsigned long gpsTimeSuppressedUntilMs = 0;
};

struct PersistentSettings {
  uint32_t magic = 0;
  int8_t utcOffsetHours = -5;
  uint16_t runDurationMinutes = 10;
  int16_t batteryOffsetTenths = 12;
  uint32_t restartCount = 0;
  uint32_t crashCount = 0;
  uint32_t lastDoseEpochUtc = 0;
  uint32_t nextDoseEpochUtc = 0;
};

struct Config {
  float lowBatteryWarnV = 12.2f;
  float lowBatteryCutoffV = 12.0f;
  float batteryDividerRatio = 127.0f / 27.0f;
  float adcReferenceVolts = 3.30f;
  unsigned long maxPumpContinuousMs = 31UL * 60UL * 1000UL;
  unsigned long maxPumpDailyMs = 30UL * 60UL * 1000UL;
  unsigned long primeDoseMs = 15UL * 1000UL;
};

namespace Pins {
  constexpr uint8_t BATTERY_ADC = A0;
  constexpr uint8_t PUMP_GATE = 1;
  constexpr uint8_t ONE_WIRE = 2;
  constexpr uint8_t BUTTON_UP = 8;
  constexpr uint8_t BUTTON_DOWN = 9;
  constexpr uint8_t BUTTON_SELECT = 10;
}

constexpr unsigned long BATTERY_INTERVAL_MS = 2000;
constexpr unsigned long TEMP_INTERVAL_DISPLAY_ON_MS = 60000;
constexpr unsigned long TEMP_INTERVAL_DISPLAY_OFF_MS = 10000;
constexpr unsigned long GPS_INTERVAL_MS = 1000;
constexpr unsigned long SAFETY_INTERVAL_MS = 100;
constexpr unsigned long SERIAL_ATTACH_TIMEOUT_MS = 2500;
constexpr unsigned long MIN_GPS_SPINNER_MS = 2000;
constexpr unsigned long MIN_TEMP_SPINNER_MS = 2000;
constexpr unsigned long POST_LOCK_SCREEN_MS = 5000;
constexpr unsigned long STARTUP_LINE_MIN_MS = 1000;
constexpr unsigned long DISPLAY_TIMEOUT_MS = 60000;
constexpr unsigned long MENU_TIMEOUT_MS = 30000;
constexpr unsigned long SHORT_PRESS_MAX_MS = 2000;
constexpr unsigned long TEMP_STARTUP_RETRY_MS = 1000;
constexpr unsigned long GPS_MANUAL_TIME_HOLDOFF_MS = 2UL * 60UL * 60UL * 1000UL;
constexpr long RUNUP_TARGET_LEAD_SECONDS = 45L;
constexpr long RUNUP_MIN_LEAD_SECONDS = 35L;
constexpr uint8_t STARTUP_LOG_LINES = 6;
constexpr uint8_t STARTUP_LINE_CHARS = 21;
constexpr uint32_t SETTINGS_MAGIC = 0x43484C52UL;
constexpr int BATTERY_ADC_MIN_VALID_RAW = 8;
constexpr float USB_POWER_MAX_VOLTS = 7.0f;

FlashStorage(settingsStorage, PersistentSettings);

Config config;
PersistentSettings settings;
SensorState sensors;
PumpState pump;
UiState ui;
StartupState startup;
ButtonState buttonUp;
ButtonState buttonDown;
ButtonState buttonSelect;
SystemMode systemMode = MODE_BOOT;

Bounce debouncerUp;
Bounce debouncerDown;
Bounce debouncerSelect;

TinyGPSPlus gps;
OneWire oneWireBus(Pins::ONE_WIRE);
DallasTemperature waterTempSensor(&oneWireBus);
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

char startupLog[STARTUP_LOG_LINES][STARTUP_LINE_CHARS + 1] = {};
uint8_t startupLogCount = 0;
unsigned long lastStartupLogMs = 0;

unsigned long lastUiUpdateMs = 0;
unsigned long lastBatteryReadMs = 0;
unsigned long lastTempReadMs = 0;
unsigned long lastGpsPollMs = 0;
unsigned long lastSafetyCheckMs = 0;
int watchdogTimeoutMs = 0;
float batteryHistory[10] = {};
uint8_t batteryHistoryCount = 0;
uint8_t batteryHistoryIndex = 0;

constexpr uint8_t MENU_ITEM_COUNT = 8;
const char *const MENU_ITEMS[MENU_ITEM_COUNT] = {
  "duration",
  "utc",
  "voffset",
  "manual",
  "system",
  "runup",
  "restart",
  "back"
};

float readBatteryVoltage();
void readButtons();
void updateButton(ButtonState &button, Bounce &debouncer, const char *label, unsigned long nowMs);
void updateSensors(unsigned long nowMs);
void updateBattery(unsigned long nowMs);
void updateTemperature(unsigned long nowMs);
void updateGps(unsigned long nowMs);
bool sampleTemperatureNow();
void consumeGpsData();
void updateNextSunset();
time_t computeNextDoseEpochUtc(time_t utcNow);
void refreshSchedule(bool persist);
time_t midnightUtcFor(time_t utc);
time_t sunsetEpochUtcForDate(time_t dateUtc);
void updateUi();
void runControlLogic(unsigned long nowMs);
void applyPumpOutput(unsigned long nowMs);
void runSafetyChecks(unsigned long nowMs);
void updateDisplayPower(unsigned long nowMs);
void updateStatusLed(unsigned long nowMs);
void handleSerialCommands();
void processSerialCommand(const char *command);
void printState();
void loadPersistentSettings();
void savePersistentSettings();
void updateRestartCounters();
void initializeWatchdog();
void restartController();
bool isMenuScreen(ScreenId screen);
void formatLastDoseText(char *buffer, size_t size, const char *prefix);
void formatNextDoseText(char *buffer, size_t size, const char *prefix);
void startPumpRunMinutes(uint16_t minutes, unsigned long nowMs);
void stopPumpRun();
bool performRunup();
bool gpsTimeSyncAllowed();
void setMode(SystemMode nextMode);
void enterFault(const __FlashStringHelper *message);
const __FlashStringHelper *modeName(SystemMode mode);
RestartReason detectRestartReason(uint8_t &rawCause);
const __FlashStringHelper *restartReasonName(RestartReason reason);
void initializeStartup();
bool attachDebugSerial(unsigned long timeoutMs);
void printStartupBanner();
void waitForGpsStartupLock();
void waitForTemperatureStartupLock();
bool gpsStartupLockAcquired();
bool temperatureStartupLockAcquired();
void initializeDisplay();
void startupLogLine(const char *message);
void renderStartupScreen(const char *statusLine = nullptr);
void renderIdleScreen();
void renderMenuScreen();
void renderDurationScreen();
void renderUtcScreen();
void renderBatteryOffsetScreen();
void renderManualScreen();
void renderSystemScreen();
void formatGpsSpinnerLine(char *buffer, size_t size, uint32_t spinnerTick);
const char *restartReasonText(RestartReason reason);
void waitForStartupLineDwell();
void initializeTemperatureSensor();
void noteUserActivity(unsigned long nowMs);
void setDisplayPower(bool enabled);
time_t localTimeNow();
void syncClockFromGps();
void suppressGpsTimeSync();
void openMenu();
void openDurationEditor();
void openUtcEditor();
void openBatteryOffsetEditor();
void openManualEditor();
void openSystemScreen();
bool buttonShortRelease(const ButtonState &button, unsigned long nowMs);
bool buttonEdge(const ButtonState &button);
bool buttonLongEdge(const ButtonState &button);

void setup() {
  pinMode(Pins::BUTTON_UP, INPUT_PULLUP);
  pinMode(Pins::BUTTON_DOWN, INPUT_PULLUP);
  pinMode(Pins::BUTTON_SELECT, INPUT_PULLUP);
  pinMode(Pins::PUMP_GATE, OUTPUT);
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif

  digitalWrite(Pins::PUMP_GATE, LOW);
#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, LOW);
#endif

  debouncerUp.attach(Pins::BUTTON_UP, INPUT_PULLUP);
  debouncerDown.attach(Pins::BUTTON_DOWN, INPUT_PULLUP);
  debouncerSelect.attach(Pins::BUTTON_SELECT, INPUT_PULLUP);
  debouncerUp.interval(20);
  debouncerDown.interval(20);
  debouncerSelect.interval(20);

  analogReadResolution(12);
  Wire.begin();
  initializeDisplay();
  loadPersistentSettings();
  initializeTemperatureSensor();

  initializeStartup();

  ui.lastActivityMs = millis();
  ui.screen = SCREEN_IDLE;
  setMode(MODE_IDLE);
  renderIdleScreen();
  initializeWatchdog();
}

void loop() {
  const unsigned long nowMs = millis();

  if (watchdogTimeoutMs > 0) {
    Watchdog.reset();
  }

  handleSerialCommands();
  readButtons();
  updateSensors(nowMs);
  runSafetyChecks(nowMs);
  runControlLogic(nowMs);
  applyPumpOutput(nowMs);
  updateStatusLed(nowMs);
  updateDisplayPower(nowMs);
  updateUi();
  delay(10);
}

void readButtons() {
  const unsigned long nowMs = millis();
  updateButton(buttonUp, debouncerUp, "up", nowMs);
  updateButton(buttonDown, debouncerDown, "down", nowMs);
  updateButton(buttonSelect, debouncerSelect, "select", nowMs);
}

void updateButton(ButtonState &button, Bounce &debouncer, const char *label, unsigned long nowMs) {
  debouncer.update();
  const bool activeLowPressed = debouncer.read() == LOW;
  button.changed = false;

  if (activeLowPressed && !button.pressed && !button.longPressed) {
    button.pressed = true;
    button.longPressed = false;
    button.changed = true;
    button.pressedAtMs = nowMs;
    button.lastChangeMs = nowMs;
    if (Serial) {
      Serial.print(F("button "));
      Serial.print(label);
      Serial.println(F(" fall"));
    }
    return;
  }

  if (activeLowPressed && button.pressed && !button.longPressed &&
      (nowMs - button.pressedAtMs) >= SHORT_PRESS_MAX_MS) {
    button.pressed = false;
    button.longPressed = true;
    button.changed = true;
    button.lastChangeMs = nowMs;
    if (Serial) {
      Serial.print(F("button "));
      Serial.print(label);
      Serial.println(F(" long"));
    }
    return;
  }

  if (!activeLowPressed && (button.pressed || button.longPressed)) {
    button.pressed = false;
    button.longPressed = false;
    button.changed = true;
    button.lastChangeMs = nowMs;
    if (Serial) {
      Serial.print(F("button "));
      Serial.print(label);
      Serial.println(F(" rise"));
    }
  }
}

bool buttonEdge(const ButtonState &button) {
  return button.changed && button.pressed;
}

bool buttonShortRelease(const ButtonState &button, unsigned long nowMs) {
  return button.changed && !button.pressed && !button.longPressed &&
         (nowMs - button.pressedAtMs) < SHORT_PRESS_MAX_MS;
}

bool buttonLongEdge(const ButtonState &button) {
  return button.changed && button.longPressed;
}

void updateSensors(unsigned long nowMs) {
  updateBattery(nowMs);
  updateTemperature(nowMs);
  updateGps(nowMs);
}

void updateBattery(unsigned long nowMs) {
  if (nowMs - lastBatteryReadMs < BATTERY_INTERVAL_MS) {
    return;
  }

  lastBatteryReadMs = nowMs;
  const int raw = analogRead(Pins::BATTERY_ADC);
  if (raw <= BATTERY_ADC_MIN_VALID_RAW) {
    sensors.batteryVolts = NAN;
    sensors.batteryValid = false;
    batteryHistoryCount = 0;
    batteryHistoryIndex = 0;
    return;
  }

  const float measuredVolts = readBatteryVoltage();
  if (measuredVolts < USB_POWER_MAX_VOLTS) {
    sensors.batteryVolts = NAN;
    sensors.batteryValid = false;
    batteryHistoryCount = 0;
    batteryHistoryIndex = 0;
    return;
  }

  batteryHistory[batteryHistoryIndex] = measuredVolts;
  batteryHistoryIndex = (batteryHistoryIndex + 1) % 10;
  if (batteryHistoryCount < 10) {
    batteryHistoryCount++;
  }

  float maxVolts = batteryHistory[0];
  for (uint8_t i = 1; i < batteryHistoryCount; ++i) {
    if (batteryHistory[i] > maxVolts) {
      maxVolts = batteryHistory[i];
    }
  }

  sensors.batteryVolts = maxVolts;
  sensors.batteryValid = true;
}

void updateTemperature(unsigned long nowMs) {
  const unsigned long tempIntervalMs = ui.displayOn ? TEMP_INTERVAL_DISPLAY_ON_MS : TEMP_INTERVAL_DISPLAY_OFF_MS;
  if (nowMs - lastTempReadMs < tempIntervalMs) {
    return;
  }

  lastTempReadMs = nowMs;
  sampleTemperatureNow();
}

void updateGps(unsigned long nowMs) {
  if (nowMs - lastGpsPollMs < GPS_INTERVAL_MS) {
    return;
  }

  lastGpsPollMs = nowMs;
  consumeGpsData();
  updateNextSunset();
}

bool sampleTemperatureNow() {
  waterTempSensor.requestTemperatures();

  const float tempC = waterTempSensor.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C || tempC <= -126.0f || tempC >= 125.0f) {
    sensors.tempValid = false;
    sensors.waterTempF = NAN;
    return false;
  }

  sensors.waterTempF = DallasTemperature::toFahrenheit(tempC);
  sensors.tempValid = true;
  return true;
}

void consumeGpsData() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  sensors.gpsHasFix = gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  if (gpsTimeSyncAllowed() && sensors.gpsHasFix && gps.date.isUpdated() && gps.time.isUpdated()) {
    syncClockFromGps();
  }
}

void updateNextSunset() {
  refreshSchedule(false);
}

time_t computeNextDoseEpochUtc(time_t utcNow) {
  if (timeStatus() == timeNotSet || !sensors.gpsHasFix || !gps.location.isValid()) {
    return 0;
  }

  const time_t todaySunsetUtc = sunsetEpochUtcForDate(utcNow);
  const time_t todayDoseUtc = todaySunsetUtc - SECS_PER_HOUR;
  if (utcNow < todayDoseUtc) {
    return todayDoseUtc;
  }

  const time_t tomorrowSunsetUtc = sunsetEpochUtcForDate(utcNow + SECS_PER_DAY);
  return tomorrowSunsetUtc - SECS_PER_HOUR;
}

void refreshSchedule(bool persist) {
  if (timeStatus() == timeNotSet) {
    sensors.nextSunsetHours = -1.0f;
    sensors.nextPumpRunHours = -1.0f;
    sensors.nextDoseEpochUtc = 0;
    return;
  }

  const time_t utcNow = now();

  if (!sensors.gpsHasFix || !gps.location.isValid()) {
    sensors.nextSunsetHours = -1.0f;
    sensors.nextDoseEpochUtc = settings.nextDoseEpochUtc;
    if (sensors.nextDoseEpochUtc != 0 && static_cast<time_t>(sensors.nextDoseEpochUtc) >= utcNow) {
      sensors.nextPumpRunHours = static_cast<float>(sensors.nextDoseEpochUtc - static_cast<uint32_t>(utcNow)) / 3600.0f;
    } else {
      sensors.nextPumpRunHours = -1.0f;
    }
    return;
  }

  time_t nextDoseUtc = static_cast<time_t>(settings.nextDoseEpochUtc);
  if (persist || nextDoseUtc == 0) {
    nextDoseUtc = computeNextDoseEpochUtc(utcNow);
  }

  if (nextDoseUtc == 0) {
    sensors.nextSunsetHours = -1.0f;
    sensors.nextPumpRunHours = -1.0f;
    sensors.nextDoseEpochUtc = 0;
    return;
  }

  const time_t scheduledSunsetUtc = nextDoseUtc + SECS_PER_HOUR;
  sensors.nextSunsetHours = static_cast<float>(scheduledSunsetUtc - utcNow) / 3600.0f;
  sensors.nextPumpRunHours = static_cast<float>(nextDoseUtc - utcNow) / 3600.0f;
  sensors.nextDoseEpochUtc = static_cast<uint32_t>(nextDoseUtc);

  if (persist && settings.nextDoseEpochUtc != sensors.nextDoseEpochUtc) {
    settings.nextDoseEpochUtc = sensors.nextDoseEpochUtc;
    savePersistentSettings();
  }
}

void runSafetyChecks(unsigned long nowMs) {
  if (nowMs - lastSafetyCheckMs < SAFETY_INTERVAL_MS) {
    return;
  }

  lastSafetyCheckMs = nowMs;

  if (pump.commandedOn) {
    const unsigned long currentRunMs = nowMs - pump.startedAtMs;
    if (currentRunMs > config.maxPumpContinuousMs) {
      enterFault(F("Pump runtime exceeded"));
      return;
    }

    if ((pump.accumulatedTodayMs + currentRunMs) > config.maxPumpDailyMs) {
      enterFault(F("Daily dose exceeded"));
      return;
    }
  }
}

void runControlLogic(unsigned long nowMs) {
  if (!ui.displayOn) {
    const bool anyButtonActivity =
      buttonEdge(buttonUp) || buttonEdge(buttonDown) || buttonEdge(buttonSelect) ||
      buttonShortRelease(buttonUp, nowMs) || buttonShortRelease(buttonDown, nowMs) ||
      buttonShortRelease(buttonSelect, nowMs) ||
      buttonLongEdge(buttonUp) || buttonLongEdge(buttonDown) || buttonLongEdge(buttonSelect);

    if (systemMode == MODE_DOSING && anyButtonActivity) {
      setDisplayPower(true);
      pump.keepDisplayOn = true;
      noteUserActivity(nowMs);
      return;
    }

    if (buttonShortRelease(buttonSelect, nowMs)) {
      setDisplayPower(true);
      noteUserActivity(nowMs);
      return;
    }
    return;
  }

  if (buttonEdge(buttonUp) || buttonEdge(buttonDown) || buttonEdge(buttonSelect) ||
      buttonShortRelease(buttonUp, nowMs) || buttonShortRelease(buttonDown, nowMs) ||
      buttonShortRelease(buttonSelect, nowMs) ||
      buttonLongEdge(buttonSelect)) {
    noteUserActivity(nowMs);
  }

  if (systemMode == MODE_IDLE &&
      settings.runDurationMinutes > 0 &&
      sensors.nextDoseEpochUtc != 0 &&
      now() >= static_cast<time_t>(sensors.nextDoseEpochUtc)) {
    startPumpRunMinutes(settings.runDurationMinutes, nowMs);
    Serial.println(F("Scheduled pump run started."));
    return;
  }

  if (ui.screen == SCREEN_IDLE && systemMode != MODE_DOSING && buttonLongEdge(buttonSelect)) {
    openMenu();
    return;
  }

  if (ui.screen == SCREEN_MENU) {
    if (buttonEdge(buttonUp) && ui.menuIndex > 0) {
      ui.menuIndex--;
      if (ui.menuIndex < ui.menuScroll) {
        ui.menuScroll = ui.menuIndex;
      }
    }
    if (buttonEdge(buttonDown) && ui.menuIndex < (MENU_ITEM_COUNT - 1)) {
      ui.menuIndex++;
      if (ui.menuIndex >= (ui.menuScroll + 5)) {
        ui.menuScroll = ui.menuIndex - 4;
      }
    }
    if (buttonShortRelease(buttonSelect, nowMs)) {
      switch (ui.menuIndex) {
        case 0:
          openDurationEditor();
          return;
        case 1:
          openUtcEditor();
          return;
        case 2:
          openBatteryOffsetEditor();
          return;
        case 3:
          openManualEditor();
          return;
        case 4:
          openSystemScreen();
          return;
        case 5:
          performRunup();
          ui.screen = SCREEN_IDLE;
          return;
        case 6:
          restartController();
          return;
        case 7:
          ui.screen = SCREEN_IDLE;
          return;
      }
    }
    if (buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_IDLE;
      return;
    }
    return;
  }

  if (ui.screen == SCREEN_EDIT_DURATION) {
    if (buttonEdge(buttonUp) && ui.editDurationMinutes < 30) {
      ui.editDurationMinutes++;
    }
    if (buttonEdge(buttonDown) && ui.editDurationMinutes > 0) {
      ui.editDurationMinutes--;
    }
    if (buttonShortRelease(buttonSelect, nowMs)) {
      settings.runDurationMinutes = ui.editDurationMinutes;
      savePersistentSettings();
      ui.screen = SCREEN_MENU;
    }
    if (buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_MENU;
    }
    return;
  }

  if (ui.screen == SCREEN_EDIT_UTC) {
    if (buttonEdge(buttonUp) && ui.editUtcOffsetHours < 12) {
      ui.editUtcOffsetHours++;
    }
    if (buttonEdge(buttonDown) && ui.editUtcOffsetHours > -12) {
      ui.editUtcOffsetHours--;
    }
    if (buttonShortRelease(buttonSelect, nowMs)) {
      settings.utcOffsetHours = ui.editUtcOffsetHours;
      savePersistentSettings();
      ui.screen = SCREEN_MENU;
    }
    if (buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_MENU;
    }
    return;
  }

  if (ui.screen == SCREEN_EDIT_VOFFSET) {
    if (buttonEdge(buttonUp) && ui.editBatteryOffsetTenths < 50) {
      ui.editBatteryOffsetTenths++;
    }
    if (buttonEdge(buttonDown) && ui.editBatteryOffsetTenths > -50) {
      ui.editBatteryOffsetTenths--;
    }
    if (buttonShortRelease(buttonSelect, nowMs)) {
      settings.batteryOffsetTenths = ui.editBatteryOffsetTenths;
      savePersistentSettings();
      ui.screen = SCREEN_MENU;
    }
    if (buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_MENU;
    }
    return;
  }

  if (ui.screen == SCREEN_EDIT_MANUAL) {
    if (buttonEdge(buttonUp) && ui.editManualMinutes < 30) {
      ui.editManualMinutes++;
    }
    if (buttonEdge(buttonDown) && ui.editManualMinutes > 0) {
      ui.editManualMinutes--;
    }
    if (buttonShortRelease(buttonSelect, nowMs)) {
      ui.screen = SCREEN_IDLE;
      if (ui.editManualMinutes > 0) {
        startPumpRunMinutes(ui.editManualMinutes, nowMs);
      }
    }
    if (buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_MENU;
    }
    return;
  }

  if (ui.screen == SCREEN_SYSTEM) {
    if (buttonEdge(buttonUp) && ui.systemScroll > 0) {
      ui.systemScroll--;
    }
    if (buttonEdge(buttonDown) && ui.systemScroll < 1) {
      ui.systemScroll++;
    }
    if (buttonShortRelease(buttonSelect, nowMs) || buttonLongEdge(buttonSelect)) {
      ui.screen = SCREEN_MENU;
    }
    return;
  }

  if (systemMode == MODE_DOSING) {
    pump.commandedOn = true;
    if ((nowMs - pump.startedAtMs) >= pump.targetRunMs) {
      pump.commandedOn = false;
      setMode(MODE_IDLE);
    }
    return;
  }

  if (systemMode == MODE_LOW_BATTERY || systemMode == MODE_FAULT) {
    pump.commandedOn = false;
    return;
  }

  pump.commandedOn = false;
}

void applyPumpOutput(unsigned long nowMs) {
  if (pump.commandedOn && !pump.enabled) {
    pump.enabled = true;
    pump.startedAtMs = nowMs;
    digitalWrite(Pins::PUMP_GATE, HIGH);
    Serial.println(F("Pump ON"));
  } else if (!pump.commandedOn && pump.enabled) {
    pump.enabled = false;
    pump.accumulatedTodayMs += (nowMs - pump.startedAtMs);
    digitalWrite(Pins::PUMP_GATE, LOW);
    if (pump.keepDisplayOn) {
      noteUserActivity(nowMs);
      pump.keepDisplayOn = false;
    }
    Serial.println(F("Pump OFF"));
  }
}

void updateUi() {
  if (!ui.displayOn) {
    return;
  }

  switch (ui.screen) {
    case SCREEN_IDLE:
      renderIdleScreen();
      return;
    case SCREEN_MENU:
      renderMenuScreen();
      return;
    case SCREEN_EDIT_DURATION:
      renderDurationScreen();
      return;
    case SCREEN_EDIT_UTC:
      renderUtcScreen();
      return;
    case SCREEN_EDIT_VOFFSET:
      renderBatteryOffsetScreen();
      return;
    case SCREEN_EDIT_MANUAL:
      renderManualScreen();
      return;
    case SCREEN_SYSTEM:
      renderSystemScreen();
      return;
    case SCREEN_FAULT:
    default:
      renderIdleScreen();
      return;
  }
}

void updateDisplayPower(unsigned long nowMs) {
  if (!ui.displayOn) {
    return;
  }

  if (pump.enabled && pump.keepDisplayOn) {
    return;
  }

  if (isMenuScreen(ui.screen) && (nowMs - ui.lastActivityMs) >= MENU_TIMEOUT_MS) {
    ui.screen = SCREEN_IDLE;
    return;
  }

  if ((nowMs - ui.lastActivityMs) >= DISPLAY_TIMEOUT_MS) {
    setDisplayPower(false);
  }
}

void updateStatusLed(unsigned long nowMs) {
#ifdef LED_BUILTIN
  bool ledOn = false;

  if (pump.enabled && pump.usbHeartbeatLed) {
    const unsigned long phaseMs = nowMs % 1200UL;
    ledOn = (phaseMs < 80UL) || (phaseMs >= 200UL && phaseMs < 280UL);
  }

  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
#else
  (void)nowMs;
#endif
}

void handleSerialCommands() {
  static char commandBuffer[32] = {};
  static uint8_t commandLength = 0;

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r' || incoming == '\n') {
      if (commandLength == 0) {
        continue;
      }

      commandBuffer[commandLength] = '\0';
      processSerialCommand(commandBuffer);

      commandLength = 0;
      commandBuffer[0] = '\0';
      continue;
    }

    if (commandLength < (sizeof(commandBuffer) - 1)) {
      commandBuffer[commandLength++] = incoming;
    }
  }
}

void processSerialCommand(const char *command) {
  if (strcmp(command, "help") == 0) {
    Serial.println(F("Available commands:"));
    Serial.println(F("  help"));
    Serial.println(F("  state"));
    Serial.println(F("  time HH:MM"));
    Serial.println(F("  date YYYY-MM-DD"));
    Serial.println(F("  runup"));
    Serial.println(F("  utc <offset>      (-12 to 12)"));
    Serial.println(F("  duration <mins>   (0 to 30)"));
    Serial.println(F("  pump <mins>       (0 to 30)"));
    Serial.println(F("  pump off"));
    Serial.println(F("  restart"));
    Serial.println(F("  starve"));
    return;
  }

  if (strcmp(command, "state") == 0) {
    printState();
    return;
  }

  if (strcmp(command, "starve") == 0) {
    Serial.println(F("Starving watchdog for 17 seconds..."));
    delay(17000);
    Serial.println(F("Watchdog did not fire."));
    return;
  }

  if (strcmp(command, "restart") == 0) {
    Serial.println(F("Restarting controller..."));
    delay(100);
    restartController();
    return;
  }

  if (strcmp(command, "runup") == 0) {
    if (!performRunup()) {
      Serial.println(F("No next pump run is currently available."));
      return;
    }
    Serial.println(F("Runup complete."));
    return;
  }

  int value = 0;
  int hh = 0;
  int mm = 0;
  int yyyy = 0;
  int monthValue = 0;
  int dd = 0;

  if (sscanf(command, "time %d:%d", &hh, &mm) == 2) {
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59) {
      Serial.println(F("Time must be HH:MM in 24-hour format."));
      return;
    }

    const time_t localEpoch = localTimeNow();
    const int currentDay = (timeStatus() != timeNotSet) ? day(localEpoch) : 1;
    const int currentMonth = (timeStatus() != timeNotSet) ? month(localEpoch) : 1;
    const int currentYear = (timeStatus() != timeNotSet) ? year(localEpoch) : 2026;

    setTime(hh, mm, 0, currentDay, currentMonth, currentYear);
    adjustTime(-static_cast<long>(settings.utcOffsetHours) * SECS_PER_HOUR);
    suppressGpsTimeSync();
    refreshSchedule(true);

    Serial.print(F("Local time set to "));
    if (hh < 10) {
      Serial.print(F("0"));
    }
    Serial.print(hh);
    Serial.print(F(":"));
    if (mm < 10) {
      Serial.print(F("0"));
    }
    Serial.println(mm);
    return;
  }

  if (sscanf(command, "date %d-%d-%d", &yyyy, &monthValue, &dd) == 3) {
    if (yyyy < 2000 || yyyy > 2099 || monthValue < 1 || monthValue > 12 || dd < 1 || dd > 31) {
      Serial.println(F("Date must be YYYY-MM-DD."));
      return;
    }

    const time_t utcNow = now();
    const int currentHour = (timeStatus() != timeNotSet) ? hour(utcNow) : 0;
    const int currentMinute = (timeStatus() != timeNotSet) ? minute(utcNow) : 0;
    const int currentSecond = (timeStatus() != timeNotSet) ? second(utcNow) : 0;

    setTime(currentHour, currentMinute, currentSecond, dd, monthValue, yyyy);
    suppressGpsTimeSync();
    refreshSchedule(true);

    Serial.print(F("UTC date set to "));
    Serial.print(yyyy);
    Serial.print(F("-"));
    if (monthValue < 10) {
      Serial.print(F("0"));
    }
    Serial.print(monthValue);
    Serial.print(F("-"));
    if (dd < 10) {
      Serial.print(F("0"));
    }
    Serial.println(dd);
    return;
  }

  if (strcmp(command, "pump off") == 0) {
    stopPumpRun();
    Serial.println(F("Pump run stopped."));
    return;
  }

  if (sscanf(command, "pump %d", &value) == 1) {
    if (value < 0 || value > 30) {
      Serial.println(F("Pump minutes must be between 0 and 30."));
      return;
    }

    if (value == 0) {
      stopPumpRun();
      Serial.println(F("Pump run stopped."));
      return;
    }

    startPumpRunMinutes(static_cast<uint16_t>(value), millis());
    Serial.print(F("Pump run started for "));
    Serial.print(value);
    Serial.println(F(" minutes."));
    return;
  }

  if (sscanf(command, "utc %d", &value) == 1) {
    if (value < -12 || value > 12) {
      Serial.println(F("UTC offset must be between -12 and 12."));
      return;
    }

    settings.utcOffsetHours = static_cast<int8_t>(value);
    savePersistentSettings();
    Serial.print(F("UTC offset set to "));
    Serial.println(settings.utcOffsetHours);
    return;
  }

  if (sscanf(command, "duration %d", &value) == 1) {
    if (value < 0 || value > 30) {
      Serial.println(F("Duration must be between 0 and 30 minutes."));
      return;
    }

    settings.runDurationMinutes = static_cast<uint16_t>(value);
    savePersistentSettings();
    Serial.print(F("Run duration set to "));
    Serial.print(settings.runDurationMinutes);
    Serial.println(F(" minutes."));
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(command);
}

void printState() {
  char lastDose[24];
  formatLastDoseText(lastDose, sizeof(lastDose), "");

  Serial.print(F("Mode="));
  Serial.print(modeName(systemMode));
  Serial.print(F(" Restart="));
  Serial.print(restartReasonName(startup.restartReason));
  Serial.print(F(" UTC="));
  Serial.print(settings.utcOffsetHours);
  Serial.print(F(" RunMin="));
  Serial.print(settings.runDurationMinutes);
  Serial.print(F(" Restarts="));
  Serial.print(settings.restartCount);
  Serial.print(F(" Crashes="));
  Serial.print(settings.crashCount);
  Serial.print(F(" LastDose="));
  Serial.print(lastDose);
  Serial.print(F(" NextDoseUtc="));
  Serial.print(settings.nextDoseEpochUtc);
  Serial.print(F(" Temp="));
  if (sensors.tempValid) {
    Serial.print(static_cast<int>(roundf(sensors.waterTempF)));
    Serial.print(F("F"));
  } else {
    Serial.print(F("--"));
  }
  Serial.print(F(" Battery="));
  Serial.print(sensors.batteryVolts, 2);
  Serial.print(F("V Pump="));
  Serial.println(pump.enabled ? F("ON") : F("OFF"));
}

void loadPersistentSettings() {
  settings = settingsStorage.read();

  const bool uninitialized = settings.magic != SETTINGS_MAGIC;
  const bool invalidUtcOffset = settings.utcOffsetHours < -12 || settings.utcOffsetHours > 12;
  const bool invalidRunDuration = settings.runDurationMinutes > 30U;
  const bool invalidBatteryOffset = settings.batteryOffsetTenths < -50 || settings.batteryOffsetTenths > 50;
  const bool invalidLastDose = settings.lastDoseEpochUtc > 4102444800UL;
  const bool invalidNextDose = settings.nextDoseEpochUtc > 4102444800UL;

  if (uninitialized || invalidUtcOffset || invalidRunDuration || invalidBatteryOffset || invalidLastDose || invalidNextDose) {
    settings.magic = SETTINGS_MAGIC;
    settings.utcOffsetHours = -5;
    settings.runDurationMinutes = 10;
    settings.batteryOffsetTenths = 12;
    if (uninitialized) {
      settings.restartCount = 0;
      settings.crashCount = 0;
    }
    settings.lastDoseEpochUtc = 0;
    settings.nextDoseEpochUtc = 0;
    savePersistentSettings();
    startupLogLine("Settings: defaults");
    return;
  }

  startupLogLine("Settings: loaded");
}

void savePersistentSettings() {
  settingsStorage.write(settings);
}

void updateRestartCounters() {
  settings.restartCount++;

  if (startup.restartReason == RESTART_BROWNOUT || startup.restartReason == RESTART_WATCHDOG) {
    settings.crashCount++;
  }

  savePersistentSettings();
}

void initializeWatchdog() {
  watchdogTimeoutMs = Watchdog.enable(16000);

  if (Serial) {
    Serial.print(F("Watchdog enabled: "));
    Serial.print(watchdogTimeoutMs);
    Serial.println(F(" ms"));
  }
}

void restartController() {
#if defined(ARDUINO_ARCH_SAMD)
  NVIC_SystemReset();
#else
  Watchdog.enable(16);
  while (true) {
  }
#endif
}

bool isMenuScreen(ScreenId screen) {
  return screen == SCREEN_MENU ||
         screen == SCREEN_EDIT_DURATION ||
         screen == SCREEN_EDIT_UTC ||
         screen == SCREEN_EDIT_VOFFSET ||
         screen == SCREEN_EDIT_MANUAL ||
         screen == SCREEN_SYSTEM;
}

void formatLastDoseText(char *buffer, size_t size, const char *prefix) {
  if (settings.lastDoseEpochUtc == 0) {
    snprintf(buffer, size, "%sNEVER", prefix);
    return;
  }

  const time_t localDoseTime =
    static_cast<time_t>(settings.lastDoseEpochUtc) + (static_cast<long>(settings.utcOffsetHours) * SECS_PER_HOUR);

  snprintf(
    buffer,
    size,
    "%s%02d %s %02d:%02d",
    prefix,
    day(localDoseTime),
    monthShortStr(month(localDoseTime)),
    hour(localDoseTime),
    minute(localDoseTime)
  );
}

void formatNextDoseText(char *buffer, size_t size, const char *prefix) {
  if (settings.nextDoseEpochUtc == 0) {
    snprintf(buffer, size, "%sUNKNOWN", prefix);
    return;
  }

  const time_t localDoseTime =
    static_cast<time_t>(settings.nextDoseEpochUtc) + (static_cast<long>(settings.utcOffsetHours) * SECS_PER_HOUR);

  snprintf(
    buffer,
    size,
    "%s%02d %s %02d:%02d",
    prefix,
    day(localDoseTime),
    monthShortStr(month(localDoseTime)),
    hour(localDoseTime),
    minute(localDoseTime)
  );
}

time_t midnightUtcFor(time_t utc) {
  return utc - (static_cast<long>(hour(utc)) * SECS_PER_HOUR) -
         (static_cast<long>(minute(utc)) * SECS_PER_MIN) -
         static_cast<long>(second(utc));
}

time_t sunsetEpochUtcForDate(time_t dateUtc) {
  double transit = 0.0;
  double sunrise = 0.0;
  double sunset = 0.0;
  calcSunriseSunset(dateUtc, gps.location.lat(), gps.location.lng(), transit, sunrise, sunset);

  long sunsetSeconds = lround(sunset * 3600.0);
  time_t sunsetEpochUtc = midnightUtcFor(dateUtc) + sunsetSeconds;

  if (sunsetEpochUtc < midnightUtcFor(dateUtc)) {
    sunsetEpochUtc += SECS_PER_DAY;
  }

  return sunsetEpochUtc;
}

void startPumpRunMinutes(uint16_t minutes, unsigned long nowMs) {
  ui.screen = SCREEN_IDLE;
  pump.targetRunMs = static_cast<unsigned long>(minutes) * 60UL * 1000UL;
  pump.commandedOn = true;
  pump.keepDisplayOn = ui.displayOn;
  pump.usbHeartbeatLed = !sensors.batteryValid;
  pump.startedAtMs = nowMs;
  settings.lastDoseEpochUtc = static_cast<uint32_t>(now());
  savePersistentSettings();
  refreshSchedule(true);
  setMode(MODE_DOSING);
}

void stopPumpRun() {
  pump.commandedOn = false;
  pump.targetRunMs = 0;
  pump.keepDisplayOn = false;
  pump.usbHeartbeatLed = false;
  ui.screen = SCREEN_IDLE;
  setMode(MODE_IDLE);
}

bool performRunup() {
  if (sensors.nextDoseEpochUtc == 0 || timeStatus() == timeNotSet) {
    return false;
  }

  const long secondsUntilRun = static_cast<long>(static_cast<int64_t>(sensors.nextDoseEpochUtc) - static_cast<int64_t>(now()));
  if (secondsUntilRun <= 0) {
    return false;
  }

  long secondsToAdvance = secondsUntilRun - RUNUP_TARGET_LEAD_SECONDS;
  if (secondsToAdvance < 0) {
    secondsToAdvance = 0;
  }

  adjustTime(secondsToAdvance);
  suppressGpsTimeSync();
  updateNextSunset();

  long secondsRemaining = -1;
  if (sensors.nextDoseEpochUtc != 0) {
    secondsRemaining = static_cast<long>(static_cast<int64_t>(sensors.nextDoseEpochUtc) - static_cast<int64_t>(now()));
  }

  if (secondsRemaining >= 0 && secondsRemaining <= 30L) {
    const long secondsToBackUp = RUNUP_MIN_LEAD_SECONDS - secondsRemaining;
    if (secondsToBackUp > 0) {
      adjustTime(-secondsToBackUp);
      updateNextSunset();
      if (sensors.nextDoseEpochUtc != 0) {
        secondsRemaining = static_cast<long>(static_cast<int64_t>(sensors.nextDoseEpochUtc) - static_cast<int64_t>(now()));
      }
    }
  }

  if (Serial) {
    Serial.print(F("Advanced clock by "));
    Serial.print(secondsToAdvance);
    Serial.println(F(" seconds."));
    Serial.print(F("Next pump run in "));
    Serial.print(sensors.nextPumpRunHours, 3);
    Serial.println(F(" hours."));
  }

  return true;
}

bool gpsTimeSyncAllowed() {
  return millis() >= startup.gpsTimeSuppressedUntilMs;
}

float readBatteryVoltage() {
  
  // throw the first read away, then compute the average over 20 ms.
  analogRead(Pins::BATTERY_ADC);
  int raw = 0;
  for (int i = 0; i < 10; i++) {
    delay(2);
    raw += analogRead(Pins::BATTERY_ADC);
  }
  raw /= 10;
  // Serial.printf("avg raw: %d\n", raw);

  const float adcMax = 4095.0f;
  const float measuredAtPin = (raw / adcMax) * config.adcReferenceVolts;
  float rawVolts = (raw * 3300.0f) / 4095.0f / 1000.0f * config.batteryDividerRatio +
                   (static_cast<float>(settings.batteryOffsetTenths) / 10.0f);
  float reportedMeasuredVolts = measuredAtPin * config.batteryDividerRatio;
  // Serial.printf("Volts %.1f, %.1f, %.1f\n", measuredVolts, reportedMeasuredVolts, config.adcReferenceVolts)
  return rawVolts;
}

void setMode(SystemMode nextMode) {
  if (systemMode == nextMode) {
    return;
  }

  systemMode = nextMode;

  Serial.print(F("Mode -> "));
  Serial.println(modeName(systemMode));
}

void enterFault(const __FlashStringHelper *message) {
  pump.commandedOn = false;
  setMode(MODE_FAULT);
  Serial.print(F("FAULT: "));
  Serial.println(message);
}

const __FlashStringHelper *modeName(SystemMode mode) {
  switch (mode) {
    case MODE_BOOT:
      return F("BOOT");
    case MODE_IDLE:
      return F("IDLE");
    case MODE_DOSING:
      return F("DOSING");
    case MODE_LOW_BATTERY:
      return F("LOW_BATTERY");
    case MODE_FAULT:
      return F("FAULT");
    default:
      return F("UNKNOWN");
  }
}

RestartReason detectRestartReason(uint8_t &rawCause) {
#if defined(ARDUINO_ARCH_SAMD)
  rawCause = PM->RCAUSE.reg;

  if (rawCause & PM_RCAUSE_POR) {
    return RESTART_POWER_ON;
  }
  if (rawCause & PM_RCAUSE_BOD12) {
    return RESTART_BROWNOUT;
  }
  if (rawCause & PM_RCAUSE_BOD33) {
    return RESTART_BROWNOUT;
  }
  if (rawCause & PM_RCAUSE_EXT) {
    return RESTART_EXTERNAL;
  }
  if (rawCause & PM_RCAUSE_WDT) {
    return RESTART_WATCHDOG;
  }
  if (rawCause & PM_RCAUSE_SYST) {
    return RESTART_SOFTWARE;
  }
#ifdef PM_RCAUSE_BACKUP
  if (rawCause & PM_RCAUSE_BACKUP) {
    return RESTART_BACKUP;
  }
#endif
#else
  rawCause = 0;
#endif

  return RESTART_UNKNOWN;
}

const __FlashStringHelper *restartReasonName(RestartReason reason) {
  switch (reason) {
    case RESTART_POWER_ON:
      return F("POWER_ON");
    case RESTART_BACKUP:
      return F("BACKUP");
    case RESTART_WATCHDOG:
      return F("WATCHDOG");
    case RESTART_SOFTWARE:
      return F("SOFTWARE");
    case RESTART_EXTERNAL:
      return F("EXTERNAL");
    case RESTART_BROWNOUT:
      return F("BROWNOUT");
    case RESTART_UNKNOWN:
    default:
      return F("UNKNOWN");
  }
}

void initializeStartup() {
  char line[STARTUP_LINE_CHARS + 1];

  startup.restartReason = detectRestartReason(startup.rawResetCause);
  updateRestartCounters();
  startupLogLine("Chlorduino boot");
  snprintf(line, sizeof(line), "Reset: %s", restartReasonText(startup.restartReason));
  startupLogLine(line);
  snprintf(line, sizeof(line), "Boots: %lu", static_cast<unsigned long>(settings.restartCount));
  startupLogLine(line);
  snprintf(line, sizeof(line), "Crashes: %lu", static_cast<unsigned long>(settings.crashCount));
  startupLogLine(line);
  formatLastDoseText(line, sizeof(line), "Dose: ");
  startupLogLine(line);

  Serial.begin(115200);
  startup.serialAttached = attachDebugSerial(SERIAL_ATTACH_TIMEOUT_MS);

  Serial1.begin(9600);
  printStartupBanner();
  waitForGpsStartupLock();
  waitForTemperatureStartupLock();
}

bool attachDebugSerial(unsigned long timeoutMs) {
  const unsigned long startedAtMs = millis();

  while (!Serial && (millis() - startedAtMs) < timeoutMs) {
    delay(10);
  }

  return static_cast<bool>(Serial);
}

void printStartupBanner() {
  char line[32];

  if (!Serial) {
    return;
  }

  Serial.println();
  Serial.println(F("Chlorduino booted."));
  Serial.print(F("Restart reason: "));
  Serial.println(restartReasonName(startup.restartReason));
  Serial.print(F("Reset cause register: 0x"));
  Serial.println(startup.rawResetCause, HEX);
  Serial.print(F("Debug serial attached: "));
  Serial.println(startup.serialAttached ? F("yes") : F("no"));
  formatLastDoseText(line, sizeof(line), "Last dose: ");
  Serial.println(line);
  Serial.println(F("GPS on Serial1 at 9600 baud."));
}

void waitForGpsStartupLock() {
  const unsigned long startedAtMs = millis();
  unsigned long lastStatusPrintMs = 0;
  unsigned long spinnerTick = 0;
  startupLogLine("Waiting for GPS...");

  if (Serial) {
    Serial.println(F("Waiting for GPS lock and valid UTC time..."));
  }

  while (true) {
    consumeGpsData();

    const unsigned long nowMs = millis();
    const bool lockReady = gpsStartupLockAcquired();
    const bool minSpinnerElapsed = (nowMs - startedAtMs) >= MIN_GPS_SPINNER_MS;
    if (Serial && (nowMs - lastStatusPrintMs) >= 1000) {
      lastStatusPrintMs = nowMs;
      Serial.print(F("GPS wait: sats="));
      Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0);
      Serial.print(F(" location="));
      Serial.print(gps.location.isValid() ? F("ok") : F("..."));
      Serial.print(F(" date="));
      Serial.print(gps.date.isValid() ? F("ok") : F("..."));
      Serial.print(F(" time="));
      Serial.println(gps.time.isValid() ? F("ok") : F("..."));
    }

    char spinnerLine[STARTUP_LINE_CHARS + 1];
    formatGpsSpinnerLine(spinnerLine, sizeof(spinnerLine), spinnerTick++);
    renderStartupScreen(spinnerLine);

    if (lockReady && minSpinnerElapsed) {
      break;
    }

    delay(20);
  }

  startup.gpsLocked = true;
  startup.gpsLockWaitMs = millis() - startedAtMs;

  sensors.gpsHasFix = true;
  syncClockFromGps();
  refreshSchedule(true);
  startupLogLine("GPS lock acquired");

  if (Serial) {
    char nextDoseLine[32];
    Serial.print(F("GPS lock acquired after "));
    Serial.print(startup.gpsLockWaitMs);
    Serial.println(F(" ms."));
    Serial.print(F("Latitude: "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Longitude: "));
    Serial.println(gps.location.lng(), 6);
    Serial.print(F("UTC date: "));
    Serial.print(gps.date.year());
    Serial.print(F("-"));
    if (gps.date.month() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.date.month());
    Serial.print(F("-"));
    if (gps.date.day() < 10) {
      Serial.print(F("0"));
    }
    Serial.println(gps.date.day());
    Serial.print(F("UTC time: "));
    if (gps.time.hour() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) {
      Serial.print(F("0"));
    }
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) {
      Serial.print(F("0"));
    }
    Serial.println(gps.time.second());
    formatNextDoseText(nextDoseLine, sizeof(nextDoseLine), "Next dose: ");
    Serial.println(nextDoseLine);
  }

  char line[STARTUP_LINE_CHARS + 1];
  snprintf(line, sizeof(line), "Lock in %lus", startup.gpsLockWaitMs / 1000UL);
  startupLogLine(line);
  formatNextDoseText(line, sizeof(line), "Next: ");
  startupLogLine(line);
}

void waitForTemperatureStartupLock() {
  const unsigned long startedAtMs = millis();
  unsigned long lastStatusPrintMs = 0;
  unsigned long lastSampleAttemptMs = 0;
  unsigned long spinnerTick = 0;
  startupLogLine("Waiting for temp...");

  if (Serial) {
    Serial.println(F("Waiting for temperature reading..."));
  }

  while (true) {
    const unsigned long nowMs = millis();
    if ((nowMs - lastSampleAttemptMs) >= TEMP_STARTUP_RETRY_MS || lastSampleAttemptMs == 0) {
      lastSampleAttemptMs = nowMs;
      sampleTemperatureNow();
    }

    const bool tempReady = temperatureStartupLockAcquired();
    const bool minSpinnerElapsed = (nowMs - startedAtMs) >= MIN_TEMP_SPINNER_MS;

    if (Serial && (nowMs - lastStatusPrintMs) >= 1000) {
      lastStatusPrintMs = nowMs;
      Serial.print(F("Temp wait: "));
      if (sensors.tempValid) {
        Serial.print(static_cast<int>(roundf(sensors.waterTempF)));
        Serial.println(F("F"));
      } else {
        Serial.println(F("..."));
      }
    }

    char spinnerLine[STARTUP_LINE_CHARS + 1];
    if (sensors.tempValid) {
      snprintf(spinnerLine, sizeof(spinnerLine), "[%c] Temp: %dF", "|/-\\"[spinnerTick % 4], static_cast<int>(roundf(sensors.waterTempF)));
    } else {
      snprintf(spinnerLine, sizeof(spinnerLine), "[%c] Temp acquiring", "|/-\\"[spinnerTick % 4]);
    }
    spinnerTick++;
    renderStartupScreen(spinnerLine);

    if (tempReady && minSpinnerElapsed) {
      break;
    }

    delay(20);
  }

  startupLogLine("Temp acquired");

  char line[STARTUP_LINE_CHARS + 1];
  if (sensors.tempValid) {
    snprintf(line, sizeof(line), "Temp: %d F", static_cast<int>(roundf(sensors.waterTempF)));
  } else {
    snprintf(line, sizeof(line), "Temp: unavailable");
  }
  startupLogLine(line);
  renderStartupScreen(nullptr);
  delay(POST_LOCK_SCREEN_MS);
}

bool gpsStartupLockAcquired() {
  return gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
}

bool temperatureStartupLockAcquired() {
  return sensors.tempValid;
}

void initializeDisplay() {
  oled.begin();
  oled.setPowerSave(0);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "Booting...");
  oled.sendBuffer();
}

void initializeTemperatureSensor() {
  waterTempSensor.begin();
  waterTempSensor.setWaitForConversion(true);
  sensors.tempSensorCount = static_cast<uint8_t>(waterTempSensor.getDeviceCount());

  if (sensors.tempSensorCount > 0) {
    startupLogLine("Temp sensor found");
  } else {
    startupLogLine("Temp sensor missing");
  }
}

void startupLogLine(const char *message) {
  waitForStartupLineDwell();

  if (startupLogCount < STARTUP_LOG_LINES) {
    snprintf(startupLog[startupLogCount], sizeof(startupLog[startupLogCount]), "%s", message);
    startupLogCount++;
  } else {
    for (uint8_t i = 1; i < STARTUP_LOG_LINES; ++i) {
      snprintf(startupLog[i - 1], sizeof(startupLog[i - 1]), "%s", startupLog[i]);
    }
    snprintf(startupLog[STARTUP_LOG_LINES - 1], sizeof(startupLog[STARTUP_LOG_LINES - 1]), "%s", message);
  }

  lastStartupLogMs = millis();
  renderStartupScreen();
}

void renderStartupScreen(const char *statusLine) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);

  for (uint8_t i = 0; i < startupLogCount && i < STARTUP_LOG_LINES; ++i) {
    const uint8_t y = static_cast<uint8_t>((i + 1) * 10);
    oled.drawStr(0, y, startupLog[i]);
  }

  if (statusLine != nullptr) {
    oled.drawBox(0, 54, 128, 10);
    oled.setDrawColor(0);
    oled.drawStr(0, 63, statusLine);
    oled.setDrawColor(1);
  }

  oled.sendBuffer();
}

void renderIdleScreen() {
  char line[24];
  const time_t localEpoch = localTimeNow();
  const bool pumpRowPulseOn =
    systemMode == MODE_DOSING && pump.enabled && ((millis() / 500UL) % 2UL) == 0UL;

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);

  if (timeStatus() != timeNotSet) {
    snprintf(line, sizeof(line), "%02d:%02d:%02d", hour(localEpoch), minute(localEpoch), second(localEpoch));
    oled.drawStr(0, 10, line);

	    snprintf(line, sizeof(line), "%02d %s", day(localEpoch), monthShortStr(month(localEpoch)));
    oled.drawStr(59, 10, line);

	    if (sensors.tempValid) {
	      snprintf(line, sizeof(line), "%dF", static_cast<int>(roundf(sensors.waterTempF)));
	    } else {
	      snprintf(line, sizeof(line), "--F");
	    }
	    oled.drawStr(104, 10, line);
	  } else {
	    oled.drawStr(0, 10, "--:--:--");
	    oled.drawStr(59, 10, "-- ---");
	    oled.drawStr(104, 10, "--F");
	  }

  snprintf(
    line,
    sizeof(line),
    "%lu",
    gps.satellites.isValid() ? static_cast<unsigned long>(gps.satellites.value()) : 0UL
  );
  oled.setFont(u8g2_font_open_iconic_all_1x_t);
  oled.drawGlyph(0, 21, 175);
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(10, 21, line);
  if (!gpsTimeSyncAllowed()) {
    oled.drawLine(10, 16, 26, 16);
  }

  oled.setFont(u8g2_font_open_iconic_all_1x_t);
  oled.drawGlyph(58, 21, 206);
  oled.setFont(u8g2_font_6x10_tr);
  snprintf(line, sizeof(line), "%umin", settings.runDurationMinutes);
  oled.drawStr(68, 21, line);

  if (sensors.batteryValid) {
    snprintf(line, sizeof(line), "%.1fV", sensors.batteryVolts);
  } else {
    snprintf(line, sizeof(line), "USB");
  }
  oled.setFont(u8g2_font_open_iconic_all_1x_t);
  oled.drawGlyph(0, 32, 90);
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(10, 32, line);

  oled.setFont(u8g2_font_open_iconic_all_1x_t);
  oled.drawGlyph(58, 32, 269);
  oled.setFont(u8g2_font_6x10_tr);
  snprintf(line, sizeof(line), "%+dh", settings.utcOffsetHours);
  oled.drawStr(68, 32, line);
  const unsigned long uptimeMs = millis();
  if (uptimeMs <= (5UL * 60UL * 60UL * 1000UL)) {
    snprintf(line, sizeof(line), "%lum", uptimeMs / 60000UL);
  } else if (uptimeMs < (2UL * 24UL * 60UL * 60UL * 1000UL)) {
    snprintf(line, sizeof(line), "%luh", uptimeMs / 3600000UL);
  } else {
    snprintf(line, sizeof(line), "%.1fd", uptimeMs / 86400000.0f);
  }
  oled.drawStr(92, 32, line);

  if (pumpRowPulseOn) {
    oled.drawBox(0, 45, 128, 12);
    oled.setDrawColor(0);
  }

  oled.setFont(u8g2_font_open_iconic_all_1x_t);
  oled.drawGlyph(0, 54, 241);
  oled.setFont(u8g2_font_6x10_tr);
  if (systemMode == MODE_DOSING && pump.enabled) {
    unsigned long remainingMs = 0;
    if (pump.targetRunMs > (millis() - pump.startedAtMs)) {
      remainingMs = pump.targetRunMs - (millis() - pump.startedAtMs);
    }
    const unsigned long remainingSeconds = (remainingMs + 999UL) / 1000UL;
    snprintf(line, sizeof(line), "ON %lus", remainingSeconds);
  } else if (sensors.nextPumpRunHours >= 0.0f) {
    if (sensors.nextPumpRunHours <= 3.0f) {
      unsigned long remainingMinutes = static_cast<unsigned long>(ceilf(sensors.nextPumpRunHours * 60.0f));
      if (remainingMinutes == 0 && sensors.nextPumpRunHours > 0.0f) {
        remainingMinutes = 1;
      }
      snprintf(line, sizeof(line), "in %lum", remainingMinutes);
    } else {
      snprintf(line, sizeof(line), "in %.1f hrs", sensors.nextPumpRunHours);
    }
  } else {
    snprintf(line, sizeof(line), "OFF");
  }
  oled.drawStr(12, 54, line);

  if (pumpRowPulseOn) {
    oled.setDrawColor(1);
  }

  oled.sendBuffer();
}

void renderMenuScreen() {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "Menu");

  const uint8_t visibleRows = 5;
  for (uint8_t row = 0; row < visibleRows; ++row) {
    const uint8_t itemIndex = ui.menuScroll + row;
    if (itemIndex >= MENU_ITEM_COUNT) {
      break;
    }

    const uint8_t y = static_cast<uint8_t>(22 + (row * 10));
    if (itemIndex == ui.menuIndex) {
      oled.drawStr(0, y, ">");
    }
    oled.drawStr(10, y, MENU_ITEMS[itemIndex]);
  }

  oled.sendBuffer();
}

void renderDurationScreen() {
  char line[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "Duration");
  snprintf(line, sizeof(line), "%u min", ui.editDurationMinutes);
  oled.drawStr(0, 30, line);
  oled.drawStr(0, 48, "Tap select to save");
  oled.drawStr(0, 58, "Hold select back");
  oled.sendBuffer();
}

void renderUtcScreen() {
  char line[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "UTC Offset");
  snprintf(line, sizeof(line), "UTC %+d", ui.editUtcOffsetHours);
  oled.drawStr(0, 30, line);
  oled.drawStr(0, 48, "Tap select to save");
  oled.drawStr(0, 58, "Hold select back");
  oled.sendBuffer();
}

void renderBatteryOffsetScreen() {
  char line[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "Volt Offset");
  snprintf(line, sizeof(line), "%+.1f V", static_cast<float>(ui.editBatteryOffsetTenths) / 10.0f);
  oled.drawStr(0, 30, line);
  oled.drawStr(0, 48, "Tap select to save");
  oled.drawStr(0, 58, "Hold select back");
  oled.sendBuffer();
}

void renderManualScreen() {
  char line[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(0, 10, "Manual Run");
  snprintf(line, sizeof(line), "%u min", ui.editManualMinutes);
  oled.drawStr(0, 30, line);
  oled.drawStr(0, 48, "Tap select to run");
  oled.drawStr(0, 58, "Hold select cancel");
  oled.sendBuffer();
}

void renderSystemScreen() {
  char lines[6][24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tr);

  oled.drawStr(0, 8, "System");

  snprintf(lines[0], sizeof(lines[0]), "Boots: %lu", static_cast<unsigned long>(settings.restartCount));
  snprintf(lines[1], sizeof(lines[1]), "Crashes: %lu", static_cast<unsigned long>(settings.crashCount));
  snprintf(lines[2], sizeof(lines[2]), "Reset: %s", restartReasonText(startup.restartReason));
  formatLastDoseText(lines[3], sizeof(lines[3]), "Dose: ");
  formatNextDoseText(lines[4], sizeof(lines[4]), "Next: ");
  snprintf(lines[5], sizeof(lines[5]), "GPS:%s WDT:%ds", sensors.gpsHasFix ? "yes" : "no", watchdogTimeoutMs / 1000);

  const uint8_t visibleRows = 5;
  for (uint8_t row = 0; row < visibleRows; ++row) {
    const uint8_t lineIndex = ui.systemScroll + row;
    if (lineIndex >= 6) {
      break;
    }

    const uint8_t y = static_cast<uint8_t>(18 + (row * 10));
    oled.drawStr(0, y, lines[lineIndex]);
  }

  oled.sendBuffer();
}

void formatGpsSpinnerLine(char *buffer, size_t size, uint32_t spinnerTick) {
  static const char spinnerFrames[] = {'|', '/', '-', '\\'};
  const char frame = spinnerFrames[spinnerTick % (sizeof(spinnerFrames) / sizeof(spinnerFrames[0]))];

  if (gps.satellites.isValid()) {
    snprintf(buffer, size, "[%c] Sats:%lu", frame, static_cast<unsigned long>(gps.satellites.value()));
  } else {
    snprintf(buffer, size, "[%c] GPS acquiring...", frame);
  }
}

const char *restartReasonText(RestartReason reason) {
  switch (reason) {
    case RESTART_POWER_ON:
      return "POWER_ON";
    case RESTART_BACKUP:
      return "BACKUP";
    case RESTART_WATCHDOG:
      return "WATCHDOG";
    case RESTART_SOFTWARE:
      return "SOFTWARE";
    case RESTART_EXTERNAL:
      return "EXTERNAL";
    case RESTART_BROWNOUT:
      return "BROWNOUT";
    case RESTART_UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

void waitForStartupLineDwell() {
  if (startupLogCount == 0) {
    return;
  }

  while ((millis() - lastStartupLogMs) < STARTUP_LINE_MIN_MS) {
    delay(20);
  }
}

time_t localTimeNow() {
  return now() + (static_cast<long>(settings.utcOffsetHours) * SECS_PER_HOUR);
}

void syncClockFromGps() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return;
  }

  setTime(
    gps.time.hour(),
    gps.time.minute(),
    gps.time.second(),
    gps.date.day(),
    gps.date.month(),
    gps.date.year()
  );
}

void suppressGpsTimeSync() {
  startup.gpsTimeSuppressedUntilMs = millis() + GPS_MANUAL_TIME_HOLDOFF_MS;
}

void openMenu() {
  ui.menuIndex = 0;
  ui.menuScroll = 0;
  ui.screen = SCREEN_MENU;
  if (ui.displayOn) {
    renderMenuScreen();
  }
}

void openDurationEditor() {
  ui.editDurationMinutes = settings.runDurationMinutes;
  ui.screen = SCREEN_EDIT_DURATION;
  if (ui.displayOn) {
    renderDurationScreen();
  }
}

void openUtcEditor() {
  ui.editUtcOffsetHours = settings.utcOffsetHours;
  ui.screen = SCREEN_EDIT_UTC;
  if (ui.displayOn) {
    renderUtcScreen();
  }
}

void openBatteryOffsetEditor() {
  ui.editBatteryOffsetTenths = settings.batteryOffsetTenths;
  ui.screen = SCREEN_EDIT_VOFFSET;
  if (ui.displayOn) {
    renderBatteryOffsetScreen();
  }
}

void openManualEditor() {
  ui.editManualMinutes = settings.runDurationMinutes;
  ui.screen = SCREEN_EDIT_MANUAL;
  if (ui.displayOn) {
    renderManualScreen();
  }
}

void openSystemScreen() {
  ui.systemScroll = 0;
  ui.screen = SCREEN_SYSTEM;
  if (ui.displayOn) {
    renderSystemScreen();
  }
}

void noteUserActivity(unsigned long nowMs) {
  ui.lastActivityMs = nowMs;
  if (!ui.displayOn) {
    setDisplayPower(true);
  }
}

void setDisplayPower(bool enabled) {
  ui.displayOn = enabled;
  oled.setPowerSave(enabled ? 0 : 1);
  if (enabled) {
    lastUiUpdateMs = 0;
    renderIdleScreen();
  }
}
