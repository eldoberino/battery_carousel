#include <Servo.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BatteryCarousel.h"
#include "MovingAverage.h"
#include "StableDigitalInput.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:  A4(SDA), A5(SCL)
#define OLED_RESET     -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const int POLL_INTERVAL_MS = 100;
static const int isBatteryButtonReleasedPins[] = {6, 4, 2};
static const int voltagePins[] = {A0, A2, A3};
static const int zeroSpeedPosition = 1210;
static const float voltageDivider = 18. / (120. + 18.) * 23.65 /*displayed*/ / 20.21 /*real input*/;
static const unsigned long DISPLAY_INTERVAL_MS = 1000U;

static const unsigned short MOVING_AVERAGE_WINDOW_SIZE = 3U;
static const int dim[] = {25, (display.height()-2*2)/3};
static const float adcToVolt = 3.3 / 1023.0;

Servo driveServo;

class ArduinoHwController : public HwController {
public:
  ArduinoHwController()
  : analogAverages_{
      MovingAverage(MOVING_AVERAGE_WINDOW_SIZE),
      MovingAverage(MOVING_AVERAGE_WINDOW_SIZE),
      MovingAverage(MOVING_AVERAGE_WINDOW_SIZE)
    }, stableDigitals_{
      StableDigitalInput(MOVING_AVERAGE_WINDOW_SIZE),
      StableDigitalInput(MOVING_AVERAGE_WINDOW_SIZE),
      StableDigitalInput(MOVING_AVERAGE_WINDOW_SIZE)
    }
  {}

  bool digitalRead(int pin) const override {
    int idx = pinToIndex(pin, isBatteryButtonReleasedPins);
    if (idx < 0) {
      return false;
    }
    return stableDigitals_[idx].update(::digitalRead(pin) == 1);
  }

  float analogRead(int pin) const override {
    int idx = pinToIndex(pin, voltagePins);
    if (idx < 0) {
      return 0.0f;
    }
    return analogAverages_[idx].update(::analogRead(pin));
  }

  void init() {
    for (int i=0; i != 3; ++i) {
      ::pinMode(voltagePins[i], INPUT);
      ::pinMode(isBatteryButtonReleasedPins[i], INPUT);
    }
  }

private:
  void doMoveMotor(tMotorDirection motorDirection) const override {
    const int speed = motorDirection == tMotorDirection::STOP ? 0 :
      (motorDirection == tMotorDirection::CCW ? 60 : -40);
    Serial.print(F("Speed "));
    Serial.print(abs(speed));
    speed > 0 ? Serial.println(" CCW") : Serial.println(" CW");

    if (motorDirection != tMotorDirection::STOP) {
      driveServo.attach(9, 1000, 2000);
    }
    driveServo.writeMicroseconds(zeroSpeedPosition+speed);
    if (motorDirection == tMotorDirection::STOP) {
      driveServo.detach();
    }
  }

  int pinToIndex(int pin, const int* pinDefinitions) const {
    for (int i = 0; i < 3; ++i) {
      if (pinDefinitions[i] == pin) {
        return i;
      }
    }
    return -1;
  }

  mutable MovingAverage analogAverages_[3];
  mutable StableDigitalInput stableDigitals_[3];
};


ArduinoHwController hwController;
BatteryCarousel carousel(hwController, isBatteryButtonReleasedPins, voltagePins, adcToVolt, voltageDivider);

void setup() {
  analogReference(DEFAULT);
  Serial.begin(9600);
  hwController.init();

  hwController.moveMotor(tMotorDirection::STOP);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.print(display.width());
  Serial.print('x');
  Serial.print(display.height());
  Serial.println("px");

  display.clearDisplay();
  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  for(int16_t i=0; i != 3; ++i) {
    display.setCursor(0, i*(dim[1]+2));
    display.print(F("Bat"));
    display.write('0'+i+1);
    display.drawRect(25, i*(dim[1]+2), dim[0], dim[1], SSD1306_WHITE);
  }
  display.display();
}

static void logBatteryStatus(const BatteryControl& battery) {
  Serial.print(F("Battery "));
  Serial.print(battery.batteryNr);
  Serial.print(F(" (Pin "));
  Serial.print(battery.voltagePin);
  Serial.print(F("): is "));
  Serial.print(battery.isBatteryActive ? "active" : "inactive");
  Serial.print(F(", value: "));
  Serial.print(battery.voltage);
  Serial.println('V');
}

void updateDisplay(const BatteryControl& battery) {
  const int i = battery.batteryNr-1;
  const int batLevel = battery.batteryLevel / 100.0 * dim[0];
  display.fillRect(25+1, i*(dim[1]+2)+1, batLevel, dim[1]-2, SSD1306_WHITE);
  display.fillRect(25+1+batLevel, i*(dim[1]+2)+1, dim[0]-batLevel-2, dim[1]-2, SSD1306_BLACK);

  // clear previous value
  display.fillRect(25+dim[0]+2, i*(dim[1]+2), 25+dim[0]+20, dim[1]-2, SSD1306_BLACK);

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25+dim[0]+2, i*(dim[1]+2));
  if (battery.voltage < 10.) {
    display.print(' ');
  }
  display.print(battery.voltage);
  display.print('V');

  if (battery.isBatteryActive) {
    display.print(F(" <="));
  } else {
    display.print(F("   "));
  }
}

void logBatterySwitch(const BatterySwitch& action) {
  Serial.print(F("Switching battery "));
  Serial.print(motorDirectionNames[static_cast<int>(action.motorDirection)]);
  Serial.print(F(" from "));
  Serial.print(action.sourceBatteryNr);
  Serial.print(F(" to "));
  Serial.println(action.targetBatteryNr);
}


static unsigned long nextUpdateTs = 0;

void loop() {

  const BatterySwitch action = carousel.readInputs();
  if (!action.isNone()) {
//  logBatterySwitch(action);
  }
  if (millis() > nextUpdateTs) {
    for (const BatteryControl &battery : carousel) {
      // logBatteryStatus(battery);
      updateDisplay(battery);
    }
    display.display();
    nextUpdateTs = millis() + DISPLAY_INTERVAL_MS;
  }

  if (Serial.available()) {
    const char cmd = Serial.read();

    if (cmd == 'b') {
      int targetBattery = Serial.parseInt();
      const BatterySwitch batterySwitch(carousel.getActiveBattery().batteryNr, targetBattery);
      if (batterySwitch.isNone()) {
        Serial.print(F("Already at battery: "));
        Serial.println(targetBattery);
      } else {
        Serial.print(F("Manually "));
        logBatterySwitch(batterySwitch);
        hwController.moveMotor(batterySwitch.motorDirection);
        delay(1000);
        hwController.moveMotor(tMotorDirection::STOP);
      }
    } else if (cmd == 'v') {
      int verbosity = Serial.parseInt();
      carousel.setVerbosity(verbosity != 0);
    }
  } else {
    delay(POLL_INTERVAL_MS);
  }
}
