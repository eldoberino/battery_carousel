#ifndef BATTERYCAROUSEL_H
#define BATTERYCAROUSEL_H

#ifndef ARDUINO
  #include <ostream>
#endif

enum tMotorDirection {
  CW, STOP, CCW
};
extern const char *motorDirectionNames[];

class HwController {
public:
  HwController() : motorDirection_(tMotorDirection::STOP) { }
  virtual ~HwController() = default;

  virtual bool digitalRead(int pin) const = 0;
  virtual float analogRead(int pin) const = 0;

  void moveMotor(tMotorDirection motorDirection) const {
    motorDirection_ = motorDirection;
    doMoveMotor(motorDirection);
  }

  tMotorDirection getMotorDirection() const {
    return motorDirection_;
  }

private:
  virtual void doMoveMotor(tMotorDirection motorDirection) const = 0;

  mutable tMotorDirection motorDirection_;
};

class BatteryControl {
public:
  void updateValues(float voltage, bool isBatteryButtonReleased);

  const int batteryNr;
  const int isBatteryButtonReleasedPin;
  const int voltagePin;
//private:
  bool isBatteryButtonPushed;
  float voltage;
  int batteryLevel;
  bool isBatteryActive;
};

struct BatterySwitch {
  BatterySwitch(int theSourceBatteryNr, int theTargetBatteryNr);

  bool isNone() const {
    return sourceBatteryNr == targetBatteryNr;
  }

  int sourceBatteryNr;
  int targetBatteryNr;
  tMotorDirection motorDirection;
};

class BatterySelection {
public:
  BatterySelection(const BatteryControl* batteries);

  const BatterySwitch& update();

  /**
   * the battery currently in charge of providing power
   *
   * @return const BatteryControl&
   */
  const BatteryControl& getActiveBattery() const;

  void setVerbosity(bool isVerbose) { isVerbose_ = isVerbose; }

  const BatteryControl* begin() const { return &batteries_[0]; }
  const BatteryControl* end()   const { return &batteries_[3]; }

  const BatteryControl* cbegin() const { return &batteries_[0]; }
  const BatteryControl* cend()   const { return &batteries_[3]; }

private:
  void logSwitching();

  const BatteryControl* batteries_;
  mutable const BatteryControl* activeBattery_;
  BatterySwitch currentAction_;
  bool isVerbose_;
};

class BatteryCarousel {
public:
  BatteryCarousel(
    const HwController &hwController,
    const int *isBatteryButtonReleasedPins,
    const int *voltagePins,
    float adcToVolt,
    float voltageDivider
    );

  BatterySwitch readInputs();

  const BatteryControl& getActiveBattery() const;
  void setVerbosity(bool isVerbose) { batterySelection_.setVerbosity(isVerbose); }

  BatteryControl* begin() { return &batteries_[0]; }
  BatteryControl* end()   { return &batteries_[3]; }

  const BatteryControl* begin() const { return &batteries_[0]; }
  const BatteryControl* end()   const { return &batteries_[3]; }

  const BatteryControl* cbegin() const { return &batteries_[0]; }
  const BatteryControl* cend()   const { return &batteries_[3]; }

  BatteryControl& getBattery(int batteryNr) { return batteries_[batteryNr - 1]; }
  const BatteryControl& getBattery(int batteryNr) const { return batteries_[batteryNr - 1]; }

#ifndef ARDUINO
  friend std::ostream & operator<<(std::ostream &, const BatteryCarousel &);
#endif
private:

  const HwController& hwController_;
  BatteryControl batteries_[3];
  const float adcToVolt_;
  const float voltageDivider_;
  BatterySwitch batterySwitchInAction_;
  BatterySelection batterySelection_;
};


#ifndef ARDUINO
std::ostream & operator<<(std::ostream &os, tMotorDirection dir);
std::ostream & operator<<(std::ostream &os, const BatteryControl &b);
std::ostream & operator<<(std::ostream &os, const BatterySwitch &b);
std::ostream & operator<<(std::ostream &os, const BatteryCarousel &b);
#endif

#endif

