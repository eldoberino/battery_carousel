#include <string.h>
#include "BatteryCarousel.h"
#include "BatteryList.h"

static const int nrCells = 5;
static const float minCellVoltage = 3.2;
static const float maxCellVoltage = 4.2;
static const float BATTERY_LOW_LEVEL_VOLTAGE = nrCells * (minCellVoltage + 0.1);

const char* motorDirectionNames[] = { "CW", "STOP", "CCW" };  // sync with tMotorDirection

#ifdef ARDUINO

  #include <Arduino.h>

  struct SerialLogger {
    void print(const char* s) { ::Serial.print(s); }
    void print(int v) { ::Serial.print(v); }
    void print(float v) { ::Serial.print(v); }

    void println(const char* s) { ::Serial.println(s); }
    void println(int v) { ::Serial.println(v); }
    void println(float v) { ::Serial.println(v); }
  };
  SerialLogger logger;

#else

  #include <iostream>

  struct ConsoleLogger {
    void print(const char* s) { std::cout << s; }
    void print(int v) { std::cout << v; }
    void print(float v) { std::cout << v; }

    void println(const char* s) { std::cout << s << '\n'; }
    void println(int v) { std::cout << v << '\n'; }
    void println(float v) { std::cout << v << '\n'; }
  };
  ConsoleLogger logger;
#endif

static void logSwitching(const BatterySwitch& action, bool withNewline = false) {
  logger.print("Switching from battery ");
  logger.print(action.sourceBatteryNr);
  logger.print(" to ");
  withNewline ? logger.println(action.targetBatteryNr) : logger.print(action.targetBatteryNr);
}


static tMotorDirection calculateDirection(int activeBatteryNr, int targetBatteryNr) {
  if (activeBatteryNr == targetBatteryNr) {
    return tMotorDirection::STOP;
  }
  //  cw: 1 -> 2, 2 -> 3, 3 -> 1
  // ccw: 1 -> 3, 2 -> 1, 3 -> 2
  const bool isCCW = (activeBatteryNr % 3 + 1) == targetBatteryNr;
  return isCCW ? tMotorDirection::CCW : tMotorDirection::CW;
}

BatterySwitch::BatterySwitch(int theSourceBatteryNr, int theTargetBatteryNr)
: sourceBatteryNr(theSourceBatteryNr), targetBatteryNr(theTargetBatteryNr),
  motorDirection(calculateDirection(theSourceBatteryNr, theTargetBatteryNr))
{}


BatterySelection::BatterySelection(const BatteryControl* batteries)
: batteries_(batteries), activeBattery_(nullptr),
  currentAction_(-1, -1), isVerbose_(false)
{
}

const BatterySwitch& BatterySelection::update() {
  BatteryList activatedBatteries;
  static const BatteryControl nullBattery{-1, -1, -1, false, 0.0f, 0, false};
  const BatteryControl* batteryCandidate = &nullBattery;
  for (auto& battery : *this) {
    if (battery.isBatteryActive) {
      // active : button pushed
      // init currentAction_ & activeBattery_ if needed
      if (currentAction_.targetBatteryNr == -1) {
        currentAction_ = BatterySwitch(battery.batteryNr, battery.batteryNr);
      }
      if (!activeBattery_) {
        activeBattery_ = &battery;
      }
      activatedBatteries.addBattery(battery.batteryNr);
    } else if (battery.voltage > 1.1*BATTERY_LOW_LEVEL_VOLTAGE) {
      batteryCandidate = &battery;
    }
  }
  const size_t nrActivatedBatteries = activatedBatteries.count();
  if (isVerbose_) {
    logger.print("#activated batteries: [");
    for (size_t i=0; i != nrActivatedBatteries; ++i) {
      logger.print(activatedBatteries[i]);
      if (i != nrActivatedBatteries-1) {
        logger.print(", ");
      }
    }
    logger.print("], batteryCandidate: Battery ");
    logger.print(batteryCandidate->batteryNr);
    logger.print(" @ ");
    logger.print(batteryCandidate->voltage);
    logger.println("V");
  }
  if (!currentAction_.isNone()) {
    // action finished?
    if (batteries_[currentAction_.targetBatteryNr - 1].isBatteryButtonPushed &&
        !batteries_[currentAction_.sourceBatteryNr - 1].isBatteryButtonPushed) {
      // hwController_.moveMotor(tMotorDirection::STOP);
      logger.print("Done switching from Battery ");
      logger.print(currentAction_.sourceBatteryNr);
      logger.print(" to ");
      logger.print(currentAction_.targetBatteryNr);
      logger.println(" -> MUST stop now");
      activeBattery_ = &batteries_[currentAction_.targetBatteryNr - 1];
      currentAction_ = BatterySwitch(activeBattery_->batteryNr, activeBattery_->batteryNr);
    }
    return currentAction_;
  }
  if (nrActivatedBatteries == 0) {
    if (!activeBattery_) {
      // let's check if a battery got connected with enough voltage
      if (batteryCandidate != &nullBattery) {
        currentAction_ = BatterySwitch(-1, batteryCandidate->batteryNr);
        logSwitching();
        logger.println(" due to: no active battery before");
      } else {
        logger.println("no battery candidate, yet");
      }
    } else if (activeBattery_->voltage < BATTERY_LOW_LEVEL_VOLTAGE) {
      // voltage < threshold -> marked as not active anymore
      // no ongoing action: activeBattery with still enough voltage?
      if (batteryCandidate->batteryNr != activeBattery_->batteryNr && batteryCandidate->voltage > 1.1*BATTERY_LOW_LEVEL_VOLTAGE) {
        currentAction_ = BatterySwitch(activeBattery_->batteryNr, batteryCandidate->batteryNr);
        logSwitching();
        logger.print(" due to: only ");
        logger.print(activeBattery_->voltage);
        logger.println("V");
      }
    } else if (batteryCandidate != &nullBattery) {
      currentAction_ = BatterySwitch(nullBattery.batteryNr, batteryCandidate->batteryNr);
      logSwitching();
      logger.println(" due to: no active battery");
    }
  } else if (nrActivatedBatteries == 1) {
    if (activeBattery_->batteryNr != activatedBatteries[0]) {
      logger.print("For whatever reason: Active Battery switched from ");
      logger.print(activeBattery_->batteryNr);
      logger.print(" to ");
      logger.println(activatedBatteries[0]);
      activeBattery_ = &batteries_[activatedBatteries[0] - 1];
    }

    if (activeBattery_->voltage < BATTERY_LOW_LEVEL_VOLTAGE) {
      if (batteryCandidate != &nullBattery) {
        currentAction_ = BatterySwitch(activeBattery_->batteryNr, batteryCandidate->batteryNr);
        logSwitching();
        logger.print(" due to: only ");
        logger.print(activeBattery_->voltage);
        logger.println("V");
      } else {
        logger.println("BUMMER: low voltage & no battery candidate");
      }
    }
  } else if (nrActivatedBatteries > 1) {
    // multiple battery buttons pressed
    // if no action active -> make sure only 1 is active
    if (activatedBatteries.contains(activeBattery_->batteryNr)) {
      const int otherActivatedBattery = activatedBatteries.findAlternativeBatteryTo(activeBattery_->batteryNr);
      if (otherActivatedBattery != -1) {
        logger.print("For whatever reason multiple batteries active -> release battery ");
        logger.print(otherActivatedBattery);
        logger.print(" & keep ");
        logger.println(activeBattery_->batteryNr);
        currentAction_ = BatterySwitch(otherActivatedBattery, activeBattery_->batteryNr);
      }
    }
  }
  return currentAction_;
}

void BatterySelection::logSwitching() {
  ::logSwitching(currentAction_);
}

const BatteryControl& BatterySelection::getActiveBattery() const {
  static const BatteryControl nullBattery{ -1, -1, -1, false, 0.0f, false };
  return activeBattery_ ? *activeBattery_ : nullBattery;
}

void BatteryControl::updateValues(float newVoltage, bool isButtonPushed) {
  voltage = newVoltage;
  // isBatteryButtonReleasedPin could wrongly be 0 if input voltage too small
  isBatteryButtonPushed = voltage > 3.3 && isButtonPushed;
  isBatteryActive = isBatteryButtonPushed /* && voltage > BATTERY_LOW_LEVEL_VOLTAGE*/;
  const int newBatteryLevel = (voltage - nrCells*minCellVoltage)/(nrCells*maxCellVoltage-nrCells*minCellVoltage) * 100;
  batteryLevel = newBatteryLevel > 100 ? 100 : (newBatteryLevel < 0 ? 0 : newBatteryLevel);
}


BatteryCarousel::BatteryCarousel(
    const HwController& hwController,
    const int* isBatteryButtonReleasedPins,
    const int* voltagePins,
    float adcToVolt,
    float voltageDivider)
: hwController_(hwController),
  batteries_{
    {1, isBatteryButtonReleasedPins[0], voltagePins[0], false, 0.0f, 0, false},
    {2, isBatteryButtonReleasedPins[1], voltagePins[1], false, 0.0f, 0, false},
    {3, isBatteryButtonReleasedPins[2], voltagePins[2], false, 0.0f, 0, false}
  },
  adcToVolt_(adcToVolt),
  voltageDivider_(voltageDivider),
  batterySwitchInAction_(BatterySwitch(-1, -1)),
  batterySelection_(batteries_)
{
  logger.print("Battery low level limit @ ");
  logger.print(BATTERY_LOW_LEVEL_VOLTAGE);
  logger.println("V");
}

BatterySwitch BatteryCarousel::readInputs() {
  for (auto& battery : batteries_) {
    battery.updateValues(
      hwController_.analogRead(battery.voltagePin) * adcToVolt_ / voltageDivider_,
      !hwController_.digitalRead(battery.isBatteryButtonReleasedPin)
    );
  }
  //std::cout << "batterySwitchInAction_: " << batterySwitchInAction_
  //  << ", activeBattery_: " << (activeBattery_ ? activeBattery_->batteryNr : -1)
  //  << '\n';
  const BatterySwitch oldBatteryAction = batterySwitchInAction_;
  batterySwitchInAction_ = batterySelection_.update();
  if (batterySwitchInAction_.isNone() && !oldBatteryAction.isNone()) {
    logger.print("Done ");
    ::logSwitching(oldBatteryAction, true);
    hwController_.moveMotor(tMotorDirection::STOP);
  } else if (!batterySwitchInAction_.isNone() && oldBatteryAction.isNone()) {
    logger.print("Starting ");
    ::logSwitching(batterySwitchInAction_, true);
    hwController_.moveMotor(batterySwitchInAction_.motorDirection);
  }
  return batterySwitchInAction_;
}


const BatteryControl& BatteryCarousel::getActiveBattery() const {
  return batterySelection_.getActiveBattery();
}

#ifndef ARDUINO
# include <iostream>

  std::ostream& operator<<(std::ostream& os, tMotorDirection dir) {
    static const char* motorDirectionNames[] = { "CW", "STOP", "CCW" };
    os << motorDirectionNames[static_cast<int>(dir)];
    return os;
  }

  std::ostream& operator<<(std::ostream& os, const BatteryControl& b) {
      os << "BatteryControl{"
         << "batteryNr=" << b.batteryNr
         << ", isBatteryButtonReleasedPin=" << b.isBatteryButtonReleasedPin
         << ", voltagePin=" << b.voltagePin
         << ", isBatteryButtonPushed=" << b.isBatteryButtonPushed
         << ", voltage=" << b.voltage
         << ", batteryLevel=" << b.batteryLevel
         << ", isBatteryActive=" << b.isBatteryActive
         << '}';
      return os;
  }

  std::ostream& operator<<(std::ostream& os, const BatterySwitch& b) {
      os << "BatterySwitch{";
      if (b.isNone()) {
        os << "None}";
      } else {
        os << "from=" << b.sourceBatteryNr
           << ", to=" << b.targetBatteryNr
           << ", direction=" << b.motorDirection
           << '}';
      }
      return os;
  }

  std::ostream& operator<<(std::ostream& os, const BatteryCarousel& b) {
      os << "BatteryCarousel{"
         << "batterySwitchInAction=" << b.batterySwitchInAction_
         << ", motor direction:" << b.hwController_.getMotorDirection() << ",\n";
      for (auto& battery : b.batteries_) {
        os << "- " << battery << '\n';
      }
      os << '}';
      return os;
  }
#endif
