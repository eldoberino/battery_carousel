#include <gtest/gtest.h>
#include <unordered_map>
#include "BatteryCarousel.h"
#include "MovingAverage.h"
#include "StableDigitalInput.h"
#include "BatteryList.h"

static const int isBatteryButtonReleasedPins[] = {6, 4, 2};
static const int voltagePins[] = {14 /*A0*/, 16 /*A2*/, 17 /*A3*/};
static const float adcToVolt = 1.0;  // 5.0 / 1023.0;
static const float voltageDivider = 1.0;  //6.8 / (22.0 + 6.8);

class MockHwController : public HwController {
public:
    MockHwController()
    : digital_(), analog_()
    {
    }

    bool digitalRead(int pin) const override {
        auto it = digital_.find(pin);
        return it != digital_.end() ? it->second : false;
    }

    float analogRead(int pin) const override {
        auto it = analog_.find(pin);
        return it != analog_.end() ? it->second : 0.0;
    }

    // my helpers
    void setBattery(int batteryNr, float voltage, bool isButtonPushed) {
      analog_[voltagePins[batteryNr-1]] = voltage;
      digital_[isBatteryButtonReleasedPins[batteryNr-1]] = !isButtonPushed;
    }

    void setBatteryVoltage(int batteryNr, float voltage) {
        analog_[voltagePins[batteryNr-1]] = voltage;
    }

private:
    void doMoveMotor(tMotorDirection motorDirection) const override {
    }

    std::unordered_map<int, bool> digital_;
    std::unordered_map<int, float> analog_;
};

// fixture
class BatteryCarouselTest : public ::testing::Test {
protected:
  BatteryCarouselTest()
  : carousel(hwController,
             isBatteryButtonReleasedPins,
             voltagePins,
             adcToVolt,
             voltageDivider) {
  }

  MockHwController hwController;
  BatteryCarousel carousel;
};



TEST(BatteryList, AddingBatteryIncreasesTheCount) {
  BatteryList batteryList;
  EXPECT_EQ(batteryList.count(), 0);

  batteryList.addBattery(1);
  EXPECT_EQ(batteryList.count(), 1);
}

TEST(BatteryList, AllowsCheckingIfBatteryIsListed) {
  BatteryList batteryList;

  batteryList.addBattery(42);

  EXPECT_TRUE(batteryList.contains(42));
  EXPECT_FALSE(batteryList.contains(43));
}

TEST(BatteryList, AllowsFindingAlternativeBattery) {
  BatteryList batteryList;

  batteryList.addBattery(42);

  EXPECT_EQ(batteryList.findAlternativeBatteryTo(42), -1);
  EXPECT_EQ(batteryList.findAlternativeBatteryTo(1), 42);

  batteryList.addBattery(43);
  EXPECT_EQ(batteryList.findAlternativeBatteryTo(42), 43);
  EXPECT_EQ(batteryList.findAlternativeBatteryTo(43), 42);
}

TEST(BatteryList, StopsAddingWhenFull) {
  BatteryList batteryList;
  for (int i=1; i <= BatteryList::MAX_NR_BATTERIES; ++i) {
    EXPECT_TRUE(batteryList.addBattery(i));
    EXPECT_EQ(batteryList.count(), i);
  }
  EXPECT_FALSE(batteryList.addBattery(77));
  EXPECT_EQ(batteryList.count(), BatteryList::MAX_NR_BATTERIES);
}

TEST(BatteryList, SupportsAccessByIndex) {
  BatteryList batteryList;

  batteryList.addBattery(42);
  batteryList.addBattery(43);

  EXPECT_EQ(batteryList[0], 42);
  EXPECT_EQ(batteryList[1], 43);
  EXPECT_EQ(batteryList[2], -1);
  EXPECT_EQ(batteryList[BatteryList::MAX_NR_BATTERIES+1], -1);
}



TEST_F(BatteryCarouselTest, ItReadsInputFromHardwareAndShowsCurrentState) {
    hwController.setBattery(1, 28.4, false);
    hwController.setBattery(2, 28.8, true);
    hwController.setBattery(3, 28.9, false);
    carousel.readInputs();

    const auto& activeBattery = carousel.getActiveBattery();
    EXPECT_EQ(activeBattery.batteryNr, 2);
    EXPECT_FLOAT_EQ(activeBattery.voltage, 28.8);
}

TEST_F(BatteryCarouselTest, ItProvidesBatteryFillLevel) {
    hwController.setBattery(1, 5*3.2 - 0.1, false);
    hwController.setBattery(2, 5*(4.2+3.2)/2., true);
    hwController.setBattery(3, 5*4.2 + 0.1, false);

    carousel.readInputs();

    EXPECT_EQ(carousel.getBattery(1).batteryLevel, 0);
    EXPECT_EQ(carousel.getBattery(2).batteryLevel, 50);
    EXPECT_EQ(carousel.getBattery(3).batteryLevel, 100);
}

TEST_F(BatteryCarouselTest, SupportsIterating) {
  EXPECT_EQ(std::distance(carousel.begin(), carousel.end()), 3);
  for (const BatteryControl& battery : carousel) {
    EXPECT_EQ(isBatteryButtonReleasedPins[battery.batteryNr-1], battery.isBatteryButtonReleasedPin);
    EXPECT_EQ(voltagePins[battery.batteryNr-1], battery.voltagePin);
  }
}

TEST_F(BatteryCarouselTest, WhenCurrentBatteryBelowThresholdNextBatteryGetsChosen) {
  hwController.setBattery(1, 5*4.2, true);
  hwController.setBattery(2, 5*4.2, false);
  hwController.setBattery(3, 0, false);

  BatterySwitch action = carousel.readInputs();
  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 1);
  EXPECT_TRUE(action.isNone());

  hwController.setBatteryVoltage(1, 5*3.2);
  action = carousel.readInputs();
  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 1);  // still active although below voltage threshold
  EXPECT_EQ(hwController.getMotorDirection(), tMotorDirection::CCW);
  EXPECT_EQ(action.sourceBatteryNr, 1);
  EXPECT_EQ(action.targetBatteryNr, 2);
  EXPECT_EQ(action.motorDirection, tMotorDirection::CCW);

  // for a short time 2 batteries are connected -> both active & same voltage
  hwController.setBattery(1, 5*(3.2+4.2)/2, true);
  hwController.setBattery(2, 5*(3.2+4.2)/2, true);
  action = carousel.readInputs();
  EXPECT_EQ(hwController.getMotorDirection(), tMotorDirection::CCW);
  EXPECT_EQ(action.sourceBatteryNr, 1);
  EXPECT_EQ(action.targetBatteryNr, 2);
  EXPECT_EQ(action.motorDirection, tMotorDirection::CCW);

  hwController.setBattery(1, 5*3.2, false);
  hwController.setBattery(2, 5*4.2, true);
  action = carousel.readInputs();

  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 2);
  EXPECT_EQ(hwController.getMotorDirection(), tMotorDirection::STOP);
  EXPECT_TRUE(action.isNone());
}

TEST_F(BatteryCarouselTest, WhenActiveBatteryGetsRemovedAnotherOneTakesOver) {
  hwController.setBattery(1, 5*4.2, true);
  hwController.setBattery(2, 5*3.2, false);
  hwController.setBattery(3, 5*4.0, false);
  carousel.readInputs();
  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 1);

  hwController.setBattery(1, 0.0, false);
  BatterySwitch action = carousel.readInputs();

  EXPECT_EQ(hwController.getMotorDirection(), tMotorDirection::CW);
  EXPECT_EQ(action.sourceBatteryNr, 1);
  EXPECT_EQ(action.targetBatteryNr, 3);
  EXPECT_EQ(action.motorDirection, tMotorDirection::CW);

  hwController.setBattery(3, 5*4.0, true);
  action = carousel.readInputs();
  EXPECT_EQ(hwController.getMotorDirection(), tMotorDirection::STOP);
  EXPECT_EQ(action.sourceBatteryNr, 3);
  EXPECT_EQ(action.targetBatteryNr, 3);
  EXPECT_EQ(action.motorDirection, tMotorDirection::STOP);
  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 3);
}

TEST_F(BatteryCarouselTest, WhenTwoBatteriesAreActiveThenDecideOnOne) {
  MockHwController hwController;
  BatteryCarousel carousel(hwController, isBatteryButtonReleasedPins, voltagePins, adcToVolt, voltageDivider);

  hwController.setBattery(1, 5*4.2, true);
  hwController.setBattery(2, 5*4.2, true);
  hwController.setBattery(3, 5*4.0, false);
  BatterySwitch action = carousel.readInputs();

  EXPECT_FALSE(action.isNone());
}

TEST_F(BatteryCarouselTest, WhenFirstBatteryGetsConnectedItGetsChosen) {
  hwController.setBattery(1, 0, false);
  hwController.setBattery(2, 0, false);
  hwController.setBattery(3, 0, false);
  BatterySwitch action = carousel.readInputs();
  EXPECT_TRUE(action.isNone());

  hwController.setBattery(2, 5*4.2, false);
  action = carousel.readInputs();
  EXPECT_FALSE(action.isNone());
  EXPECT_EQ(action.targetBatteryNr, 2);
}

TEST_F(BatteryCarouselTest, WhenAllBatteriesGetRemovedItChoosesFirstOneThatGetsReconnected) {
  hwController.setBattery(1, 0, false);
  hwController.setBattery(2, 5*4.2, true);
  hwController.setBattery(3, 0, false);
  BatterySwitch action = carousel.readInputs();
  EXPECT_TRUE(action.isNone());
  EXPECT_EQ(carousel.getActiveBattery().batteryNr, 2);

  hwController.setBatteryVoltage(2, 0.0);
  action = carousel.readInputs();
  EXPECT_TRUE(action.isNone());
//EXPECT_EQ(carousel.getActiveBattery().batteryNr, -1);  // TODO: select but no voltage -> still considered active?

  hwController.setBattery(1, 5*4.2, false);
  action = carousel.readInputs();
  EXPECT_FALSE(action.isNone());
  EXPECT_EQ(action.targetBatteryNr, 1);
}

TEST(BatterySwitch, ItDeterminesDirectionFromCurrentBatteryToTargetBattery) {
  EXPECT_EQ(BatterySwitch(1, 3).motorDirection, tMotorDirection::CW);
  EXPECT_EQ(BatterySwitch(3, 1).motorDirection, tMotorDirection::CCW);
  EXPECT_EQ(BatterySwitch(1, 2).motorDirection, tMotorDirection::CCW);
  EXPECT_EQ(BatterySwitch(2, 1).motorDirection, tMotorDirection::CW);

  EXPECT_EQ(BatterySwitch(1, 1).motorDirection, tMotorDirection::STOP);
  EXPECT_TRUE(BatterySwitch(1, 1).isNone());
}

TEST(MovingAverage, ItReturnsAverageOverPreviousNValues) {
  MovingAverage ma3(3);
  EXPECT_DOUBLE_EQ(ma3.update(1.0), 1.0);
  EXPECT_DOUBLE_EQ(ma3.update(2.0), 1.5);
  EXPECT_DOUBLE_EQ(ma3.update(3.0), 2.0);
  EXPECT_DOUBLE_EQ(ma3.update(4.0), 3.0);
  EXPECT_DOUBLE_EQ(ma3.update(5.0), 4.0);
}

TEST(StableDigitalInput, ItOnlyChangesStateWhenAllValuesAreEqual) {
  StableDigitalInput stableDigital(3);
  EXPECT_EQ(stableDigital.update(true), true);
  EXPECT_EQ(stableDigital.update(true), true);
  EXPECT_EQ(stableDigital.update(false), true);
  EXPECT_EQ(stableDigital.update(false), true);
  EXPECT_EQ(stableDigital.update(false), false);
  EXPECT_EQ(stableDigital.update(true), false);
  EXPECT_EQ(stableDigital.update(true), false);
  EXPECT_EQ(stableDigital.update(true), true);
}

TEST(BatterySelection, ItDeterminesDirectionFromCurrentBatteryToTargetBattery) {
  // currently active battery, selected battery, should generate action if required
  // distinguished between buttons that got pushed and where position currently is
  BatteryControl batteries[3] = {
    {1, isBatteryButtonReleasedPins[0], voltagePins[0], false, 0.0f, 0, false},
    {2, isBatteryButtonReleasedPins[1], voltagePins[1], false, 0.0f, 0, false},
    {3, isBatteryButtonReleasedPins[2], voltagePins[2], false, 0.0f, 0, false}
  };
  BatterySelection selector(batteries);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, -1);

  batteries[1].updateValues(5*4.2, true);
  selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);

  // connect battery 3
  batteries[2].updateValues(5*4.2, false);
  selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);

  // battery 3 button gets pressed (for whatever reason)
  // -> must turn back to battery 2
  batteries[2].updateValues(5*4.2, true);
  const BatterySwitch& action1 = selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);
  EXPECT_FALSE(action1.isNone());
  EXPECT_EQ(action1.targetBatteryNr, 2);
  EXPECT_EQ(action1.sourceBatteryNr, 3);

  // battery 3 button unpressed again
  // -> no action
  batteries[2].updateValues(5*4.2, false);
  const BatterySwitch& action2 = selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);
  EXPECT_TRUE(action2.isNone());

  // battery 2 voltage goes down
  // -> must switch to battery 3
  batteries[1].updateValues(5*3.2, true);
  const BatterySwitch& action3 = selector.update();
  EXPECT_FALSE(action3.isNone());
  EXPECT_EQ(action3.sourceBatteryNr, 2);
  EXPECT_EQ(action3.targetBatteryNr, 3);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);

  // battery 2 & 3 buttons are pushed
  // -> batteries are connected
  batteries[1].updateValues(5*(3.2+4.2)/2, true);
  batteries[2].updateValues(5*(3.2+4.2)/2, true);
  const BatterySwitch& action4 = selector.update();
  EXPECT_FALSE(action4.isNone());
  EXPECT_EQ(action4.sourceBatteryNr, 2);
  EXPECT_EQ(action4.targetBatteryNr, 3);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);

  // battery 2 button no longer pushed
  // -> only battery 3 active
  batteries[1].updateValues(5*3.2, false);
  batteries[2].updateValues(5*4.2, true);
  const BatterySwitch& action5 = selector.update();
  EXPECT_TRUE(action5.isNone());
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 3);
}

TEST(BatterySelection, WhenFirstBatteryGetsConnectedItGetsChosen) {
  BatteryControl batteries[3] = {
    {1, isBatteryButtonReleasedPins[0], voltagePins[0], false, 0.0f, 0, false},
    {2, isBatteryButtonReleasedPins[1], voltagePins[1], false, 0.0f, 0, false},
    {3, isBatteryButtonReleasedPins[2], voltagePins[2], false, 0.0f, 0, false}
  };
  BatterySelection selector(batteries);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, -1);

  // another battery button is pushed but no voltage -> not detectable
  batteries[1].updateValues(5*4.2, false);
  const BatterySwitch& action = selector.update();
  EXPECT_FALSE(action.isNone());
  EXPECT_EQ(selector.getActiveBattery().batteryNr, -1); // still none -> only when destination reached

  batteries[1].updateValues(5*4.2, true);
  const BatterySwitch& action2 = selector.update();
  EXPECT_TRUE(action2.isNone());
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);
}

TEST(BatterySelection, WhenTwoBatteryButtonsArePushedThenConsiderVoltageAndDecideOnOne_DirectionCCW) {
  BatteryControl batteries[3] = {
    {1, isBatteryButtonReleasedPins[0], voltagePins[0], false, 5*4.2, 0, true},
    {2, isBatteryButtonReleasedPins[1], voltagePins[1], false, 5*3.2, 0, false},
    {3, isBatteryButtonReleasedPins[2], voltagePins[2], false, 0.0f, 0, false}
  };
  BatterySelection selector(batteries);
  selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 1);

  batteries[0].updateValues(5*(4.2+3.2)/2, true);
  batteries[1].updateValues(5*(4.2+3.2)/2, true);
  const BatterySwitch& action = selector.update();
  EXPECT_FALSE(action.isNone());
  EXPECT_EQ(action.targetBatteryNr, 1);
  EXPECT_EQ(action.sourceBatteryNr, 2);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 1);
}

TEST(BatterySelection, WhenTwoBatteryButtonsArePushedThenConsiderVoltageAndDecideOnOne_DirectionCW) {
  BatteryControl batteries[3] = {
    {1, isBatteryButtonReleasedPins[0], voltagePins[0], false, 5*3.2, 0, false},
    {2, isBatteryButtonReleasedPins[1], voltagePins[1], false, 5*4.2, 0, true},
    {3, isBatteryButtonReleasedPins[2], voltagePins[2], false, 0.0f, 0, false}
  };
  BatterySelection selector(batteries);
  selector.update();
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);

  batteries[0].updateValues(5*(4.2+3.2)/2, true);
  batteries[1].updateValues(5*(4.2+3.2)/2, true);
  const BatterySwitch& action = selector.update();
  EXPECT_FALSE(action.isNone());
  EXPECT_EQ(action.targetBatteryNr, 2);
  EXPECT_EQ(action.sourceBatteryNr, 1);
  EXPECT_EQ(selector.getActiveBattery().batteryNr, 2);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

