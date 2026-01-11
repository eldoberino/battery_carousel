#pragma once

#include <stddef.h>

class BatteryList {
public:
    bool addBattery(int batteryNr);
    size_t count() const;
    bool contains(int batteryNr) const;
    int findAlternativeBatteryTo(int batteryNr) const;

    int operator[](size_t index) const;

    static constexpr size_t MAX_NR_BATTERIES = 3;
private:
    char activatedBatteries[MAX_NR_BATTERIES + 1U]{ '\0' };
};

