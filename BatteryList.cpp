#include "BatteryList.h"

#include <string.h>

constexpr size_t BatteryList::MAX_NR_BATTERIES;

bool BatteryList::addBattery(int batteryNr) {
    const unsigned int len = ::strlen(activatedBatteries);
    if (len == MAX_NR_BATTERIES) {
      return false;
    }
    activatedBatteries[len] = static_cast<char>(batteryNr);
    activatedBatteries[len + 1] = '\0';
    return true;
}

size_t BatteryList::count() const {
    return ::strlen(activatedBatteries);
}

bool BatteryList::contains(int batteryNr) const {
    return ::strchr(activatedBatteries, static_cast<char>(batteryNr));
}

int BatteryList::findAlternativeBatteryTo(int batteryNr) const {
    for (const char* pos = activatedBatteries;
         pos != activatedBatteries + ::strlen(activatedBatteries);
         ++pos) {
        const int otherActivatedBattery = static_cast<int>(*pos);
        if (otherActivatedBattery != batteryNr) {
            return otherActivatedBattery;
        }
    }
    return -1;
}

int BatteryList::operator[](size_t index) const {
    if (index >= ::strlen(activatedBatteries)) {
        return -1;
    }
    return static_cast<int>(activatedBatteries[index]);
}
