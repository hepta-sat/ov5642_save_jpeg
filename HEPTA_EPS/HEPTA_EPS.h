#ifndef MBED_HEPTAEPS_H
#define MBED_HEPTAEPS_H
#include "HeptaBattery.h"

class HEPTA_EPS : public HeptaBattery{
public:
    //HeptaBattery battery;
    HEPTA_EPS(
            PinName bat,
            PinName bat_ct
    );
  
private:  
    //HeptaBattery _battery;
};

#endif