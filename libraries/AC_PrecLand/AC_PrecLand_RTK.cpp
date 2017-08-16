#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_RTK.h"

// Constructor
AC_PrecLand_RTK::AC_PrecLand_RTK(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_RTK::init()
{
    _state.healthy = true;
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_RTK::update()
{
    _state.healthy = true;
}

