#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>

class AC_PrecLand_RTK : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_RTK(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override { return true; }

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override { return AP_HAL::millis(); }

    // return true if there is a valid los measurement available
    bool have_los_meas() override { return true; }
};

