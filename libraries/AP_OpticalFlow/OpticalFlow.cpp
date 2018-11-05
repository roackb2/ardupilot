#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OpticalFlow.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF &ahrs)
    : _ahrs(ahrs),
      _last_update_ms(0)
{
    // healthy flag will be overwritten on update
    _flags.healthy = false;
}

void OpticalFlow::init(void)
{
}

void OpticalFlow::update(void)
{
}

