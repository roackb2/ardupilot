#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_IRLock_RTK.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLock_RTK::AC_PrecLand_IRLock_RTK(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock(),
      _have_los_meas(false),
      _los_meas_time_ms(0),
      _use_rtk(false)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock_RTK::init()
{
    irlock.init(get_bus());
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_IRLock_RTK::update()
{
    // update health
    _state.healthy = irlock.healthy();
    
    // get new sensor data
    irlock.update();
    
    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms) {
        irlock.get_unit_vector_body(_los_meas_body);
        _have_los_meas = true;
        _los_meas_time_ms = irlock.last_update_ms();
    }
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_IRLock_RTK::get_los_body(Vector3f& ret) {
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_IRLock_RTK::los_meas_time_ms() {
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_IRLock_RTK::have_los_meas() {
    return _have_los_meas;
}

void AC_PrecLand_IRLock_RTK::handle_msg(mavlink_message_t* msg)
{
    // parse mavlink message
    __mavlink_landing_target_t packet;
    mavlink_msg_landing_target_decode(msg, &packet);

    if (packet.frame == 100) _use_rtk = true; else _use_rtk = false;
}

