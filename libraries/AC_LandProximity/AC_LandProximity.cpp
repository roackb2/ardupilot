#include "AC_LandProximity.h"

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AC_LandProximity::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: enable land proximity
    // @Description: enable land proximity
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED", 0, AC_LandProximity, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AC_LandProximity::AC_LandProximity()
{
}

#if 0
uint8_t AC_LandProximity::getMode()
{
    uint8_t enable_value;
    uint8_t send = APDS9930_ENABLE | AUTO_INCREMENT;
    /* Read current ENABLE register */
    if (_dev->transfer(&send, 1, &enable_value, 1)) return enable_value;
    return 0xff;
}
bool AC_LandProximity::setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == 0xff ) {
        return false;
    }
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if(mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
        
    /* Write value back to ENABLE register */
    if (_dev->write_register(APDS9930_ENABLE | AUTO_INCREMENT, reg_val)) return true; 
    return false;
}
#endif

bool AC_LandProximity::setProximityGain(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AC_LandProximity::setLEDDrive(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AC_LandProximity::setProximityDiode(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) {
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    drive = drive << 4;
    val &= 0b11001111;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

bool AC_LandProximity::enableProximitySensor()
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return false;
    }
    if( !setLEDDrive(DEFAULT_PDRIVE) ) {
        return false;
    }
    if (!_dev->write_register(APDS9930_ENABLE | AUTO_INCREMENT, 0x25)) {//power, proximity, proximity interrupt
        return false;
    }
    
    return true;
}

bool AC_LandProximity::checkId() {
    uint8_t send = APDS9930_ID | AUTO_INCREMENT;
    uint8_t val;
    if (_dev->transfer(&send, 1, &val, 1) && val == 0x39) return true;
    return false;
}

void AC_LandProximity::timer() {
/*    
    uint8_t send = APDS9930_PDATAL | AUTO_INCREMENT;
    uint8_t val;
    uint16_t val_word;
    proximity_val =0;
    proximity = false;
    if (!_enabled) return;
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    val_word = val;
    send = APDS9930_PDATAH | AUTO_INCREMENT;
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    val_word += ((uint16_t)val << 8);
    proximity_val = val_word;
    if (val_word > 900) proximity = true; else proximity = false;
*/
    uint8_t send = APDS9930_STATUS | AUTO_INCREMENT;
    uint8_t val;
    //proximity_val = 0;
    proximity = false;
    if (!_enabled) return;
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    if (val & 0x20) {
        proximity = true;
        //proximity_val = 1;
        send = CLEAR_PROX_INT;
        _dev->transfer(&send, 1, 0, 0);
    }
}

bool AC_LandProximity::setProximityIntHighThreshold(uint16_t threshold)
{
    uint8_t lo;
    uint8_t hi;
    hi = threshold >> 8;
    lo = threshold & 0x00FF;

    if( !_dev->write_register(APDS9930_PIHTL | AUTO_INCREMENT, lo) ) {
        return false;
    }
    if( !_dev->write_register(APDS9930_PIHTH | AUTO_INCREMENT, hi) ) {
        return false;
    }
    
    return true;
}

void AC_LandProximity::init()
{
    _dev = hal.i2c_mgr->get_device(0, 0x39);
    if (_dev && _dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!checkId()) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_ENABLE | AUTO_INCREMENT, 0)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_ATIME | AUTO_INCREMENT, DEFAULT_ATIME)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_WTIME | AUTO_INCREMENT, DEFAULT_WTIME)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_PPULSE | AUTO_INCREMENT, DEFAULT_PPULSE)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_POFFSET | AUTO_INCREMENT, DEFAULT_POFFSET)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!_dev->write_register(APDS9930_CONFIG | AUTO_INCREMENT, DEFAULT_CONFIG)) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!setProximityDiode(DEFAULT_PDIODE)) {
            _dev->get_semaphore()->give();
            return;
        }
        if( !setProximityIntHighThreshold(DEFAULT_PIHT) ) {
            _dev->get_semaphore()->give();
            return;
        }
        if( !_dev->write_register(APDS9930_PERS | AUTO_INCREMENT, DEFAULT_PERS) ) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!enableProximitySensor()) {
            _dev->get_semaphore()->give();
            return;
        }            
        _dev->get_semaphore()->give();
        _dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AC_LandProximity::timer, void));
    }    
}
