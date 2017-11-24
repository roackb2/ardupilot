#include "AC_LandProximity.h"

#define AUTO_INCREMENT          0xA0
#define REPEATED_BYTE           0x80

#define APDS9930_ENABLE         0x00
#define APDS9930_ATIME          0x01
#define APDS9930_PTIME          0x02
#define APDS9930_WTIME          0x03
#define APDS9930_AILTL          0x04
#define APDS9930_AILTH          0x05
#define APDS9930_AIHTL          0x06
#define APDS9930_AIHTH          0x07
#define APDS9930_PILTL          0x08
#define APDS9930_PILTH          0x09
#define APDS9930_PIHTL          0x0A
#define APDS9930_PIHTH          0x0B
#define APDS9930_PERS           0x0C
#define APDS9930_CONFIG         0x0D
#define APDS9930_PPULSE         0x0E
#define APDS9930_CONTROL        0x0F
#define APDS9930_ID             0x12
#define APDS9930_STATUS         0x13
#define APDS9930_Ch0DATAL       0x14
#define APDS9930_Ch0DATAH       0x15
#define APDS9930_Ch1DATAL       0x16
#define APDS9930_Ch1DATAH       0x17
#define APDS9930_PDATAL         0x18
#define APDS9930_PDATAH         0x19
#define APDS9930_POFFSET        0x1E

#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define SLEEP_AFTER_INT         6
#define ALL                     7

#define DEFAULT_ATIME           0xFF
#define DEFAULT_WTIME           0xFF
#define DEFAULT_PTIME           0xFF
#define DEFAULT_PPULSE          0x08
#define DEFAULT_POFFSET         0       // 0 offset
#define DEFAULT_CONFIG          0
#define DEFAULT_PDRIVE          0
#define DEFAULT_PDIODE          2
#define DEFAULT_PGAIN           2
#define DEFAULT_AGAIN           2
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x22 

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

bool AC_LandProximity::setAmbientLightGain(uint8_t drive)
{
    uint8_t val;
    uint8_t send = APDS9930_CONTROL | AUTO_INCREMENT;
    
    /* Read value from CONTROL register */
    if (!_dev->transfer(&send, 1, &val, 1)) { 
        return false;
    }
    
    /* Set bits in register to given value */
    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if (!_dev->write_register(APDS9930_CONTROL | AUTO_INCREMENT, val)) {
        return false;
    }
    
    return true;
}

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

bool AC_LandProximity::enableLightSensor()
{
    
    /* Set default gain, interrupts, enable power, and enable sensor */
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return false;
    }
    if( !setMode(POWER, 1) ){
        return false;
    }
    if( !setMode(AMBIENT_LIGHT, 1) ) {
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
    if( !setMode(POWER, 1) ){
        return false;
    }
    if( !setMode(PROXIMITY, 1) ) {
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
    uint8_t send = APDS9930_PDATAL | AUTO_INCREMENT;
    uint8_t val;
    uint16_t val_word;
    if (!_enabled) {
        proximity_val =0;
        proximity = false;
        return;
    }
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    val_word = val;
    send = APDS9930_PDATAH | AUTO_INCREMENT;
    if (!_dev->transfer(&send, 1, &val, 1)) return;
    val_word += ((uint16_t)val << 8);
    proximity_val = val_word;
    if (val_word > 900) proximity = true; else proximity = false;
}

void AC_LandProximity::init()
{
    _dev = hal.i2c_mgr->get_device(0, 0x39);
    if (_dev && _dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!checkId()) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!setMode(ALL, 0)) {
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
        if (!enableLightSensor()) {
            _dev->get_semaphore()->give();
            return;
        }
        if (!enableProximitySensor()) {
            _dev->get_semaphore()->give();
            return;
        }            
        _dev->get_semaphore()->give();
        _dev->register_periodic_callback(50000, FUNCTOR_BIND_MEMBER(&AC_LandProximity::timer, void));
    }    
}
