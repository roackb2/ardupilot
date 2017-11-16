#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_LandProximity
{
public:
    AC_LandProximity();
    void init();
    bool proximity;
    uint16_t proximity_val;
    
    static const struct AP_Param::GroupInfo var_info[];
private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t getMode();
    bool setMode(uint8_t mode, uint8_t enable);
    bool enableLightSensor();
    bool enableProximitySensor();
    bool setAmbientLightGain(uint8_t drive);
    bool setProximityGain(uint8_t drive);
    bool setLEDDrive(uint8_t drive);
    bool setProximityDiode(uint8_t drive);
    bool checkId();
    void timer();
    
    AP_Int8 _enabled;
};
