#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/I2CDevice.h>

namespace Linux {

class RCOutput_Mambo : public AP_HAL::RCOutput {
public:
    RCOutput_Mambo();

    static RCOutput_Mambo *from(AP_HAL::RCOutput *rcout) {
        return static_cast<RCOutput_Mambo*>(rcout);
    }

    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override;
private:
    uint16_t _period_us_to_rpm(uint16_t period_us);

    int actuators_fd;
    uint16_t _frequency;
    uint16_t _min_pwm;
    uint16_t _max_pwm;
    uint16_t _period_us[4];
    uint16_t _rpm_ref[4];
    bool _cork;
    bool _changed;
};

}
